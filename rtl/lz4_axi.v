// ============================================================================
//  lz4_axi.v - LZ4 decoder behind an AXI4 master stream engine
// ----------------------------------------------------------------------------
//  The compressed source is fetched from memory and the decompressed output is
//  packed into 64-bit words and written back.  The CPU programmes src/dst/len
//  and watches done.
//
//  WHY BURSTS.  Single-beat reads sustained back-to-back gave a
//  non-deterministic short output on silicon (klen 00a94aba / 00a94489 /
//  00a41542 against 00a95600) while the engine's own counters were exact
//  (read words == ceil(len/8), fed == len, err == 0): the data was wrong, not
//  the bookkeeping.  The only other reader on this crossbar, the CPU, reads
//  slowly and byte-wise; the only other *bulk* reader, the DMA, moves bursts.
//  So the read is now issued as a burst of up to RBURST beats - the DMA's
//  proven pattern - instead of a stream of single beats.
//
//  READ vs WRITE.  A read transaction is still not started while a write
//  transaction is in flight, and vice versa (the engine used to interleave
//  them, which nothing else on this bridge does).
//
//  CHECKS.  dbg_sum XORs every source word the engine reads; the loader
//  computes the same XOR over the source with the CPU and compares.  If the
//  two differ, the read path is corrupting data; if they agree, the fault is
//  further in.
//
//  Every register has exactly ONE always block that drives it.
// ============================================================================
module lz4_axi #(
    parameter integer AXI_AW  = 64,
    parameter integer AXI_DW  = 64,
    parameter integer AXI_IDW = 4,
    parameter integer IFW     = 16,   // input word FIFO depth, 64-bit words
    parameter integer OFW     = 16    // output word FIFO depth
) (
    input  logic          clk,
    input  logic          rst_n,

    input  logic          start,
    input  logic [63:0]   src_addr,
    input  logic [63:0]   dst_addr,
    input  logic [31:0]   ilen,
    output logic          busy,
    output logic          done,
    output logic          error,
    output logic [31:0]   out_count,

    // debug: counters and the source XOR
    output logic [31:0]   dbg_rd,
    output logic [31:0]   dbg_wr,
    output logic [31:0]   dbg_feed,
    output logic [31:0]   dbg_err,
    output logic [63:0]   dbg_sum,

    // AXI4 master
    output logic [AXI_IDW-1:0]  m_arid,
    output logic [AXI_AW-1:0]   m_araddr,
    output logic [7:0]          m_arlen,
    output logic [2:0]          m_arsize,
    output logic [1:0]          m_arburst,
    output logic                m_arvalid,
    input  logic                m_arready,
    input  logic [AXI_IDW-1:0]  m_rid,
    input  logic [AXI_DW-1:0]   m_rdata,
    input  logic [1:0]          m_rresp,
    input  logic                m_rlast,
    input  logic                m_rvalid,
    output logic                m_rready,
    output logic [AXI_IDW-1:0]  m_awid,
    output logic [AXI_AW-1:0]   m_awaddr,
    output logic [7:0]          m_awlen,
    output logic [2:0]          m_awsize,
    output logic [1:0]          m_awburst,
    output logic                m_awvalid,
    input  logic                m_awready,
    output logic [AXI_DW-1:0]   m_wdata,
    output logic [AXI_DW/8-1:0] m_wstrb,
    output logic                m_wlast,
    output logic                m_wvalid,
    input  logic                m_wready,
    input  logic [AXI_IDW-1:0]  m_bid,
    input  logic [1:0]          m_bresp,
    input  logic                m_bvalid,
    output logic                m_bready
);

    localparam integer IPW = $clog2(IFW);
    localparam integer OPW = $clog2(OFW);

    // ------------------------------------------------------------------
    // History window
    // ------------------------------------------------------------------
    (* ram_style = "block" *) logic [7:0] win [0:65535];
    logic [15:0] win_waddr, win_raddr;
    logic [7:0]  win_wdata, win_rdata, win_rdata_q;
    logic        win_we;
    assign win_rdata = win_rdata_q;
    always @(posedge clk) begin
        if (win_we) win[win_waddr] <= win_wdata;
        win_rdata_q <= win[win_raddr];
    end

    // ------------------------------------------------------------------
    // Control / status
    // ------------------------------------------------------------------
    logic started, done_q, error_q;
    assign busy  = started & ~done_q & ~error_q;
    assign done  = done_q;
    assign error = error_q;

    wire [31:0] in_words = (ilen + 32'd7) >> 3;

    // write FSM state, declared early: the read FSM's arvalid depends on it
    localparam [1:0] W_IDLE = 2'd0, W_AW = 2'd1, W_W = 2'd2, W_B = 2'd3;
    logic [1:0] wst;
    wire  wr_inflight = (wst != W_IDLE);

    wire rd_err = (rd_state == 1'b1) && m_rvalid && (m_rresp != 2'b00);
    wire wr_err = wr_pop && (m_bresp != 2'b00);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)     dbg_err <= 32'd0;
        else if (start) dbg_err <= 32'd0;
        else            dbg_err <= dbg_err + (rd_err ? 32'd1 : 32'd0)
                                          + (wr_err ? 32'd1 : 32'd0);
    end

    // ------------------------------------------------------------------
    // Input: AXI read -> word FIFO -> byte feed.  SINGLE BEAT: a burst version
    // was tried and hung on silicon after 4 words (the write side stalled and
    // the read/write exclusion then blocked the reads), so the read is one
    // 64-bit beat at a time.  dbg_sum XORs every word read, so the loader can
    // compare the engine's view of the source with the CPU's.
    // ------------------------------------------------------------------
    logic [63:0]     iwmem [0:IFW-1];
    logic [IPW-1:0]  iwwp, iwrp;
    logic [IPW:0]    iwcnt;
    logic [31:0]     rd_word;        // next word index to fetch
    logic            rd_state;       // 0 = idle, 1 = waiting for R

    wire rd_inflight = (rd_state == 1'b1);

    assign m_arid    = '0;
    assign m_arlen   = 8'd0;
    assign m_arsize  = 3'd3;                     // 8 bytes
    assign m_arburst = 2'd1;                     // INCR
    assign m_araddr  = src_addr + {rd_word, 3'b000};
    assign m_arvalid = (rd_state == 1'b0) && busy && (rd_word < in_words)
                       && (iwcnt < IFW[IPW:0]) && !wr_inflight;
    assign m_rready  = 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_state <= 1'b0; rd_word <= 32'd0; iwwp <= '0; iwcnt <= '0;
            dbg_rd <= 32'd0; dbg_sum <= 64'd0;
        end else if (start) begin
            rd_state <= 1'b0; rd_word <= 32'd0; iwwp <= '0; iwcnt <= '0;
            dbg_rd <= 32'd0; dbg_sum <= 64'd0;
        end else begin
            if (rd_state == 1'b0) begin
                if (m_arvalid && m_arready) rd_state <= 1'b1;
            end else if (m_rvalid) begin
                iwmem[iwwp] <= m_rdata;
                iwwp        <= iwwp + 1'b1;
                rd_word     <= rd_word + 32'd1;
                dbg_rd      <= dbg_rd + 32'd1;
                dbg_sum     <= dbg_sum ^ m_rdata;
                rd_state    <= 1'b0;
            end
            // iwcnt = words WAITING to be loaded, one assignment only
            iwcnt <= iwcnt + ((rd_state == 1'b1 && m_rvalid) ? 1'b1 : 1'b0)
                          - (feed_load ? 1'b1 : 1'b0);
        end
    end

    // ---- byte feed: shift the head word out one byte at a time ----
    logic [63:0] ish;
    logic [3:0]  iw_rem;
    logic [31:0] ip_taken;
    wire  iw_avail  = (iwcnt != 0);
    wire  feed_last = (iw_rem == 4'd1);
    wire  feed_take = (iw_rem != 0) && (ip_taken < ilen);
    wire  in_valid  = busy && feed_take;
    wire [7:0] in_data = ish[7:0];
    wire  in_last   = (ip_taken == (ilen - 32'd1));
    wire  in_ready;
    wire  dec_takes = in_valid && in_ready;
    wire  need_word = (iw_rem == 4'd0) || (dec_takes && feed_last);
    wire  feed_load = need_word && iw_avail;

    assign dbg_feed = ip_taken;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ish <= 64'd0; iw_rem <= 4'd0; ip_taken <= 32'd0; iwrp <= '0;
        end else if (start) begin
            ish <= 64'd0; iw_rem <= 4'd0; ip_taken <= 32'd0; iwrp <= '0;
        end else if (busy) begin
            if (dec_takes) ip_taken <= ip_taken + 32'd1;
            if (feed_load) begin
                ish    <= iwmem[iwrp];
                iwrp   <= iwrp + 1'b1;
                iw_rem <= 4'd8;
            end else if (dec_takes && (iw_rem != 4'd0)) begin
                ish    <= ish >> 8;
                iw_rem <= iw_rem - 4'd1;
            end
        end
    end

    // ------------------------------------------------------------------
    // Decoder
    // ------------------------------------------------------------------
    wire [7:0] out_data;
    wire       out_valid, out_ready;
    wire       dec_done, dec_error;

    lz4_decoder u_dec (
        .clk(clk), .rst_n(rst_n),
        .in_data(in_data), .in_valid(in_valid), .in_last(in_last),
        .in_ready(in_ready),
        .out_data(out_data), .out_valid(out_valid), .out_ready(out_ready),
        .win_waddr(win_waddr), .win_wdata(win_wdata), .win_we(win_we),
        .win_raddr(win_raddr), .win_rdata(win_rdata),
        .done(dec_done), .error(dec_error)
    );

    // ------------------------------------------------------------------
    // Output: pack bytes into words -> word FIFO -> AXI write FSM
    // ------------------------------------------------------------------
    logic [71:0]     owmem [0:OFW-1];
    logic [OPW-1:0]  owwp;
    logic [OPW:0]    owcnt;
    logic [63:0]     osh;
    logic [3:0]      onb;
    logic            flushed;
    logic [63:0]     wsh;
    logic [7:0]      wstrb_q;
    logic [OPW-1:0]  owrp;
    logic [31:0]     ow_word;

    wire ow_full = (owcnt >= OFW[OPW:0]);
    assign out_ready = (((onb + {3'b0, out_valid}) < 4'd7) || !ow_full);

    wire flush_req = dec_done && (onb != 4'd0) && !flushed && (owcnt < OFW[OPW:0]);
    wire pack_done = out_valid && out_ready && (onb == 4'd7);

    assign m_awid    = '0;
    assign m_awlen   = 8'd0;
    assign m_awsize  = 3'd3;
    assign m_awburst = 2'd1;
    assign m_awaddr  = dst_addr + {ow_word, 3'b000};
    assign m_awvalid = (wst == W_AW);
    assign m_wdata   = wsh;
    assign m_wstrb   = wstrb_q;
    assign m_wlast   = 1'b1;
    assign m_wvalid  = (wst == W_W);
    assign m_bready  = 1'b1;

    wire wr_pop = (wst == W_B) && m_bvalid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            osh <= 64'd0; onb <= 4'd0; owwp <= '0; owcnt <= '0;
            out_count <= 32'd0; flushed <= 1'b0;
        end else if (start) begin
            osh <= 64'd0; onb <= 4'd0; owwp <= '0; owcnt <= '0;
            out_count <= 32'd0; flushed <= 1'b0;
        end else if (busy) begin
            if (out_valid && out_ready) begin
                out_count <= out_count + 32'd1;
                if (onb == 4'd7) begin
                    owmem[owwp] <= {8'hFF, osh | ({56'd0, out_data} << 56)};
                    owwp   <= owwp + 1'b1;
                    osh    <= 64'd0;
                    onb    <= 4'd0;
                end else begin
                    osh <= osh | ({56'd0, out_data} << {onb, 3'b000});
                    onb <= onb + 4'd1;
                end
            end
            if (flush_req) begin
                owmem[owwp] <= {(8'hFF >> (8 - onb)), osh};
                owwp    <= owwp + 1'b1;
                osh     <= 64'd0;
                onb     <= 4'd0;
                flushed <= 1'b1;
            end
            owcnt <= owcnt + (pack_done ? 1'b1 : 1'b0)
                          + (flush_req ? 1'b1 : 1'b0)
                          - (wr_pop    ? 1'b1 : 1'b0);
        end
    end

    // ---- write FSM: AW -> W -> B, one word at a time ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wst <= W_IDLE; wsh <= 64'd0; wstrb_q <= 8'hFF;
            ow_word <= 32'd0; owrp <= '0; dbg_wr <= 32'd0;
        end else if (start) begin
            wst <= W_IDLE; wsh <= 64'd0; wstrb_q <= 8'hFF;
            ow_word <= 32'd0; owrp <= '0; dbg_wr <= 32'd0;
        end else begin
            case (wst)
            // no write starts while a read transaction is in flight, nor in the
            // same cycle a read address handshake is taking place
            W_IDLE: if ((owcnt != 0) && !rd_inflight
                        && !(m_arvalid && m_arready)) begin
                        wsh     <= owmem[owrp][63:0];
                        wstrb_q <= owmem[owrp][71:64];
                        wst     <= W_AW;
                    end
            W_AW:  if (m_awready) wst <= W_W;
            W_W:   if (m_wready)  wst <= W_B;
            W_B:   if (m_bvalid) begin
                        wst     <= W_IDLE;
                        ow_word <= ow_word + 32'd1;
                    end
            default: wst <= W_IDLE;
            endcase
            if (wr_pop) begin
                owrp   <= owrp + 1'b1;
                dbg_wr <= dbg_wr + 32'd1;
            end
        end
    end

    // ------------------------------------------------------------------
    // Completion
    // ------------------------------------------------------------------
    wire all_done = dec_done && (onb == 4'd0) && (owcnt == 0)
                    && (wst == W_IDLE) && !flush_req;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            started <= 1'b0; done_q <= 1'b0; error_q <= 1'b0;
        end else begin
            if (dec_error) error_q <= 1'b1;
            if (all_done)  done_q  <= 1'b1;
            if (start) begin
                started <= 1'b1; done_q <= 1'b0; error_q <= 1'b0;
            end
        end
    end

endmodule
