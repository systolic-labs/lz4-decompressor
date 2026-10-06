// ============================================================================
//  lz4_axi.v - LZ4 decoder behind an AXI4 master stream engine
// ----------------------------------------------------------------------------
//  Step 4 of doc/lz4_dma_engine.md.  Same buffered engine as lz4_stream.v, but
//  the two byte ports are replaced by a real AXI4 master: the compressed source
//  is fetched as 64-bit words from memory, and the decompressed output is
//  packed into 64-bit words and written back.  The CPU only programmes
//  src/dst/len and watches done - three accesses per block instead of ~3.6 M.
//
//  Single-beat 64-bit reads and writes (arlen/awlen = 0).  The DDR4 budget says
//  bursts buy nothing here: 11.1 MB of output is 1.39 M writes at ~5 cycles =
//  0.14 s, against the decoder's 0.43 s - the decoder stays the limit.  Bursts
//  are a later optimisation if the DDR4 path turns out to cost more.
//
//  The final output word is written with a byte strobe (a partial word), which
//  is why the output FIFO entries carry {strobe, data}.
//
//  The read FSM issues one AR at a time and parks until R; the write FSM issues
//  AW then W then waits for B.  Fixed ID 0 - the spike has one master.
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

    // control (MMIO registers in the SoC)
    input  logic          start,
    input  logic [63:0]   src_addr,
    input  logic [63:0]   dst_addr,
    input  logic [31:0]   ilen,
    output logic          busy,
    output logic          done,
    output logic          error,
    output logic [31:0]   out_count,

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
    // History window (decoder contract: registered read).
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

    // ------------------------------------------------------------------
    // Input: AXI read FSM -> word FIFO -> byte feed to the decoder
    // ------------------------------------------------------------------
    logic [63:0]     iwmem [0:IFW-1];
    logic [IPW-1:0]  iwwp, iwrp;
    logic [IPW:0]    iwcnt;
    logic [31:0]     rd_word;        // next word index to fetch
    logic            rd_state;       // 0 = idle, 1 = waiting for R
    logic            rd_busy;

    assign m_arid    = '0;
    assign m_arlen   = 8'd0;
    assign m_arsize  = 3'd3;         // 8 bytes
    assign m_arburst = 2'd1;         // INCR
    assign m_araddr  = src_addr + {rd_word, 3'b000};
    assign m_arvalid = (rd_state == 1'b0) && busy && (rd_word < in_words)
                       && (iwcnt < IFW[IPW:0]);
    assign m_rready  = 1'b1;

    wire iw_full = (iwcnt >= IFW[IPW:0]);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_state <= 1'b0; rd_busy <= 1'b0; rd_word <= 32'd0;
            iwwp <= '0; iwrp <= '0; iwcnt <= '0;
        end else begin
            case (rd_state)
            1'b0: if (m_arvalid && m_arready) begin
                      rd_state <= 1'b1;
                  end
            1'b1: if (m_rvalid) begin
                      iwmem[iwwp] <= m_rdata;
                      iwwp        <= iwwp + 1'b1;
                      rd_word     <= rd_word + 32'd1;
                      rd_state    <= 1'b0;
                  end
            endcase
            // iwcnt counts words WAITING to be loaded (the word in ish is not
            // counted), so it goes up on a fetch and down when ish takes one.
            // One assignment: two separate ifs would drop the +1 whenever a
            // fetch and a load land in the same cycle.
            iwcnt <= iwcnt + ((rd_state == 1'b1 && m_rvalid) ? 1'b1 : 1'b0)
                          - (feed_load ? 1'b1 : 1'b0);
        end
    end

    // ---- byte feed: shift the head word out one byte at a time ----
    logic [63:0] ish;
    logic [3:0]  iw_rem;             // valid bytes left in ish
    logic [31:0] ip_taken;
    wire  iw_avail  = (iwcnt != 0);
    wire  feed_last = (iw_rem == 4'd1);
    wire  feed_take = (iw_rem != 0) && (ip_taken < ilen);
    wire  in_valid  = busy && feed_take;
    wire [7:0] in_data = ish[7:0];
    wire  in_last   = (ip_taken == (ilen - 32'd1));
    wire  in_ready;
    wire  dec_takes = in_valid && in_ready;
    // Need the next word when ish is empty, or in the same cycle the last byte
    // of the current word is consumed - that overlap is what removes the bubble
    // between words.
    wire  need_word = (iw_rem == 4'd0) || (dec_takes && feed_last);
    wire  feed_load = need_word && iw_avail;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
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
    logic [71:0]     owmem [0:OFW-1];   // {strobe[7:0], data[63:0]}
    logic [OPW-1:0]  owwp, owrp;
    logic [OPW:0]    owcnt;
    logic [63:0]     osh;
    logic [3:0]      onb;               // bytes packed in osh, 0..7
    logic            flushed;
    logic [31:0]     ow_word;           // write word index

    wire ow_full = (owcnt >= OFW[OPW:0]);
    // Accept a byte if it lands mid-word, or if completing a word there is a
    // slot for it.  The +out_valid counts the byte already in flight.
    assign out_ready = (((onb + {3'b0, out_valid}) < 4'd7) || !ow_full);

    wire pack_push = (out_valid && out_ready && (onb == 4'd7));
    wire flush_req = dec_done && (onb != 4'd0) && !flushed && (owcnt < OFW[OPW:0]);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            osh <= 64'd0; onb <= 4'd0; owwp <= '0; owrp <= '0; owcnt <= '0;
            ow_word <= 32'd0; out_count <= 32'd0; flushed <= 1'b0;
        end else if (busy) begin
            if (out_valid && out_ready) begin
                out_count <= out_count + 32'd1;
                if (onb == 4'd7) begin
                    owmem[owwp] <= {8'hFF, osh | ({56'd0, out_data} << 56)};
                    owwp   <= owwp + 1'b1;
                    owcnt  <= owcnt + 1'b1;
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
                owcnt   <= owcnt + 1'b1;
                osh     <= 64'd0;
                onb     <= 4'd0;
                flushed <= 1'b1;
            end
            // write FSM pops one word
            if (wr_pop) owcnt <= owcnt - 1'b1;
        end
    end

    // ---- write FSM: AW -> W -> B, one word at a time ----
    logic [1:0] wst;
    logic [63:0] wsh;
    logic [7:0]  wstrb_q;
    localparam [1:0] W_IDLE = 2'd0, W_AW = 2'd1, W_W = 2'd2, W_B = 2'd3;

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
            wst <= W_IDLE; wsh <= 64'd0; wstrb_q <= 8'hFF;
        end else case (wst)
        W_IDLE: if (owcnt != 0) begin
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
        endcase
        if (wr_pop) owrp <= owrp + 1'b1;
    end

    // ------------------------------------------------------------------
    // Completion: decoder done, packer empty, FIFO empty, write FSM idle.
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
                started <= 1'b1; done_q <= 1'b0; error_q <= 1'b0; flushed <= 1'b0;
                rd_word <= 32'd0; ow_word <= 32'd0; ip_taken <= 32'd0;
                iwcnt <= '0; iwwp <= '0; iwrp <= '0;
                owcnt <= '0; owwp <= '0; owrp <= '0;
                ish <= 64'd0; iw_rem <= 4'd0; osh <= 64'd0; onb <= 4'd0;
            end
        end
    end

endmodule
