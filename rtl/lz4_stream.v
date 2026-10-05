// ============================================================================
//  lz4_stream.v - LZ4 decoder behind a buffered stream engine (no MMIO)
// ----------------------------------------------------------------------------
//  Spike / template for the engine path in doc/lz4_dma_engine.md.  The CPU is
//  out of the byte loop entirely: a source port reads the compressed bytes from
//  an external memory into an input FIFO, the decoder eats from that FIFO, and
//  an output FIFO drains the decompressed bytes to an external memory.
//
//  It exists to answer one question before any AXI work: does a buffered engine,
//  with a real (1-cycle) memory latency and finite FIFOs, still reach the
//  decoder's ceiling of ~2 cycles per output byte, or does the buffering cost
//  eat it?  The source/sink here are generic byte ports; in the SoC they become
//  an AXI4 master reading/writing DDR4.
//
//  Control is three signals: pulse start with ilen, then watch done/error.  That
//  is the whole CPU cost of a block.
//
//  Memory contract:
//    * source: present src_raddr with src_valid; src_rdata holds that byte on
//      the next cycle with src_rvalid high (a registered read).
//    * sink:   dst_waddr/dst_wdata are presented with dst_we for one cycle; the
//      memory writes on the rising edge.
// ============================================================================
module lz4_stream #(
    parameter integer IFIFO = 32,    // input FIFO depth, bytes
    parameter integer OFIFO = 32     // output FIFO depth, bytes
) (
    input  logic         clk,
    input  logic         rst_n,

    // control (in the SoC these are MMIO registers)
    input  logic         start,      // pulse: begin a block
    input  logic [31:0]  ilen,       // compressed block length
    output logic         busy,
    output logic         done,
    output logic         error,
    output logic [31:0]  out_count,  // decompressed bytes produced

    // source memory port (compressed bytes in)
    output logic [31:0]  src_raddr,
    output logic         src_valid,
    input  logic         src_rvalid,
    input  logic [7:0]   src_rdata,

    // sink memory port (decompressed bytes out)
    output logic [31:0]  dst_waddr,
    output logic [7:0]   dst_wdata,
    output logic         dst_we
);

    localparam integer IPW = (IFIFO <= 2) ? 1 : $clog2(IFIFO);
    localparam integer OPW = (OFIFO <= 2) ? 1 : $clog2(OFIFO);

    // ------------------------------------------------------------------
    // History window: 64 KiB, registered read (the decoder's contract).
    // ------------------------------------------------------------------
    (* ram_style = "block" *) logic [7:0] win [0:65535];
    logic [15:0] win_waddr, win_raddr;
    logic [7:0]  win_wdata, win_rdata;
    logic        win_we;
    logic [7:0]  win_rdata_q;

    assign win_rdata = win_rdata_q;
    always @(posedge clk) begin
        if (win_we) win[win_waddr] <= win_wdata;
        win_rdata_q <= win[win_raddr];
    end

    // ------------------------------------------------------------------
    // Input side: fetch into a FIFO, feed the decoder.
    // ------------------------------------------------------------------
    logic [7:0]      i_fifo [0:IFIFO-1];
    logic [IPW-1:0]  i_wp, i_rp;
    logic [7:0]      i_cnt;          // bytes currently in the input FIFO
    logic [31:0]     ip_issued;      // bytes requested from source
    logic [31:0]     ip_taken;       // bytes handed to the decoder
    logic [7:0]      inflight;       // source reads in flight (0..1 typical)

    wire i_room   = ({2'b0, i_cnt} + {2'b0, inflight}) < IFIFO[7:0];
    wire i_want   = (ip_issued < ilen) && i_room;
    assign src_valid = busy && i_want;
    assign src_raddr = ip_issued;
    wire i_done   = (ip_issued >= ilen) && (inflight == 8'd0);

    wire in_valid = (i_cnt != 8'd0);
    wire [7:0] in_data = i_fifo[i_rp];
    wire in_last  = (ip_taken == (ilen - 32'd1));
    wire in_ready;
    wire dec_takes = in_valid && in_ready;

    // ------------------------------------------------------------------
    // Output side: decode into a FIFO, drain to the sink.
    // ------------------------------------------------------------------
    logic [7:0]      o_fifo [0:OFIFO-1];
    logic [OPW-1:0]  o_wp, o_rp;
    logic [7:0]      o_cnt;

    wire out_valid, out_ready;
    wire [7:0] out_data;

    // Room for the decoder's next byte, counting the one already in flight -
    // the same test the MMIO wrapper uses, and for the same reason: out_valid is
    // registered, so a plain (o_cnt < OFIFO) lets one byte too many be offered
    // and it is dropped without a flag.
    assign out_ready = ((o_cnt + {7'd0, out_valid}) < OFIFO[7:0]);
    wire dec_done, dec_error;

    lz4_decoder u_dec (
        .clk(clk), .rst_n(rst_n),
        .in_data(in_data), .in_valid(in_valid), .in_last(in_last),
        .in_ready(in_ready),
        .out_data(out_data), .out_valid(out_valid), .out_ready(out_ready),
        .win_waddr(win_waddr), .win_wdata(win_wdata), .win_we(win_we),
        .win_raddr(win_raddr), .win_rdata(win_rdata),
        .done(dec_done), .error(dec_error)
    );

    wire o_push  = out_valid;                 // out_ready is high whenever room
    wire o_drain = (o_cnt != 8'd0);
    assign dst_we    = busy && o_drain;
    assign dst_waddr = out_count;
    assign dst_wdata = o_fifo[o_rp];

    assign busy = started & ~done & ~error;

    logic started;
    logic done_q, error_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_wp <= '0; i_rp <= '0; i_cnt <= 8'd0;
            ip_issued <= 32'd0; ip_taken <= 32'd0; inflight <= 8'd0;
            o_wp <= '0; o_rp <= '0; o_cnt <= 8'd0;
            out_count <= 32'd0;
            started <= 1'b0; done_q <= 1'b0; error_q <= 1'b0;
        end else begin
            // ---- source fetch ----
            if (src_valid) ip_issued <= ip_issued + 32'd1;
            if (src_rvalid) begin
                i_fifo[i_wp] <= src_rdata;
                i_wp         <= i_wp + 1'b1;
            end
            inflight <= inflight + (src_valid ? 8'd1 : 8'd0)
                                - (src_rvalid ? 8'd1 : 8'd0);

            // ---- input FIFO occupancy ----
            i_cnt <= i_cnt + (src_rvalid ? 8'd1 : 8'd0)
                          - (dec_takes ? 8'd1 : 8'd0);

            // ---- decoder handshake ----
            if (dec_takes) begin
                i_rp     <= i_rp + 1'b1;
                ip_taken <= ip_taken + 32'd1;
            end

            // ---- output FIFO ----
            if (o_push && (o_cnt < OFIFO[7:0])) begin
                o_fifo[o_wp] <= out_data;
                o_wp         <= o_wp + 1'b1;
            end
            if (o_drain) o_rp <= o_rp + 1'b1;
            o_cnt <= o_cnt + (o_push ? 8'd1 : 8'd0) - (o_drain ? 8'd1 : 8'd0);
            if (o_drain) out_count <= out_count + 32'd1;

            // ---- completion ----
            if (dec_error) error_q <= 1'b1;
            if (dec_done && (o_cnt == 8'd0)) done_q <= 1'b1;

            if (start) begin
                started  <= 1'b1;
                done_q   <= 1'b0;
                error_q  <= 1'b0;
                i_wp <= '0; i_rp <= '0; i_cnt <= 8'd0;
                ip_issued <= 32'd0; ip_taken <= 32'd0; inflight <= 8'd0;
                o_wp <= '0; o_rp <= '0; o_cnt <= 8'd0;
                out_count <= 32'd0;
            end
        end
    end

    assign done  = done_q;
    assign error = error_q;

endmodule
