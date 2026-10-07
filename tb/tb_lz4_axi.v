// ============================================================================
//  tb_lz4_axi.v - self-checking bench for the AXI4 stream engine
// ----------------------------------------------------------------------------
//  Models a single-beat 64-bit AXI4 slave memory, programmes the engine with
//  src/dst/len, waits for done, and compares the decompressed region byte for
//  byte.  Measures cycles from start to done.
//
//  iverilog -g2012 -o tb_axi tb_lz4_axi.v lz4_axi.v lz4_decoder.v && vvp tb_axi
// ============================================================================
`timescale 1ns/10ps
`include "lens.vh"

module tb_lz4_axi;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg         start = 0;
    reg  [63:0] src_addr = 64'h0;
    reg  [63:0] dst_addr = 64'h10_0000;      // 1 MiB into the memory
    reg  [31:0] ilen = 0;
    wire        busy, done, error;
    wire [31:0] out_count;

    logic [3:0]  m_arid, m_rid, m_awid, m_bid;
    logic [63:0] m_araddr, m_awaddr, m_rdata, m_wdata;
    logic [7:0]  m_arlen, m_awlen;
    logic [2:0]  m_arsize, m_awsize;
    logic [1:0]  m_arburst, m_awburst, m_rresp, m_bresp;
    logic        m_arvalid, m_arready, m_rlast, m_rvalid, m_rready;
    logic        m_awvalid, m_awready, m_wlast, m_wvalid, m_wready, m_bvalid, m_bready;
    logic [7:0]  m_wstrb;

    lz4_axi dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .src_addr(src_addr), .dst_addr(dst_addr), .ilen(ilen),
        .busy(busy), .done(done), .error(error), .out_count(out_count),
        .m_arid(m_arid), .m_araddr(m_araddr), .m_arlen(m_arlen), .m_arsize(m_arsize),
        .m_arburst(m_arburst), .m_arvalid(m_arvalid), .m_arready(m_arready),
        .m_rid(m_rid), .m_rdata(m_rdata), .m_rresp(m_rresp), .m_rlast(m_rlast),
        .m_rvalid(m_rvalid), .m_rready(m_rready),
        .m_awid(m_awid), .m_awaddr(m_awaddr), .m_awlen(m_awlen), .m_awsize(m_awsize),
        .m_awburst(m_awburst), .m_awvalid(m_awvalid), .m_awready(m_awready),
        .m_wdata(m_wdata), .m_wstrb(m_wstrb), .m_wlast(m_wlast), .m_wvalid(m_wvalid),
        .m_wready(m_wready),
        .m_bid(m_bid), .m_bresp(m_bresp), .m_bvalid(m_bvalid), .m_bready(m_bready)
    );

    // ---- single-beat 64-bit AXI4 slave memory ----
    localparam integer MW = 19;                       // 512 K words = 4 MiB
    reg [63:0] mem [0:(1<<MW)-1];

    reg         rd_busy;
    reg [63:0]  rd_addr_q;
    reg [63:0]  wr_addr_q;
    reg [7:0]   rd_cnt;
    integer     b;

    assign m_arready = ~rd_busy;
    assign m_rvalid  = rd_busy;
    assign m_rdata   = mem[rd_addr_q[MW+2:3]];
    assign m_rlast   = (rd_cnt == 8'd1);
    assign m_rresp   = 2'b00;
    assign m_rid     = 4'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_busy <= 1'b0; rd_addr_q <= 64'd0; rd_cnt <= 8'd0;
        end else if (!rd_busy && m_arvalid && m_arready) begin
            rd_addr_q <= m_araddr;
            rd_cnt    <= m_arlen + 8'd1;
            rd_busy   <= 1'b1;
        end else if (rd_busy && m_rready) begin
            if (rd_cnt == 8'd1) rd_busy <= 1'b0;
            else begin rd_cnt <= rd_cnt - 8'd1; rd_addr_q <= rd_addr_q + 64'd8; end
        end
    end

    reg         aw_seen, w_seen, bvalid;
    assign m_awready = ~aw_seen;
    assign m_wready  = ~w_seen;
    assign m_bvalid  = bvalid;
    assign m_bresp   = 2'b00;
    assign m_bid     = 4'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            aw_seen <= 1'b0; w_seen <= 1'b0; bvalid <= 1'b0; wr_addr_q <= 64'd0;
        end else begin
            if (m_awvalid && m_awready) begin
                wr_addr_q <= m_awaddr;
                aw_seen   <= 1'b1;
            end
            if (m_wvalid && m_wready) begin
                for (b = 0; b < 8; b = b + 1)
                    if (m_wstrb[b]) mem[wr_addr_q[MW+2:3]][8*b +: 8] <= m_wdata[8*b +: 8];
                w_seen <= 1'b1;
            end
            // BVALID only once BOTH AW and W have been accepted - a slave that
            // answers right after AW makes the master wait in W_B for a response
            // that has already gone.
            if (!bvalid && aw_seen && w_seen) begin
                aw_seen <= 1'b0; w_seen <= 1'b0; bvalid <= 1'b1;
            end else if (bvalid && m_bready) begin
                bvalid <= 1'b0;
            end
        end
    end

    // ---- load the compressed block and the expectation ----
    reg [7:0] input_bytes  [0:`IN_LEN-1];
    reg [7:0] expect_bytes [0:`EXP_LEN-1];

    integer k, errs, first_bad, cyc;
    reg [1023:0] verdict;

    initial begin
        $readmemh("in.hex", input_bytes);
        $readmemh("exp.hex", expect_bytes);
        errs = 0; first_bad = -1; cyc = 0;

        for (k = 0; k < (1<<MW); k = k + 1) mem[k] = 64'd0;
        for (k = 0; k < `IN_LEN; k = k + 1)
            mem[k/8][8*(k%8) +: 8] = input_bytes[k];

        repeat (4) @(posedge clk);
        rst_n <= 1'b1;
        @(posedge clk);

        ilen  <= `IN_LEN;
        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;

        begin : wait_done
            forever begin
                @(posedge clk);
                cyc = cyc + 1;
                if (done === 1'b1 || error === 1'b1) disable wait_done;
            end
        end
        repeat (6) @(posedge clk);

        for (k = 0; k < `EXP_LEN; k = k + 1) begin
            if (mem[(dst_addr[MW+2:3]) + k/8][8*(k%8) +: 8] !== expect_bytes[k]) begin
                if (first_bad < 0) first_bad = k;
                errs = errs + 1;
            end
        end
        if (out_count != `EXP_LEN) errs = errs + 1;

        verdict = "PASS";
        if (error)                 verdict = "FAIL decoder-error";
        if (out_count != `EXP_LEN) verdict = "FAIL wrong-length";
        if (errs != 0)             verdict = "FAIL wrong-bytes";

        $display("--------------------------------------------------");
        $display("  input bytes    : %0d", `IN_LEN);
        $display("  output bytes   : %0d of %0d expected", out_count, `EXP_LEN);
        $display("  engine cycles  : %0d", cyc);
        $display("  cycles/out-byte: %0d.%02d", cyc / `EXP_LEN,
                 ((cyc * 100) / `EXP_LEN) % 100);
        $display("  ms @ 50MHz     : %0d.%02d", cyc / 50000,
                 ((cyc * 1000) / 50000) % 1000);
        if (first_bad >= 0)
            $display("  first mismatch : byte %0d  got %02x  expected %02x",
                     first_bad,
                     mem[(dst_addr[MW+2:3]) + first_bad/8][8*(first_bad%8) +: 8],
                     expect_bytes[first_bad]);
        $display("  RESULT: %0s", verdict);
        $display("--------------------------------------------------");
        $finish;
    end

    initial begin
        #2_000_000_000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
