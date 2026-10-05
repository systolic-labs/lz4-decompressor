// ============================================================================
//  tb_lz4_stream.v - self-checking bench for the buffered stream engine
// ----------------------------------------------------------------------------
//  Models the two memories the engine talks to (a registered-read source and a
//  write sink) and measures the block's cost in cycles: does the buffering
//  (finite FIFOs + 1-cycle memory latency) hold the decoder's ~2 cycles/output
//  byte, or does it starve/stall?
//
//  iverilog -g2012 -o tb_str tb_lz4_stream.v lz4_stream.v lz4_decoder.v && vvp tb_str
// ============================================================================
`timescale 1ns/10ps
`include "lens.vh"

module tb_lz4_stream;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;                 // 100 MHz

    reg         start = 0;
    reg  [31:0] ilen  = 0;
    wire        busy, done, error;
    wire [31:0] out_count;
    wire [31:0] src_raddr;
    wire        src_valid;
    reg         src_rvalid;
    reg  [7:0]  src_rdata;
    wire [31:0] dst_waddr;
    wire [7:0]  dst_wdata;
    wire        dst_we;

    lz4_stream dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .ilen(ilen), .busy(busy), .done(done), .error(error),
        .out_count(out_count),
        .src_raddr(src_raddr), .src_valid(src_valid),
        .src_rvalid(src_rvalid), .src_rdata(src_rdata),
        .dst_waddr(dst_waddr), .dst_wdata(dst_wdata), .dst_we(dst_we)
    );

    // ---- source memory: registered read, one cycle of latency ----
    reg [7:0] src_mem [0:`IN_LEN-1];
    always @(posedge clk) begin
        src_rvalid <= src_valid;
        if (src_valid) src_rdata <= src_mem[src_raddr];
    end

    // ---- sink memory ----
    reg [7:0] dst_mem [0:`EXP_LEN-1];
    always @(posedge clk) if (dst_we) dst_mem[dst_waddr] <= dst_wdata;

    reg [7:0] expect_bytes [0:`EXP_LEN-1];

    integer k, errs, first_bad;
    integer cyc;
    reg [1023:0] verdict;

    initial begin
        $readmemh("in.hex", src_mem);
        $readmemh("exp.hex", expect_bytes);
        errs = 0; first_bad = -1; cyc = 0;

        repeat (4) @(posedge clk);
        rst_n <= 1'b1;
        @(posedge clk);

        // ---- pulse start with the compressed length ----
        ilen  <= `IN_LEN;
        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;

        // ---- count cycles until done ----
        begin : wait_done
            forever begin
                @(posedge clk);
                cyc = cyc + 1;
                if (done === 1'b1 || error === 1'b1) disable wait_done;
            end
        end
        repeat (4) @(posedge clk);

        for (k = 0; k < `EXP_LEN; k = k + 1)
            if (dst_mem[k] !== expect_bytes[k]) begin
                if (first_bad < 0) first_bad = k;
                errs = errs + 1;
            end
        if (out_count != `EXP_LEN) errs = errs + 1;

        verdict = "PASS";
        if (error)                    verdict = "FAIL decoder-error";
        if (out_count != `EXP_LEN)    verdict = "FAIL wrong-length";
        if (errs != 0)                verdict = "FAIL wrong-bytes";

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
                     first_bad, dst_mem[first_bad], expect_bytes[first_bad]);
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
