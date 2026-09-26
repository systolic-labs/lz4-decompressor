// ============================================================================
//  tb_lz4_decoder.v - self-checking testbench for lz4_decoder
// ----------------------------------------------------------------------------
//  Feeds a real LZ4 block (produced by the BSD-2-Clause reference compressor)
//  and compares every decompressed byte against the original plaintext.  The
//  window is modelled exactly as the decoder's contract states: a write on the
//  rising edge, and one cycle of read latency.
//
//  iverilog -g2005 -o tb_lz4 tb_lz4_decoder.v lz4_decoder.v && vvp tb_lz4
// ============================================================================
`timescale 1ns/10ps
`include "lens.vh"

module tb_lz4_decoder;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  [7:0] in_data  = 8'd0;
    reg        in_valid = 1'b0;
    reg        in_last  = 1'b0;
    wire       in_ready;
    wire [7:0] out_data;
    wire       out_valid;
    reg        out_ready = 1'b0;
    wire [15:0] win_waddr;
    wire [7:0]  win_wdata;
    wire        win_we;
    wire [15:0] win_raddr;
    wire [7:0]  win_rdata;
    wire        done, error;

    lz4_decoder dut (
        .clk(clk), .rst_n(rst_n),
        .in_data(in_data), .in_valid(in_valid), .in_last(in_last), .in_ready(in_ready),
        .out_data(out_data), .out_valid(out_valid), .out_ready(out_ready),
        .win_waddr(win_waddr), .win_wdata(win_wdata), .win_we(win_we),
        .win_raddr(win_raddr), .win_rdata(win_rdata),
        .done(done), .error(error)
    );

    // 64 KiB history window: write on the edge, one cycle of read latency.
    reg [7:0] mem [0:65535];
    reg [7:0] rd;
    always @(posedge clk) begin
        if (win_we) mem[win_waddr] <= win_wdata;
        rd <= mem[win_raddr];
    end
    assign win_rdata = rd;

    reg [7:0] input_bytes  [0:`IN_LEN-1];
    reg [7:0] expect_bytes [0:`EXP_LEN-1];
    reg [7:0] got_bytes    [0:`EXP_LEN-1];

    integer i, nout, errs, first_bad, k;
    reg [1023:0] verdict;

    initial begin
        $readmemh("in.hex", input_bytes);
        $readmemh("exp.hex", expect_bytes);
        nout = 0; errs = 0; first_bad = -1;

        repeat (4) @(posedge clk);
        rst_n <= 1'b1;
        @(posedge clk);

        out_ready <= 1'b1;

        // ---- stimulus: present a byte, wait for ready, then advance ----
        fork
            begin
                for (i = 0; i < `IN_LEN; i = i + 1) begin
                    in_data  <= input_bytes[i];
                    in_valid <= 1'b1;
                    in_last  <= (i == `IN_LEN - 1);
                    @(posedge clk);
                    while (in_ready !== 1'b1) @(posedge clk);
                    in_valid <= 1'b0;
                    in_last  <= 1'b0;
                    @(posedge clk);
                end
            end
        join_none

        // ---- collector ----
        fork
            begin
                while (nout < `EXP_LEN + 16) begin
                    @(posedge clk);
                    if (out_valid) begin
                        if (nout < `EXP_LEN) got_bytes[nout] = out_data;
                        nout = nout + 1;
                    end
                end
            end
        join_none

        // ---- wait for the block to finish ----
        begin : wait_done
            forever begin
                @(posedge clk);
                if (done === 1'b1 || error === 1'b1) disable wait_done;
            end
        end
        repeat (20) @(posedge clk);

        for (k = 0; k < `EXP_LEN; k = k + 1) begin
            if (got_bytes[k] !== expect_bytes[k]) begin
                if (first_bad < 0) first_bad = k;
                errs = errs + 1;
            end
        end
        if (nout != `EXP_LEN) errs = errs + 1;

        verdict = "PASS";
        if (error)            verdict = "FAIL decoder-error";
        if (nout != `EXP_LEN) verdict = "FAIL wrong-length";
        if (errs != 0)        verdict = "FAIL wrong-bytes";

        $display("--------------------------------------------------");
        $display("  input bytes    : %0d", `IN_LEN);
        $display("  output bytes   : %0d of %0d expected", nout, `EXP_LEN);
        $display("  decoder error  : %b", error);
        if (first_bad >= 0) begin
            $display("  first mismatch : byte %0d  got %02x  expected %02x",
                     first_bad, got_bytes[first_bad], expect_bytes[first_bad]);
            $display("  --- first 40 bytes: got / expected ---");
            for (k = 0; k < 40; k = k + 1)
                $display("    %0d: %02x %02x", k, got_bytes[k], expect_bytes[k]);
        end
        $display("  RESULT: %0s", verdict);
        $display("--------------------------------------------------");
        $finish;
    end

endmodule
