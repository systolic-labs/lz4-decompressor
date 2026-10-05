// ============================================================================
//  tb_lz4_engine.v - LZ4 decoder throughput ceiling with an ideal feeder
// ----------------------------------------------------------------------------
//  Spike for the DMA / autonomous-engine path.  The MMIO wrapper feeds four
//  compressed bytes per ~27-30-cycle access, which is what caps the stage at
//  ~25 cycles per output byte.  The question this bench answers is the other
//  end: if a byte is presented EVERY cycle the decoder will take one, and the
//  output is sunk with no back-pressure, how many cycles per output byte does
//  the decoder itself cost?  That is the floor an engine path could reach.
//
//  It is the block-level bench (tb_lz4_decoder.v) with two changes: the feeder
//  never leaves a gap (in_valid stays high, the byte advances on the handshake),
//  and a free-running cycle counter brackets the block.  Self-checking, same
//  in.hex / exp.hex vectors.
//
//  iverilog -g2012 -o tb_eng tb_lz4_engine.v lz4_decoder.v && vvp tb_eng
// ============================================================================
`timescale 1ns/10ps
`include "lens.vh"

module tb_lz4_engine;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;                 // 100 MHz

    reg  [7:0] in_data  = 8'd0;
    reg        in_valid = 1'b0;
    reg        in_last  = 1'b0;
    wire       in_ready;
    wire [7:0] out_data;
    wire       out_valid;
    wire       out_ready = 1'b1;          // ideal sink: never back-pressures
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
    integer cyc;
    reg [1023:0] verdict;

    // ---- ideal feeder: hold in_valid high, advance the byte on the handshake.
    // No gap between bytes - this is what an autonomous engine would do with a
    // prefetch buffer ahead of the decoder.
    always @(posedge clk) begin
        if (!rst_n) begin
            i <= 0;
            in_data  <= input_bytes[0];
            in_valid <= 1'b1;
            in_last  <= (`IN_LEN == 1);
        end else if (in_valid && in_ready) begin
            if (i >= `IN_LEN - 1) begin
                in_valid <= 1'b0;               // last byte accepted
                in_last  <= 1'b0;
            end else begin
                i        <= i + 1;
                in_data  <= input_bytes[i + 1];
                in_last  <= (i + 1 == `IN_LEN - 1);
            end
        end
    end

    // ---- ideal sink: take every output byte the cycle it is offered.
    always @(posedge clk) begin
        if (rst_n && out_valid) begin
            if (nout < `EXP_LEN) got_bytes[nout] = out_data;
            nout = nout + 1;
        end
    end

    always @(posedge clk) if (rst_n && !done && !error) cyc <= cyc + 1;

    initial begin
        $readmemh("in.hex", input_bytes);
        $readmemh("exp.hex", expect_bytes);
        nout = 0; errs = 0; first_bad = -1; cyc = 0;

        repeat (4) @(posedge clk);
        rst_n <= 1'b1;
        @(posedge clk);

        begin : wait_done
            forever begin
                @(posedge clk);
                if (done === 1'b1 || error === 1'b1) disable wait_done;
            end
        end
        repeat (4) @(posedge clk);

        for (k = 0; k < `EXP_LEN; k = k + 1)
            if (got_bytes[k] !== expect_bytes[k]) begin
                if (first_bad < 0) first_bad = k;
                errs = errs + 1;
            end
        if (nout != `EXP_LEN) errs = errs + 1;

        verdict = "PASS";
        if (error)            verdict = "FAIL decoder-error";
        if (nout != `EXP_LEN) verdict = "FAIL wrong-length";
        if (errs != 0)        verdict = "FAIL wrong-bytes";

        $display("--------------------------------------------------");
        $display("  input bytes    : %0d", `IN_LEN);
        $display("  output bytes   : %0d of %0d expected", nout, `EXP_LEN);
        $display("  decoder cycles : %0d", cyc);
        $display("  cycles/out-byte: %0d.%02d", cyc / `EXP_LEN,
                 ((cyc * 100) / `EXP_LEN) % 100);
        $display("  ms @100MHz     : %0d.%02d", cyc / 100000,
                 ((cyc * 1000) / 100000) % 1000);
        $display("  ms @ 50MHz     : %0d.%02d", cyc / 50000,
                 ((cyc * 1000) / 50000) % 1000);
        if (first_bad >= 0)
            $display("  first mismatch : byte %0d  got %02x  expected %02x",
                     first_bad, got_bytes[first_bad], expect_bytes[first_bad]);
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
