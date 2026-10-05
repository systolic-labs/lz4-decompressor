// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Systolic Labs
// ============================================================================
//  tb_lz4_mmio.v - AXI-level self-checking testbench for lz4_mmio
// ----------------------------------------------------------------------------
//  Drives the peripheral the way firmware does: start a block, wait until CTRL
//  says the decoder can accept a byte, write it, collect output, repeat.
//
//  The wait on bit 4 is the whole point.  The decoder does not consume input
//  while it is inside a match, and the kernel vector below contains a 98 576-byte
//  match; software that writes without waiting overwrites a waiting byte once per
//  byte of that match.  The first version had no such bit and produced a
//  corrupted 13 190 715-byte "kernel" with no error flagged.  A testbench that
//  ignores bit 4 would certify exactly that bug, so it is checked, not assumed.
//
//  OUTPUT DRAIN is the auto-increment group FIFO: read 0x1C for the number of
//  whole groups ready (once per batch, amortised), then for each group read the
//  low half at 0x18 (idempotent) and the high half at 0x08 (which pops the group).
//  Two accesses per 8-byte group plus one count read per batch.
//
//  iverilog -g2012 -o tb_mmio tb_lz4_mmio.v lz4_mmio.v lz4_decoder.v && vvp tb_mmio
// ============================================================================
`timescale 1ns/10ps
`include "lens.vh"

module tb_lz4_mmio;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  [7:0]  s_awid = 0;   reg [63:0] s_awaddr = 0; reg [7:0] s_awlen = 0;
    reg  [2:0]  s_awsize = 3'd3; reg [1:0] s_awburst = 2'd1; reg s_awvalid = 0;
    wire        s_awready;
    reg  [63:0] s_wdata = 0;  reg [7:0] s_wstrb = 8'hFF; reg s_wlast = 1; reg s_wvalid = 0;
    wire        s_wready;
    wire [7:0]  s_bid; wire [1:0] s_bresp; wire s_bvalid; reg s_bready = 1;
    reg  [7:0]  s_arid = 0;   reg [63:0] s_araddr = 0; reg [7:0] s_arlen = 0;
    reg  [2:0]  s_arsize = 3'd3; reg [1:0] s_arburst = 2'd1; reg s_arvalid = 0;
    wire        s_arready;
    wire [7:0]  s_rid; wire [63:0] s_rdata; wire [1:0] s_rresp; wire s_rlast; wire s_rvalid;
    reg         s_rready = 1;

    lz4_mmio dut (
        .clk(clk), .rst_n(rst_n),
        .s_awid(s_awid), .s_awaddr(s_awaddr), .s_awlen(s_awlen), .s_awsize(s_awsize),
        .s_awburst(s_awburst), .s_awvalid(s_awvalid), .s_awready(s_awready),
        .s_wdata(s_wdata), .s_wstrb(s_wstrb), .s_wlast(s_wlast), .s_wvalid(s_wvalid),
        .s_wready(s_wready),
        .s_bid(s_bid), .s_bresp(s_bresp), .s_bvalid(s_bvalid), .s_bready(s_bready),
        .s_arid(s_arid), .s_araddr(s_araddr), .s_arlen(s_arlen), .s_arsize(s_arsize),
        .s_arburst(s_arburst), .s_arvalid(s_arvalid), .s_arready(s_arready),
        .s_rid(s_rid), .s_rdata(s_rdata), .s_rresp(s_rresp), .s_rlast(s_rlast),
        .s_rvalid(s_rvalid), .s_rready(s_rready)
    );

    localparam [63:0] A_CTRL = 64'h00, A_IN = 64'h04, A_OUT = 64'h08;
    localparam [63:0] A_PRODUCED = 64'h0C, A_TAKEN = 64'h10, A_GC = 64'h14;
    // The FIFO head group: low half at 0x18 (idempotent), high half at 0x08
    // (reading it pops the group); the whole-group count and the two input flags
    // are at 0x1C.  Consumption is the high-half read itself - there is no
    // explicit consume write any more.
    localparam [63:0] A_O64 = 64'h18, A_OSTAT = 64'h1C, A_O64HI = 64'h08;
    localparam [63:0] A_INLEN = 64'h14;

    reg [7:0] input_bytes  [0:`IN_LEN-1];
    reg [7:0] expect_bytes [0:`EXP_LEN-1];
    reg [7:0] got_bytes    [0:`EXP_LEN-1];

    integer i, nout, errs, first_bad, k, wdog, extra, j, nv, draining, ready;
    integer gi;
    integer wcnt, dcnt;
    integer nacq;               // all MMIO accesses (one per load / store)
    integer ndrain;             // accesses issued by the output drain alone
    integer ncall, ncall_ne, nsum, nmax;   // batch statistics
    integer lat;                // modelled CPU cost per MMIO access, in clocks
    reg [63:0] w64;
    reg [63:0] iw;
    reg [31:0] produced, gtaken;
    reg        done_seen;
    reg [1023:0] verdict;
    reg [63:0] rd_value;

    task axi_write(input [63:0] addr, input [63:0] data);
        begin
            // Hold every channel signal until the slave takes it, and sample the
            // registered readies on the FALLING edge: after a rising edge they
            // have already been lowered and the transaction is silently dropped.
            s_awid = 8'hA5; s_awaddr = addr; s_awvalid = 1'b1;
            s_wdata = data; s_wstrb = 8'hFF; s_wlast = 1'b1; s_wvalid = 1'b1;
            @(negedge clk);
            wdog = 0;
            while (s_awready !== 1'b1 && wdog < 256) begin @(negedge clk); wdog = wdog + 1; end
            @(posedge clk);
            s_awvalid = 1'b0;
            @(negedge clk);
            wdog = 0;
            while (s_wready !== 1'b1 && wdog < 256) begin @(negedge clk); wdog = wdog + 1; end
            @(posedge clk);
            s_wvalid = 1'b0;
            @(negedge clk);
            wdog = 0;
            while (s_bvalid !== 1'b1 && wdog < 256) begin @(negedge clk); wdog = wdog + 1; end
            @(posedge clk);
            nacq = nacq + 1;
            repeat (lat) @(posedge clk);
        end
    endtask

    task axi_read(input [63:0] addr);
        begin
            s_arid = 8'h5A; s_araddr = addr; s_arvalid = 1'b1;
            @(negedge clk);
            wdog = 0;
            while (s_arready !== 1'b1 && wdog < 256) begin @(negedge clk); wdog = wdog + 1; end
            @(posedge clk);
            s_arvalid = 1'b0;
            @(negedge clk);
            wdog = 0;
            while (s_rvalid !== 1'b1 && wdog < 256) begin @(negedge clk); wdog = wdog + 1; end
            rd_value = s_rdata;
            @(posedge clk);
            nacq = nacq + 1;
            repeat (lat) @(posedge clk);
        end
    endtask

    // Drain every whole group currently in the FIFO.  One status read gives the
    // count; then each group is two 32-bit halves - the low half idempotent, the
    // high half popping the group.  This is the 1 + 2G protocol the design note
    // describes, and it is the same shape firmware uses.
    task drain_ready;
        integer n, g, jj;
        begin
            axi_read(A_OSTAT); ndrain = ndrain + 1;
            n = rd_value[3:0];
            ncall = ncall + 1;
            if (n > 0) begin ncall_ne = ncall_ne + 1; nsum = nsum + n; end
            if (n > nmax) nmax = n;
            for (g = 0; g < n; g = g + 1) begin
                axi_read(A_O64);   ndrain = ndrain + 1;           // 0x18 low
                w64[31:0]  = rd_value[31:0];
                axi_read(A_O64HI); ndrain = ndrain + 1;           // 0x08 high, pops
                w64[63:32] = rd_value[31:0];
                for (jj = 0; jj < 8; jj = jj + 1) begin
                    if (nout < `EXP_LEN) got_bytes[nout] = w64[8*jj +: 8];
                    else extra = extra + 1;
                    nout = nout + 1;
                end
            end
        end
    endtask

    initial begin
        $readmemh("in.hex", input_bytes);
        $readmemh("exp.hex", expect_bytes);
        if (!$value$plusargs("lat=%d", lat)) lat = 0;
        nout = 0; errs = 0; first_bad = -1; extra = 0; nacq = 0;
        ndrain = 0; ncall = 0; ncall_ne = 0; nsum = 0; nmax = 0;
        repeat (6) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        axi_write(A_CTRL, 64'h1);                    // start a block

        // Input is staged four bytes per write.  The end of the block is carried
        // by the block length, written once via INLEN (0x14) after the start.
        axi_write(A_INLEN, `IN_LEN);
        for (i = 0; i < `IN_LEN; i = i + 4) begin
            // Wait until the input staging is empty (CTRL bit4).  While it is
            // NOT empty the decoder is back-pressured (its output buffer is
            // full), so that is the moment to drain - and only then.  Draining
            // on every write instead would keep the FIFO nearly empty and throw
            // away the run-ahead the FIFO exists for; the whole point of the
            // FIFO is that the decoder may sit 72 bytes ahead of the CPU.  A
            // bound covers the decoder's longest match.
            wcnt = 0; ready = 0;
            while (!ready && wcnt < 400000) begin
                axi_read(A_CTRL);
                if (rd_value[4]) ready = 1;          // staging empty: write now
                else if (rd_value[3]) drain_ready;   // blocked: bank a full batch
                wcnt = wcnt + 1;
            end

            // Assemble the group: byte gi in bits [8*gi +: 8].  Bytes past the
            // end of the stream are padding; they are never consumed because
            // INLEN tells the decoder which byte is the block's last.
            iw = 64'd0;
            for (gi = 0; gi < 4; gi = gi + 1)
                if (i + gi < `IN_LEN)
                    iw = iw | ({56'd0, input_bytes[i + gi]} << (8 * gi));
            axi_write(A_IN, iw);
        end

        // Drain the remainder and wait for done.
        done_seen = 1'b0; dcnt = 0;
        while (!done_seen && dcnt < 2000000) begin
            axi_read(A_CTRL);
            done_seen = rd_value[2];
            drain_ready;
            dcnt = dcnt + 1;
        end

        // `done` may assert with a final group still staged; take everything.
        draining = 1; dcnt = 0;
        while (draining && dcnt < 200000) begin
            axi_read(A_OSTAT);
            if (rd_value[3:0] == 0) draining = 0;
            else drain_ready;
            dcnt = dcnt + 1;
        end

        axi_read(A_CTRL);
        if (rd_value[1]) errs = errs + 1;            // bit1 = error
        axi_read(A_PRODUCED); produced = rd_value[31:0];
        axi_read(A_GC);       gtaken   = rd_value[31:0];

        for (k = 0; k < `EXP_LEN; k = k + 1)
            if (got_bytes[k] !== expect_bytes[k]) begin
                if (first_bad < 0) first_bad = k;
                errs = errs + 1;
            end
        if (nout != `EXP_LEN) errs = errs + 1;

        verdict = "PASS";
        if (nout != `EXP_LEN) verdict = "FAIL wrong-length";
        if (errs != 0)        verdict = "FAIL wrong-bytes";

        $display("--------------------------------------------------");
        $display("  compressed     : %0d", `IN_LEN);
        $display("  decompressed   : %0d of %0d expected", nout, `EXP_LEN);
        $display("  extra past end : %0d", extra);
        $display("  MMIO accesses  : %0d total, %0d in the output drain", nacq, ndrain);
        $display("  drain/group    : %0d.%02d   (%0d groups)",
                 ndrain / (`EXP_LEN / 8), ((ndrain * 100) / (`EXP_LEN / 8)) % 100,
                 `EXP_LEN / 8);
        $display("  total/group    : %0d.%02d",
                 nacq / (`EXP_LEN / 8), ((nacq * 100) / (`EXP_LEN / 8)) % 100);
        $display("  drain/byte     : %0d.%03d", ndrain / `EXP_LEN,
                 ((ndrain * 1000) / `EXP_LEN) % 1000);
        $display("  batch G        : mean %0d.%02d over %0d non-empty reads, max %0d",
                 (ncall_ne ? nsum / ncall_ne : 0),
                 (ncall_ne ? ((nsum * 100) / ncall_ne) % 100 : 0),
                 ncall_ne, nmax);
        $display("  access model   : %0d clocks per MMIO access", lat);
        $display("  produced=%0d  groups_taken=%0d  (x8 = %0d)",
                 produced, gtaken, gtaken * 8);
        if (first_bad >= 0)
            $display("  first mismatch : byte %0d  got %02x  expected %02x",
                     first_bad, got_bytes[first_bad], expect_bytes[first_bad]);
        $display("  RESULT: %0s", verdict);
        $display("--------------------------------------------------");
        $finish;
    end

    initial begin
        // 20 s of simulated time: the real kernel stream needs ~220 ms of AXI
        // traffic and the drain models the CPU's 60-cycle access cost, so the old
        // 400 ms watchdog killed the run at a few percent of the way through.
        #20_000_000_000;
        $display("TIMEOUT: no completion after 20 s of simulated time");
        $finish;
    end

endmodule
