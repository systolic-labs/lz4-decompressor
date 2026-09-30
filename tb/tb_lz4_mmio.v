// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Systolic Labs
// ============================================================================
//  tb_lz4_mmio.v - AXI-level self-checking testbench for lz4_mmio
// ----------------------------------------------------------------------------
//  Drives the peripheral the way firmware will: start a block, wait until CTRL
//  says the decoder can accept a byte, write it, collect output, repeat.
//
//  The wait on bit 4 is the whole point.  The decoder does not consume input
//  while it is inside a match, and the kernel vector below contains a 98 576-byte
//  match; software that writes without waiting overwrites a waiting byte once per
//  byte of that match.  The first version had no such bit and produced a
//  corrupted 13 190 715-byte "kernel" with no error flagged.  A testbench that
//  ignores bit 4 would certify exactly that bug, so it is checked, not assumed.
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
    localparam [63:0] A_PRODUCED = 64'h0C, A_TAKEN = 64'h10, A_FLAGS = 64'h14;
    // 8 staged bytes, read idempotently; the valid count; and consumption is an
    // explicit write to CTRL bit1 - never a side effect of the read.
    localparam [63:0] A_O64 = 64'h18, A_OSTAT = 64'h1C;

    reg [7:0] input_bytes  [0:`IN_LEN-1];
    reg [7:0] expect_bytes [0:`EXP_LEN-1];
    reg [7:0] got_bytes    [0:`EXP_LEN-1];

    integer i, nout, errs, first_bad, k, wdog, extra, j, nv, draining, ready;
    // Dedicated loop counters.  wdog is reset to 0 inside every axi_read, so a
    // wait loop that calls axi_read and tests wdog can never trip its own
    // bound: it would spin forever if the condition it waits on never comes.
    integer wcnt, dcnt;
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
        end
    endtask

    // Take one staged group: read the valid count, read the word, consume it with
    // an explicit write.  Factored out because it has to run *inside* the
    // readiness wait too: once 8 bytes are staged the decoder's out_ready drops,
    // so a wait that does not drain leaves the decoder stalled and the bench
    // spins its whole bound on every single input byte.
    task drain_once;
        begin
            axi_read(A_OSTAT);
            if (rd_value[3:0] != 0) begin
                nv = rd_value[3:0];
                axi_read(A_O64);
                for (j = 0; j < 8; j = j + 1)
                    if (j < nv) begin
                        if (nout < `EXP_LEN) got_bytes[nout] = rd_value[8*j +: 8];
                        else extra = extra + 1;
                        nout = nout + 1;
                    end
                axi_write(A_CTRL, 64'h2);
            end
        end
    endtask

    initial begin
        $readmemh("in.hex", input_bytes);
        $readmemh("exp.hex", expect_bytes);
        nout = 0; errs = 0; first_bad = -1; extra = 0;
        repeat (6) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        axi_write(A_CTRL, 64'h1);                    // start a block

        for (i = 0; i < `IN_LEN; i = i + 1) begin
            // Wait until the decoder can take a byte.  This is the fix: without
            // it, every write inside a long match destroys a waiting byte.
            // The bound has to cover the decoder's longest match: it takes no
            // input while copying, and giving up means writing anyway, which
            // overwrites the pending byte and loses the following sequence.  A
            // 40 000-byte match is ~80 000 cycles, about 3 000 iterations here.
            // Generous is safe now that wcnt is a dedicated counter - the loop
            // really does expire, and it drains while it waits.
            wcnt = 0; ready = 0;
            while (!ready && wcnt < 400000) begin
                drain_once;                 // keep the staging moving while we wait
                axi_read(A_CTRL);
                if (rd_value[4]) ready = 1;
                wcnt = wcnt + 1;
            end

            axi_write(A_IN, {55'd0, (i == `IN_LEN - 1), input_bytes[i]});

            // Drain whatever is ready before moving on.  The delay before each
            // output read is the whole point: on silicon a register access costs
            // 50-90 cycles while this bench used a handful, so the decoder was
            // never actually made to wait on a full output holding register.  That
            // stall path is where silicon emits every byte twice, and a bench that
            // drains at full speed cannot see it.
            dcnt = 0; draining = 1;
            while (draining && dcnt < 64) begin
                axi_read(A_OSTAT);
                if (rd_value[3:0] == 0) draining = 0;
                else begin drain_once; dcnt = dcnt + 1; end
            end
        end

        // Drain the remainder and wait for done.
        wdog = 0;
        // Same reasoning: each pass costs several AXI accesses, and the tail
        // after the last input byte is at most one staged group.
        dcnt = 0;
        while (!rd_value[2] && dcnt < 20000) begin
            axi_read(A_CTRL);
            axi_read(A_OSTAT);
            if (rd_value[3:0] != 0) begin
                nv = rd_value[3:0];
                axi_read(A_O64);
                for (j = 0; j < 8; j = j + 1)
                    if (j < nv) begin
                        if (nout < `EXP_LEN) got_bytes[nout] = rd_value[8*j +: 8];
                        else extra = extra + 1;
                        nout = nout + 1;
                    end
                axi_write(A_CTRL, 64'h2);
            end
            dcnt = dcnt + 1;
        end

        // `done` may assert with a final group still staged, so take it
        // unconditionally: the last bytes of the stream are exactly that group.
        draining = 1;
        while (draining) begin
            axi_read(A_OSTAT);
            if (rd_value[3:0] == 0) draining = 0;
            else drain_once;
        end

        axi_read(A_CTRL);
        if (rd_value[1]) errs = errs + 1;            // bit1 = error

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
        $display("  extra past end : %0d   (silicon produced 2 093 115)", extra);
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
