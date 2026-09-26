// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Systolic Labs
// ============================================================================
//  tb_lz4_mmio.v - AXI-level self-checking testbench for lz4_mmio
// ----------------------------------------------------------------------------
//  Drives the peripheral the way firmware will: start a block, write compressed
//  bytes, poll for output, read it.  Every decompressed byte is compared against
//  the original, so this checks the integration contract - register map, handshake,
//  ID echo - and not only the decoder.
//
//  Channel handling, which is where this went wrong the first time: every signal
//  is held until the slave has taken it, and the slave's registered readies are
//  sampled on the FALLING edge.  Sampling after a rising edge reads a ready that
//  has already been lowered, the transaction is dropped, and the DUT looks dead.
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

    reg [7:0] input_bytes  [0:`IN_LEN-1];
    reg [7:0] expect_bytes [0:`EXP_LEN-1];
    reg [7:0] got_bytes    [0:`EXP_LEN-1];

    integer i, nout, errs, first_bad, k, wdog;
    reg     more, drain;
    reg [1023:0] verdict;
    reg [63:0] rd_value;

    // ---- write one register: hold both channels until each is taken --------
    task axi_write(input [63:0] addr, input [63:0] data);
        begin
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

    initial begin
        $readmemh("in.hex", input_bytes);
        $readmemh("exp.hex", expect_bytes);
        nout = 0; errs = 0; first_bad = -1;
        repeat (6) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        axi_write(A_CTRL, 64'h1);                 // start a block

        for (i = 0; i < `IN_LEN; i = i + 1) begin
            axi_write(A_IN, {55'd0, (i == `IN_LEN - 1), input_bytes[i]});

            // Drain what is ready before feeding more, exactly as firmware will.
            wdog = 0; drain = 1'b1;
            while (drain && wdog < 512) begin
                axi_read(A_CTRL);
                if (rd_value[3]) begin              // bit3 = output byte available
                    axi_read(A_OUT);
                    if (nout < `EXP_LEN) got_bytes[nout] = rd_value[7:0];
                    nout = nout + 1;
                end else begin
                    drain = 1'b0;
                end
                wdog = wdog + 1;
            end
        end

        // Drain the remainder and wait for done.
        wdog = 0; more = 1'b1;
        while (more && wdog < 400000) begin
            axi_read(A_CTRL);
            if (rd_value[2]) begin                  // bit2 = done
                more = 1'b0;
            end else if (rd_value[3]) begin
                axi_read(A_OUT);
                if (nout < `EXP_LEN) got_bytes[nout] = rd_value[7:0];
                nout = nout + 1;
            end
            wdog = wdog + 1;
        end

        axi_read(A_CTRL);
        if (rd_value[1]) errs = errs + 1;            // bit1 = error

        for (k = 0; k < `EXP_LEN; k = k + 1) begin
            if (got_bytes[k] !== expect_bytes[k]) begin
                if (first_bad < 0) first_bad = k;
                errs = errs + 1;
            end
        end
        if (nout != `EXP_LEN) errs = errs + 1;

        verdict = "PASS";
        if (nout != `EXP_LEN) verdict = "FAIL wrong-length";
        if (errs != 0)        verdict = "FAIL wrong-bytes";

        $display("--------------------------------------------------");
        $display("  compressed     : %0d", `IN_LEN);
        $display("  decompressed   : %0d of %0d expected", nout, `EXP_LEN);
        if (first_bad >= 0)
            $display("  first mismatch : byte %0d  got %02x  expected %02x",
                     first_bad, got_bytes[first_bad], expect_bytes[first_bad]);
        $display("  RESULT: %0s", verdict);
        $display("--------------------------------------------------");
        $finish;
    end

    initial begin
        // Generous: a real kernel vector is ~389 000 AXI transactions, each of
        // which costs ~20 clocks of handshaking, so 50 ms of simulated time is
        // not enough and would look like a hang rather than a slow test.
        #600_000_000;
        $display("TIMEOUT: no completion after 600 ms of simulated time");
        $finish;
    end

endmodule
