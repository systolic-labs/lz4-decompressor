// ============================================================================
//  tb_lz4_engine.v - register-level bench for the SoC LZ4 engine
// ----------------------------------------------------------------------------
//  Drives the AXI4-Lite register file exactly as the loader does (SRC/DST/LEN,
//  then CTRL.start, then poll CTRL.done) and checks the decompressed region in a
//  modelled AXI4 slave memory.  This is the gate before the SoC build: the only
//  new logic versus tb_lz4_axi.v is the register file.
//
//  iverilog -g2012 -o tb_eng2 tb_lz4_engine.v lz4_engine.v lz4_axi.v lz4_decoder.v && vvp tb_eng2
// ============================================================================
`timescale 1ns/10ps
`include "lens.vh"

module tb_lz4_engine_reg;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    // ---- AXI4-Lite slave (registers) ----
    reg  [3:0]  s_awid = 0;  reg [63:0] s_awaddr = 0; reg [7:0] s_awlen = 0;
    reg  [2:0]  s_awsize = 3'd3; reg [1:0] s_awburst = 2'd1; reg s_awvalid = 0;
    wire        s_awready;
    reg  [63:0] s_wdata = 0; reg [7:0] s_wstrb = 8'hFF; reg s_wlast = 1; reg s_wvalid = 0;
    wire        s_wready;
    wire [3:0]  s_bid; wire [1:0] s_bresp; wire s_bvalid; reg s_bready = 1;
    reg  [3:0]  s_arid = 0;  reg [63:0] s_araddr = 0; reg [7:0] s_arlen = 0;
    reg  [2:0]  s_arsize = 3'd3; reg [1:0] s_arburst = 2'd1; reg s_arvalid = 0;
    wire        s_arready;
    wire [3:0]  s_rid; wire [63:0] s_rdata; wire [1:0] s_rresp; wire s_rlast; wire s_rvalid;
    reg         s_rready = 1;

    // ---- AXI4 master (data) ----
    wire [3:0]  m_arid, m_rid, m_awid, m_bid;
    wire [63:0] m_araddr, m_awaddr, m_rdata, m_wdata;
    wire [7:0]  m_arlen, m_awlen, m_wstrb;
    wire [2:0]  m_arsize, m_awsize;
    wire [1:0]  m_arburst, m_awburst, m_rresp, m_bresp;
    wire        m_arvalid, m_arready, m_rlast, m_rvalid, m_rready;
    wire        m_awvalid, m_awready, m_wlast, m_wvalid, m_wready, m_bvalid, m_bready;

    lz4_engine dut (
        .clk(clk), .rst_n(rst_n),
        .s_awid(s_awid), .s_awaddr(s_awaddr), .s_awlen(s_awlen), .s_awsize(s_awsize),
        .s_awburst(s_awburst), .s_awvalid(s_awvalid), .s_awready(s_awready),
        .s_wdata(s_wdata), .s_wstrb(s_wstrb), .s_wlast(s_wlast), .s_wvalid(s_wvalid),
        .s_wready(s_wready),
        .s_bid(s_bid), .s_bresp(s_bresp), .s_bvalid(s_bvalid), .s_bready(s_bready),
        .s_arid(s_arid), .s_araddr(s_araddr), .s_arlen(s_arlen), .s_arsize(s_arsize),
        .s_arburst(s_arburst), .s_arvalid(s_arvalid), .s_arready(s_arready),
        .s_rid(s_rid), .s_rdata(s_rdata), .s_rresp(s_rresp), .s_rlast(s_rlast),
        .s_rvalid(s_rvalid), .s_rready(s_rready),
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
    localparam integer MW = 19;
    reg [63:0] mem [0:(1<<MW)-1];
    reg rd_busy, aw_seen, w_seen, bvalid;
    reg [63:0] rd_addr_q, wr_addr_q;
    reg [7:0]  rd_cnt;
    integer b;

    assign m_arready = ~rd_busy;
    assign m_rvalid  = rd_busy;
    assign m_rdata   = mem[rd_addr_q[MW+2:3]];
    assign m_rlast   = (rd_cnt == 8'd1);
    assign m_rresp   = 2'b00;
    assign m_rid     = 4'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin rd_busy <= 1'b0; rd_addr_q <= 64'd0; rd_cnt <= 8'd0; end
        else if (!rd_busy && m_arvalid && m_arready) begin
            rd_addr_q <= m_araddr; rd_cnt <= m_arlen + 8'd1; rd_busy <= 1'b1;
        end else if (rd_busy && m_rready) begin
            if (rd_cnt == 8'd1) rd_busy <= 1'b0;
            else begin rd_cnt <= rd_cnt - 8'd1; rd_addr_q <= rd_addr_q + 64'd8; end
        end
    end

    assign m_awready = ~aw_seen;
    assign m_wready  = ~w_seen;
    assign m_bvalid  = bvalid;
    assign m_bresp   = 2'b00;
    assign m_bid     = 4'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            aw_seen <= 0; w_seen <= 0; bvalid <= 0; wr_addr_q <= 64'd0;
        end else begin
            if (m_awvalid && m_awready) begin wr_addr_q <= m_awaddr; aw_seen <= 1'b1; end
            if (m_wvalid && m_wready) begin
                for (b = 0; b < 8; b = b + 1)
                    if (m_wstrb[b]) mem[wr_addr_q[MW+2:3]][8*b +: 8] <= m_wdata[8*b +: 8];
                w_seen <= 1'b1;
            end
            if (!bvalid && aw_seen && w_seen) begin aw_seen <= 0; w_seen <= 0; bvalid <= 1'b1; end
            else if (bvalid && m_bready) bvalid <= 1'b0;
        end
    end

    // ---- AXI4-Lite register access tasks ----
    integer wdog;
    reg [63:0] rdv;

    task reg_write(input [63:0] addr, input [63:0] data);
        begin
            s_awid = 4'd0; s_awaddr = addr; s_awvalid = 1;
            s_wdata = data; s_wstrb = 8'hFF; s_wlast = 1; s_wvalid = 1;
            @(negedge clk); wdog = 0;
            while (s_awready !== 1'b1 && wdog < 64) begin @(negedge clk); wdog = wdog + 1; end
            @(posedge clk); s_awvalid = 0;
            @(negedge clk); wdog = 0;
            while (s_wready !== 1'b1 && wdog < 64) begin @(negedge clk); wdog = wdog + 1; end
            @(posedge clk); s_wvalid = 0;
            @(negedge clk); wdog = 0;
            while (s_bvalid !== 1'b1 && wdog < 64) begin @(negedge clk); wdog = wdog + 1; end
            @(posedge clk);
        end
    endtask

    task reg_read(input [63:0] addr);
        begin
            s_arid = 4'd0; s_araddr = addr; s_arvalid = 1;
            @(negedge clk); wdog = 0;
            while (s_arready !== 1'b1 && wdog < 64) begin @(negedge clk); wdog = wdog + 1; end
            @(posedge clk); s_arvalid = 0;
            @(negedge clk); wdog = 0;
            while (s_rvalid !== 1'b1 && wdog < 64) begin @(negedge clk); wdog = wdog + 1; end
            rdv = s_rdata;
            @(posedge clk);
        end
    endtask

    reg [7:0] input_bytes  [0:`IN_LEN-1];
    reg [7:0] expect_bytes [0:`EXP_LEN-1];
    integer k, errs, first_bad, cyc;
    reg [31:0] out_bytes;
    reg [63:0] ctrl_now;
    reg [1023:0] verdict;

    localparam [63:0] R_CTRL = 64'h00, R_SRCL = 64'h04, R_SRCH = 64'h08;
    localparam [63:0] R_DSTL = 64'h0C, R_DSTH = 64'h10, R_LEN = 64'h14, R_OUT = 64'h18;
    localparam [63:0] R_DBG  = 64'h1C;

    initial begin
        $readmemh("in.hex", input_bytes);
        $readmemh("exp.hex", expect_bytes);
        errs = 0; first_bad = -1; cyc = 0;

        for (k = 0; k < (1<<MW); k = k + 1) mem[k] = 64'd0;
        for (k = 0; k < `IN_LEN; k = k + 1) mem[k/8][8*(k%8) +: 8] = input_bytes[k];

        repeat (4) @(posedge clk);
        rst_n <= 1'b1;
        @(posedge clk);

        reg_write(R_SRCL, 64'h0);
        reg_write(R_SRCH, 64'h0);
        reg_write(R_DSTL, 64'h10_0000);
        reg_write(R_DSTH, 64'h0);
        reg_write(R_LEN,  `IN_LEN);
        reg_write(R_CTRL, 64'h1);              // start

        begin : wait_done
            forever begin
                @(posedge clk); cyc = cyc + 1;
                reg_read(R_CTRL);
                if (rdv[1] || rdv[2]) disable wait_done;
            end
        end
        repeat (6) @(posedge clk);

        for (k = 0; k < `EXP_LEN; k = k + 1)
            if (mem[64'h10_0000/8 + k/8][8*(k%8) +: 8] !== expect_bytes[k]) begin
                if (first_bad < 0) first_bad = k;
                errs = errs + 1;
            end
        reg_read(R_OUT);
        out_bytes = rdv[31:0];
        reg_read(R_CTRL);
        ctrl_now = rdv;
        if (out_bytes != `EXP_LEN) errs = errs + 1;

        // Debug path: source XOR as the CPU sees it vs the engine's dbg_sum,
        // and check the 0x1C selector returns the two different halves.
        begin
            reg [63:0] csum; reg [31:0] lo, hi; integer wi;
            csum = 64'd0;
            for (wi = 0; wi < ((`IN_LEN + 7) / 8); wi = wi + 1)
                csum = csum ^ mem[wi];
            reg_write(R_DBG, 64'd4); reg_read(R_DBG); lo = rdv[31:0];
            reg_write(R_DBG, 64'd5); reg_read(R_DBG); hi = rdv[31:0];
            $display("  csum cpu=%h eng=%h", csum, {hi, lo});
            if ({hi, lo} !== csum) begin errs = errs + 1; $display("  CSUM MISMATCH"); end
        end

        verdict = "PASS";
        if (ctrl_now[2])             verdict = "FAIL decoder-error";
        if (out_bytes != `EXP_LEN)   verdict = "FAIL wrong-length";
        if (errs != 0)               verdict = "FAIL wrong-bytes";

        $display("--------------------------------------------------");
        $display("  input bytes    : %0d", `IN_LEN);
        $display("  output bytes   : %0d of %0d expected", out_bytes, `EXP_LEN);
        $display("  cycles (incl regs): %0d", cyc);
        if (first_bad >= 0)
            $display("  first mismatch : byte %0d  got %02x  expected %02x",
                     first_bad, mem[64'h10_0000/8 + first_bad/8][8*(first_bad%8) +: 8],
                     expect_bytes[first_bad]);
        $display("  RESULT: %0s", verdict);
        $display("--------------------------------------------------");
        $finish;
    end

    initial begin
        #4_000_000_000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
