// ============================================================================
//  tb_lz4_engine_rst.v - AXI4 master rules and per-block restart, engine level
// ----------------------------------------------------------------------------
//  Two things the vector benches cannot see, both of them integration hazards a
//  reviewer asks about:
//
//    A. AXI4 burst rules.  An INCR burst must never cross a 4 KiB aligned
//       boundary.  This bench monitors every AR and AW the engine issues and
//       checks two things: the length field is 0 (the engine is single-beat) and
//       an 8-byte beat starting at that address does not cross 4 KiB.  It runs
//       the transfer stream across a boundary on purpose, with src and dst placed
//       so the *stream* spans 0x1000 and 0x2000.
//
//       A single 8-byte beat only crosses 4 KiB if its address is not 8-byte
//       aligned.  So the rule holds for free on aligned addresses and the real
//       requirement is alignment - which nothing checked.  Case A2 programmes a
//       misaligned source and asserts the engine refuses the block instead of
//       issuing a protocol-violating transfer.
//
//    B. Per-block restart.  A truncated block leaves the decoder mid-sequence.
//       Programming a fresh block must return the whole engine to a clean state,
//       without an FPGA-wide reset.  The bench decodes a deliberately truncated
//       block, then a good one, and compares the second output byte for byte.
//
//  iverilog -g2012 -I<vector dir> -o tb_rst tb_lz4_engine_rst.v lz4_axi.v lz4_decoder.v
// ============================================================================
`timescale 1ns/10ps
`include "lens.vh"

module tb_lz4_engine_rst;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    // src and dst chosen so the transfer streams straddle 4 KiB boundaries:
    // src runs 0x0FF0 upward, dst starts 8 bytes below 0x100000.  They must not
    // overlap - the output of the largest vector is 96 KiB and would otherwise
    // overwrite the compressed source while the engine is still reading it.
    localparam [63:0] SRC = 64'h0000_0FF0;
    localparam [63:0] DST = 64'h000F_FFF8;

    reg         start = 0;
    reg  [63:0] src_addr = 0;
    reg  [63:0] dst_addr = 0;
    reg  [31:0] ilen = 0;
    wire        busy, done, error;
    wire [31:0] out_count;
    wire [31:0] dbg_rd, dbg_wr, dbg_feed, dbg_err;
    wire [63:0] dbg_sum;
    wire [31:0] dbg_hash;

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
        .dbg_rd(dbg_rd), .dbg_wr(dbg_wr), .dbg_feed(dbg_feed), .dbg_err(dbg_err),
        .dbg_sum(dbg_sum), .dbg_hash(dbg_hash),
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

    // ---- single-beat 64-bit AXI4 slave memory (same model as tb_lz4_axi) ----
    localparam integer MW = 19;                       // 512 K words = 4 MiB
    reg [63:0] mem [0:(1<<MW)-1];

    reg         rd_busy;
    reg [63:0]  rd_addr_q, wr_addr_q;
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

    reg aw_seen, w_seen, bvalid;
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
            if (!bvalid && aw_seen && w_seen) begin
                aw_seen <= 1'b0; w_seen <= 1'b0; bvalid <= 1'b1;
            end else if (bvalid && m_bready) begin
                bvalid <= 1'b0;
            end
        end
    end

    // ---- AXI4 rule monitor -------------------------------------------------
    // 4096 is a multiple of 8, so an 8-byte aligned beat can never straddle a
    // 4 KiB boundary; only a misaligned address can.  Offset 4088 is the last
    // one whose 8 bytes stay below the line.
    integer n_ar, n_aw, n_len, n_cross;
    always @(posedge clk) begin
        if (m_arvalid && m_arready) begin
            n_ar = n_ar + 1;
            if (m_arlen != 8'd0)      n_len   = n_len + 1;
            if (m_araddr[11:0] > 12'd4088) n_cross = n_cross + 1;
        end
        if (m_awvalid && m_awready) begin
            n_aw = n_aw + 1;
            if (m_awlen != 8'd0)      n_len   = n_len + 1;
            if (m_awaddr[11:0] > 12'd4088) n_cross = n_cross + 1;
        end
    end

    reg [7:0] input_bytes  [0:`IN_LEN-1];
    reg [7:0] expect_bytes [0:`EXP_LEN-1];

    integer i, k, errs, fails, cases;
    reg [1023:0] verdict;
    reg          saw_error, saw_done;

    task poke(input [63:0] a, input [7:0] d);
        begin
            mem[a[MW+2:3]][8*a[2:0] +: 8] = d;
        end
    endtask

    function [7:0] peek(input [63:0] a);
        begin
            peek = mem[a[MW+2:3]][8*a[2:0] +: 8];
        end
    endfunction

    task clear_region(input [63:0] base, input integer n);
        integer j;
        begin
            for (j = 0; j < n; j = j + 1) poke(base + j, 8'h00);
        end
    endtask

    // Pulse start and wait for done or error, bounded by `watch`.
    task run_block(input [63:0] sa, input [63:0] da, input integer len, input integer watch);
        integer c;
        begin
            src_addr = sa; dst_addr = da; ilen = len;
            start = 1'b1; @(posedge clk); start = 1'b0;
            saw_done = 0; saw_error = 0;
            c = 0;
            while (!saw_done && !saw_error && c < watch) begin
                @(posedge clk);
                c = c + 1;
                if (done)  saw_done  = 1'b1;
                if (error) saw_error = 1'b1;
            end
            @(posedge clk);
        end
    endtask

    task report_case(input integer ok, input integer what, input integer a, input integer b2);
        begin
            cases = cases + 1;
            if (ok) $display("    [ok]");
            else begin
                fails = fails + 1;
                $display("    [BAD]  %0d (got %0d, expected %0d)", what, a, b2);
            end
        end
    endtask

    initial begin
        for (i = 0; i < (1<<MW); i = i + 1) mem[i] = 64'd0;
        $readmemh("in.hex",  input_bytes);
        $readmemh("exp.hex", expect_bytes);
        fails = 0; cases = 0;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);

        // ==================================================================
        $display("==================================================");
        $display(" A1. aligned transfer stream across 4 KiB boundaries");
        $display("==================================================");
        n_ar = 0; n_aw = 0; n_len = 0; n_cross = 0;
        clear_region(SRC, `IN_LEN);
        clear_region(DST, `EXP_LEN);
        for (k = 0; k < `IN_LEN; k = k + 1) poke(SRC + k, input_bytes[k]);
        $display("  case: %0d compressed bytes at 0x%h, output at 0x%h", `IN_LEN, SRC, DST);
        run_block(SRC, DST, `IN_LEN, 400000);
        errs = 0;
        for (k = 0; k < `EXP_LEN; k = k + 1)
            if (peek(DST + k) !== expect_bytes[k]) errs = errs + 1;
        $display("    AR beats %0d, AW beats %0d, length!=0 %0d, 4KiB crossings %0d",
                 n_ar, n_aw, n_len, n_cross);
        if (saw_error) begin
            $display("    [BAD]  decoder raised error");
            fails = fails + 1;
        end else if (!saw_done) begin
            $display("    [BAD]  never completed");
            fails = fails + 1;
        end else if (errs != 0) begin
            $display("    [BAD]  %0d wrong bytes in the output", errs);
            fails = fails + 1;
        end else if (n_cross != 0 || n_len != 0) begin
            $display("    [BAD]  AXI4 burst rule violated");
            fails = fails + 1;
        end else begin
            $display("    [ok]   output exact, no burst crosses a 4 KiB line");
        end
        cases = cases + 1;

        // ==================================================================
        $display("==================================================");
        $display(" A2. misaligned source address");
        $display("==================================================");
        n_ar = 0; n_aw = 0; n_len = 0; n_cross = 0;
        $display("  case: source at 0x0FFC (not 8-byte aligned)");
        run_block(64'h0000_0FFC, DST, 64, 20000);
        $display("    AR beats %0d, length!=0 %0d, 4KiB crossings %0d, error %b",
                 n_ar, n_len, n_cross, saw_error);
        if (n_cross == 0 && saw_error) begin
            $display("    [ok]   refused, no protocol-violating transfer issued");
        end else if (n_cross != 0) begin
            $display("    [BAD]  %0d transfers cross a 4 KiB line (AXI4 forbids this)", n_cross);
            fails = fails + 1;
        end else begin
            $display("    [BAD]  misaligned address accepted without error");
            fails = fails + 1;
        end
        cases = cases + 1;

        // ==================================================================
        $display("==================================================");
        $display(" B. restart after a truncated block");
        $display("==================================================");
        clear_region(DST, `EXP_LEN);
        for (k = 0; k < `IN_LEN; k = k + 1) poke(SRC + k, input_bytes[k]);
        $display("  step 1: truncated block, ilen = %0d of %0d", `IN_LEN / 2, `IN_LEN);
        // The watch must cover a *full* decode of half the block: the ratio of
        // this codec is never below 1, so half the input can still expand to far
        // more than half the output, and a short watch would report a completed
        // decode as a hang.
        run_block(SRC, DST, `IN_LEN / 2, 400000);
        $display("    done %b, error %b  (either is acceptable: the stream is short)",
                 saw_done, saw_error);

        $display("  step 2: reprogramme with the whole block, no reset in between");
        clear_region(DST, `EXP_LEN);
        run_block(SRC, DST, `IN_LEN, 400000);
        errs = 0;
        for (k = 0; k < `EXP_LEN; k = k + 1)
            if (peek(DST + k) !== expect_bytes[k]) errs = errs + 1;
        if (saw_error) begin
            $display("    [BAD]  second block raised error - engine not clean");
            fails = fails + 1;
        end else if (!saw_done) begin
            $display("    [BAD]  second block never completed");
            fails = fails + 1;
        end else if (errs != 0) begin
            $display("    [BAD]  %0d wrong bytes after restart, out_count %0d of %0d",
                     errs, out_count, `EXP_LEN);
            fails = fails + 1;
        end else begin
            $display("    [ok]   restart decoded %0d bytes exact", out_count);
        end
        cases = cases + 1;

        // ==================================================================
        $display("==================================================");
        $display(" C. output length not a whole number of 8-byte words");
        $display("==================================================");
        // Every shipped vector decompresses to a multiple of 8 bytes, so the
        // engine's flush of a partial accumulator - flush_req, with byte strobes
        // on the final write - is never reached by them.  Build a block that ends
        // mid-word: 12 literals, an offset-1 run of 64, then 6 literals = 82.
        begin : partial_final_word
            integer j, nl, blen, elen;
            reg [7:0] blk [0:127];
            reg [7:0] exp2 [0:255];
            nl = 0;
            blk[nl] = 8'hCF; nl = nl + 1;                 // 12 literals, match nibble 15
            for (j = 0; j < 12; j = j + 1) begin blk[nl] = 8'h41 + j[7:0]; nl = nl + 1; end
            blk[nl] = 8'h01; nl = nl + 1;                 // offset 1
            blk[nl] = 8'h00; nl = nl + 1;
            blk[nl] = 8'd45; nl = nl + 1;                 // match length 64 - 19
            blk[nl] = 8'h60; nl = nl + 1;                 // final sequence: 6 literals
            for (j = 0; j < 6; j = j + 1) begin blk[nl] = 8'hE0 + j[7:0]; nl = nl + 1; end
            blen = nl;
            elen = 0;
            for (j = 0; j < 12; j = j + 1) begin exp2[elen] = 8'h41 + j[7:0]; elen = elen + 1; end
            for (j = 0; j < 64; j = j + 1) begin exp2[elen] = exp2[elen - 1]; elen = elen + 1; end
            for (j = 0; j < 6; j = j + 1) begin exp2[elen] = 8'hE0 + j[7:0]; elen = elen + 1; end

            $display("  case: %0d compressed -> %0d bytes out (%0d bytes past the last word)",
                     blen, elen, 8 - (elen % 8));
            clear_region(SRC, blen);
            clear_region(DST, elen + 16);
            for (j = 0; j < blen; j = j + 1) poke(SRC + j, blk[j]);
            run_block(SRC, DST, blen, 200000);
            errs = 0;
            for (j = 0; j < elen; j = j + 1)
                if (peek(DST + j) !== exp2[j]) errs = errs + 1;
            // The strobes on the final write must cover only the live bytes.
            for (j = elen; j < elen + 8; j = j + 1)
                if (peek(DST + j) !== 8'h00) errs = errs + 100;
            if (!saw_done) begin
                $display("    [BAD]  never completed (error %b)", saw_error);
                fails = fails + 1;
            end else if (errs != 0) begin
                $display("    [BAD]  %0d wrong or clobbered bytes after the flush", errs);
                fails = fails + 1;
            end else begin
                $display("    [ok]   %0d bytes exact, and the bytes past them untouched", elen);
            end
            cases = cases + 1;
        end

        $display("==================================================");
        verdict = "PASS";
        if (fails != 0) verdict = "FAIL engine rules";
        $display("  %0d cases, %0d failed", cases, fails);
        $display("  RESULT: %0s", verdict);
        $display("==================================================");
        $finish;
    end

endmodule
