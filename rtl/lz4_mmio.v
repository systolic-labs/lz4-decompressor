// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Systolic Labs
// ============================================================================
//  lz4_mmio.v - LZ4 block decompressor as an AXI4-Lite peripheral
// ----------------------------------------------------------------------------
//  Deliberately the simplest interface that is *correct*.  One byte in, one byte
//  out, and - the part the first version lacked - a status bit saying whether the
//  decoder can accept a byte at all.
//
//  Why that bit matters, from the first silicon run: the decoder does not consume
//  input while it is inside a match, and the kernel's stream contains a 98 576-byte
//  match.  Software that wrote IN unconditionally overwrote a byte still waiting,
//  once per byte of that match, desynchronising the stream - the run produced
//  13 190 715 bytes instead of 11 097 600 with no error flagged, and the kernel
//  did not boot.  The byte-at-a-time testbench missed it because its vector had
//  only short matches, so the decoder was almost always ready.  `CTRL` bit 4 now
//  exposes readiness and firmware waits on it: the constraint lives where software
//  can respect it.
//
//  Wider access and background DMA were both tried for speed and neither belongs
//  in this file: a 64-bit FIFO wrapper did not converge, and the measured MMIO
//  cost (50-90 cycles per access, against 5 for a 64-bit DDR4 access) means a
//  byte-per-register interface cannot be fast whatever its width.  Speed needs the
//  decoder fed by an autonomous engine instead of by the CPU - a separate piece of
//  work.  This file is the *functional* path, and it is one boot away from proving
//  the accelerator in silicon.
//
//  Register map (word index = addr[5:2]):
//    0x00  CTRL   W: bit0 = start a fresh block
//                 R: bit0 busy, bit1 error, bit2 done,
//                    bit3 = output byte available, bit4 = input accepted
//    0x04  IN     W: [7:0] compressed byte, [8] = last byte of the block
//    0x08  OUT    R: [7:0] decompressed byte; reading releases the decoder
//    0x0C  PROGRESS R: [31:16] bytes in, [15:0] bytes out
//
//  Window: 64 KiB of block RAM inside the peripheral.  The read is REGISTERED -
//  that is the decoder's stated contract and it is also what lets Vivado infer
//  block RAM: a combinational read costs 13 926 LUTs as distributed RAM against
//  277.
//
//  The read response must echo s_arid into s_rid and the write response must echo
//  s_awid into s_bid: the SoC routes through axi_riscv_atomics, which matches
//  responses to requests by ID, and hardcoding either leaves the CPU waiting
//  forever - on silicon, "banner then silence".
// ============================================================================
module lz4_mmio #(
    parameter integer AXI_AW = 64,
    parameter integer AXI_DW = 64,
    parameter integer AXI_IDW = 8
) (
    input  logic                  clk,
    input  logic                  rst_n,

    input  logic [AXI_IDW-1:0]    s_awid,
    input  logic [AXI_AW-1:0]     s_awaddr,
    input  logic [7:0]            s_awlen,
    input  logic [2:0]            s_awsize,
    input  logic [1:0]            s_awburst,
    input  logic                  s_awvalid,
    output logic                  s_awready,
    input  logic [AXI_DW-1:0]     s_wdata,
    input  logic [AXI_DW/8-1:0]   s_wstrb,
    input  logic                  s_wlast,
    input  logic                  s_wvalid,
    output logic                  s_wready,
    output logic [AXI_IDW-1:0]    s_bid,
    output logic [1:0]            s_bresp,
    output logic                  s_bvalid,
    input  logic                  s_bready,
    input  logic [AXI_IDW-1:0]    s_arid,
    input  logic [AXI_AW-1:0]     s_araddr,
    input  logic [7:0]            s_arlen,
    input  logic [2:0]            s_arsize,
    input  logic [1:0]            s_arburst,
    input  logic                  s_arvalid,
    output logic                  s_arready,
    output logic [AXI_IDW-1:0]    s_rid,
    output logic [AXI_DW-1:0]     s_rdata,
    output logic [1:0]            s_rresp,
    output logic                  s_rlast,
    output logic                  s_rvalid,
    input  logic                  s_rready
);

    // ------------------------------------------------------------------
    // History window: 64 KiB, registered read
    // ------------------------------------------------------------------
    (* ram_style = "block" *) logic [7:0] win [0:65535];
    logic [15:0] win_waddr, win_raddr;
    logic [7:0]  win_wdata, win_rdata;
    logic        win_we;
    reg   [7:0]  win_rdata_q;

    assign win_rdata = win_rdata_q;
    always @(posedge clk) begin
        if (win_we) win[win_waddr] <= win_wdata;
        win_rdata_q <= win[win_raddr];
    end

    // ------------------------------------------------------------------
    // Byte-at-a-time handshake to software
    // ------------------------------------------------------------------
    logic        in_pending;      // a byte is waiting for the decoder
    logic [7:0]  in_byte;
    logic        in_last_q;
    logic        in_ready, out_valid;
    logic [7:0]  out_data;
    // Output is staged 8 bytes at a time.  The old scheme made reading OUT
    // destructive: the byte was released by the read itself, cleared by an
    // access-derived condition, and every bug in this peripheral came out of
    // that (a clear one access late duplicated every literal).  Reading the
    // staged word is now idempotent and software advances with an explicit
    // write, so a re-read cannot duplicate anything.
    logic [63:0] o_stage;
    logic [3:0]  o_cnt;      // valid bytes in o_stage, 0..8
    // Snapshot taken when software reads the count.  The three accesses that make
    // up a take - count, data, consume - must describe the SAME bytes; without
    // this the decoder stages more in between and the consume, which clears
    // everything, drops them.  That is the 5% the loader measured (10 540 334
    // against 11 097 600).  With the snapshot the data served is the data
    // counted, and the consume removes exactly that many bytes.
    logic [63:0] snap;
    logic [3:0]  snap_n;
    logic        snap_v;
    wire  rd_cnt  = rd_hit && (s_araddr[5:2] == 4'd7);
    logic        dec_done, dec_error;
    logic        done_q, error_q, started;
    logic [31:0] in_count, out_count;
    // Instrumentation, because two different wrappers produced the same wrong
    // output length on silicon (13 190 715 against 11 097 600) while simulation
    // passed.  take_count is what the DECODER actually consumed and last_taken
    // whether the end-of-block flag ever reached it: together they separate "the
    // wrapper loses input" from "the decoder misses the end of the block".
    logic [31:0] take_count;
    logic        last_taken;

    wire busy = started & ~done_q & ~error_q;

    // The decoder takes the pending byte exactly when it says it is ready.
    wire dec_takes = in_pending && in_ready;

    lz4_decoder u_dec (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_data    (in_byte),
        .in_valid   (in_pending),
        .in_last    (in_last_q),
        .in_ready   (in_ready),
        .out_data   (out_data),
        .out_valid  (out_valid),
        .out_ready  (o_cnt < 4'd8), // room in the staging register
        .win_waddr  (win_waddr),
        .win_wdata  (win_wdata),
        .win_we     (win_we),
        .win_raddr  (win_raddr),
        .win_rdata  (win_rdata),
        .done       (dec_done),
        .error      (dec_error)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_pending <= 1'b0; in_byte <= 8'd0; in_last_q <= 1'b0;
            o_stage <= 64'd0; o_cnt <= 4'd0;
            snap <= 64'd0; snap_n <= 4'd0; snap_v <= 1'b0;
            done_q <= 1'b0; error_q <= 1'b0; started <= 1'b0;
            in_count <= 32'd0; out_count <= 32'd0;
            take_count <= 32'd0; last_taken <= 1'b0;
        end else begin
            if (dec_takes) begin
                in_pending <= 1'b0;
                take_count <= take_count + 32'd1;
                last_taken <= in_last_q;      // sticky: did the end flag ever land
            end

            if (out_valid && (o_cnt < 4'd8)) begin
                o_stage   <= o_stage | (({56'd0, out_data}) << ({o_cnt, 3'b000}));
                o_cnt     <= o_cnt + 4'd1;
                out_count <= out_count + 32'd1;
            end
            // Explicit consume: software has taken the staged word.  Only a
            // write does this - never a read.
            // Consume exactly what software read, not everything.  Software reads
            // the count, then the data, then consumes - and the decoder can stage
            // more bytes in between.  Clearing everything dropped those, which is
            // the 5% shortfall the loader reported (10 540 334 bytes against
            // 11 097 600).  The write carries the count; the remainder shifts down
            // so the bytes that arrived keep their order.
            // Freeze the group when software asks for its size, so the data it
            // then reads is the data it counted.
            if (rd_cnt) begin
                snap   <= o_stage;
                snap_n <= o_cnt;
                snap_v <= 1'b1;
            end
            // Consume exactly the bytes the snapshot held.  Anything the decoder
            // staged since stays, shifted down, and is read on the next take.
            if (wr_hit && wr_reg == 4'd0 && s_wdata[1]) begin
                o_cnt   <= (o_cnt >= snap_n) ? (o_cnt - snap_n) : 4'd0;
                o_stage <= o_stage >> ({snap_n, 3'b000});
                snap_v  <= 1'b0;
            end
            if (dec_done)  done_q  <= 1'b1;
            if (dec_error) error_q <= 1'b1;

            if (wr_hit) begin
                case (wr_reg)
                4'd0: if (s_wdata[0]) begin
                    started    <= 1'b1;
                    done_q     <= 1'b0;
                    error_q    <= 1'b0;
                    o_cnt      <= 4'd0;
                    in_pending <= 1'b0;
                    in_count   <= 32'd0;
                    out_count  <= 32'd0;
                    take_count <= 32'd0;
                    last_taken <= 1'b0;
                end
                4'd1: begin
                    in_byte    <= s_wdata[7:0];
                    in_last_q  <= s_wdata[8];
                    in_pending <= 1'b1;
                    in_count   <= in_count + 32'd1;
                end
                default: ;
                endcase
            end

            // Clear on the OUT read ITSELF, using the current address.  rd_reg is
            // loaded at the end of the AR handshake, so testing it here looked one
            // access into the past: out_avail was only released on the access AFTER
            // the OUT read, and the CTRL read in between still reported bit3=1 for a
            // byte already consumed.  Software following the documented pattern
            // (read CTRL, if bit3 read OUT) then read OUT a second time and got the
            // same out_reg back -- one duplicated byte per starvation episode, which
            // is exactly the 13 190 715 produced against 11 097 600 expected.
            // (no read side effect any more: the staged data is only consumed
            // by the explicit write above.)
        end
    end

    // ------------------------------------------------------------------
    // AXI4-Lite write channel
    // ------------------------------------------------------------------
    logic        aw_cap;
    logic [3:0]  wr_reg;
    logic [AXI_IDW-1:0] aw_id_q;
    wire         wr_hit = s_wready && s_wvalid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_awready <= 1'b1; s_wready <= 1'b0; s_bvalid <= 1'b0;
            s_bid <= '0; s_bresp <= 2'b00; aw_cap <= 1'b0;
            aw_id_q <= '0; wr_reg <= 4'd0;
        end else begin
            if (s_awready && s_awvalid) begin
                wr_reg    <= s_awaddr[5:2];
                aw_id_q   <= s_awid;
                aw_cap    <= 1'b1;
                s_awready <= 1'b0;
                s_wready  <= 1'b1;
            end
            if (s_wready && s_wvalid) begin
                s_wready <= 1'b0;
                s_bid    <= aw_id_q;
                s_bvalid <= 1'b1;
            end
            if (s_bvalid && s_bready) begin
                s_bvalid  <= 1'b0;
                s_awready <= 1'b1;
                aw_cap    <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------
    // AXI4-Lite read channel
    // ------------------------------------------------------------------
    logic [3:0]  rd_reg;
    wire         rd_hit = s_arready && s_arvalid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_arready <= 1'b1; s_rvalid <= 1'b0; s_rid <= '0;
            s_rdata <= '0; s_rresp <= 2'b00; s_rlast <= 1'b0; rd_reg <= 4'd0;
        end else begin
            if (s_arready && s_arvalid) begin
                s_arready <= 1'b0;
                s_rvalid  <= 1'b1;
                s_rid     <= s_arid;
                s_rlast   <= 1'b1;
                rd_reg    <= s_araddr[5:2];
                case (s_araddr[5:2])
                // bit4 = "writing a byte now will be taken", which is NOT the
                // same as "no byte is pending": inside a match the decoder owes
                // no byte yet wants none, and a write then would be overwritten
                // at the next clock.  Readiness is the decoder actually asking.
                // Every 32-bit value is REPLICATED into both halves of the 64-bit
                // read word.  The interconnect places the value in the lane the
                // address selects, so a register at an odd 32-bit offset used to
                // read back as zeros - which is exactly what hung the loader here:
                // LZ4_OSTAT at 0x1C always read 0, lz4_take8 never consumed, the
                // staging stayed full and the decoder refused input forever.
                // Replication is the standard way to serve a narrow register on a
                // wide bus, and it makes every offset work.
                4'd0: s_rdata <= {2{{27'd0, (~in_pending && in_ready), (o_cnt != 4'd0),
                                     done_q, error_q, busy}}};
                4'd6: s_rdata <= {2{(snap_v ? snap[31:0] : o_stage[31:0])}};  // low half, frozen group
                4'd7: s_rdata <= {2{{28'd0, o_cnt}}};  // size, and this read freezes the group
                // The staged word is read as two 32-bit halves, not one 64-bit
                // load: a 64-bit MMIO load never completes on this interconnect
                // (no RVALID, core waits forever).  The high half lives at 0x08,
                // the old byte register, which is free now that reading no longer
                // consumes - deliberately NOT at 0x20: everything this peripheral
                // has ever decoded sits between 0x00 and 0x1C, and an address the
                // interconnect does not route answers nothing, which hangs the
                // core exactly the same way.  Stay inside the proven window.
                4'd2: s_rdata <= {2{(snap_v ? snap[63:32] : o_stage[63:32])}}; // high half, frozen group
                4'd3: s_rdata <= {2{out_count}};          // bytes produced
                // 0x10: what the decoder consumed, and whether it ever saw the
                // end-of-block flag.  A short take_count means the wrapper lost
                // input; a full one with the flag clear means the end was never
                // signalled to the decoder.
                4'd4: s_rdata <= {2{take_count}};
                4'd5: s_rdata <= {2{{30'd0, last_taken, started}}};
                default: s_rdata <= '0;
                endcase
            end
            if (s_rvalid && s_rready) begin
                s_rvalid  <= 1'b0;
                s_arready <= 1'b1;
            end
        end
    end

endmodule
