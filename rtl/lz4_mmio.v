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
//                    bit3 = an output group is available, bit4 = input staging empty
//    0x04  IN     W: [31:0] four compressed bytes, consumed one at a time
//    0x14  INLEN  W: compressed block length; in_last is asserted on its last
//                    byte.  Four data bytes fill a 32-bit write, so the end
//                    cannot be a per-byte flag - the length carries it instead.
//    0x18  OLO    R: low  32 bits of the output FIFO head group.  Idempotent:
//                    re-reading it does NOT advance the FIFO.
//    0x08  OHI    R: high 32 bits of the head group.  Reading it COMPLETES the
//                    pair and pops (commits) that group - this is the auto-
//                    increment.  It is not idempotent: see the trade below.
//    0x1C  OSTAT  R: [3:0] whole 8-byte groups ready in the FIFO, [4] started,
//                    [5] the end flag ever reached the decoder.  No side effect.
//    0x14  GC     R: groups software has consumed (a lost group is detectable).
//    0x0C  PROGRESS R: [31:0] bytes the decoder produced
//    0x10  TAKEN  R: bytes the decoder consumed from the input staging
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
    // Input staging: four compressed bytes per MMIO write
    //
    // Software used to write one byte per store - 2.79 M of them for the kernel
    // - each followed by a busy-wait on CTRL.  The register window 0x00-0x1C is
    // full and 0x20 was shown to be unrouted, so no new address is needed or
    // available: the existing IN register at 0x04 already accepts a 32-bit
    // write.  The four bytes it carries are staged here and the decoder eats
    // them one at a time at its own rate, so the CPU's stores AND its polling
    // both fall by four.
    //
    // END OF BLOCK.  A 32-bit write that already carries four data bytes has no
    // spare bit for a per-byte end flag - four bytes plus four flags is 36 bits
    // and does not fit, whatever field the flags are given.  So the end is not a
    // per-write flag: the wrapper is told the block's compressed length once
    // (INLEN, 0x14) and derives in_last from the number of bytes it has already
    // consumed.  The final group is padded and the decoder stops on the right
    // byte whatever its position in the group, which is why no valid-byte count
    // is needed either.  INLEN must be written after the start write, because a
    // start clears it.
    //
    // Lane note: this is a deliberate 32-bit write to 0x04, read from
    // s_wdata[31:0].  Do NOT widen it to a 64-bit store to gain another factor:
    // a 64-bit store on a 32-bit path arrives as two beats whose byte strobes
    // say which bytes are live, and a peripheral that just reads s_wdata[31:0]
    // drops the upper half silently.
    // ------------------------------------------------------------------
    logic [31:0] i_stage;   // staged compressed bytes; byte 0 is consumed next
    logic [2:0]  i_cnt;     // staged bytes not yet consumed, 0..4
    logic [31:0] i_len;     // compressed block length; in_last fires on its last byte
    wire  [7:0]  in_byte   = i_stage[7:0];
    wire         in_last_q = (take_count == (i_len - 32'd1));
    logic        in_ready, out_valid;
    logic [7:0]  out_data;
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

    // ------------------------------------------------------------------
    // Output: a FIFO of frozen 8-byte groups, drained by auto-increment
    //
    // The old drain cost FOUR MMIO accesses per 8-byte group: read the size
    // (0x1C, which also froze the group), read the low half (0x18), read the
    // high half (0x08) and write CTRL bit1 to consume.  Measured on the kernel
    // stream that is 1 387 200 groups x 4 = 5 548 800 accesses for 11 097 600
    // bytes, and it is ~98 % of the 7 060 ms the stage takes (the decoder itself
    // needs ~433 ms).
    //
    // Here the frozen single group becomes a FIFO of DEPTH frozen groups, so the
    // count read amortises over a whole batch and the consume write disappears:
    //
    //     read 0x1C      how many whole groups are ready (once per batch)
    //     read 0x18      low half  - advances nothing, safe to retry
    //     read 0x08      high half - completes the pair and pops the group
    //
    // For a batch of G groups that is 1 + 2G accesses for 8G bytes - 2.0 per
    // group as G grows, the factor of two.  The decoder also runs ahead: out_ready
    // stays high until the FIFO AND the byte accumulator are both full, so it is
    // buffered (DEPTH+1)*8 bytes past the CPU instead of stopping at 8.
    //
    // THE HONEST TRADE.  The old freeze-plus-explicit-consume protocol was safe
    // under retry: a repeated read returned the same bytes and consumption only
    // happened on an explicit write.  The 0x08 read is NOT idempotent - a retried
    // or fault-interrupted 0x08 read pops the next group as well, so it silently
    // advances past data (the loss class of failure).  Three things buy the
    // safety back as far as it can be bought:
    //   * the low half (0x18) has no side effect, so only the high half commits;
    //   * groups_taken (readable at 0x14) counts the groups software has popped,
    //     the same instrument role take_count plays for the input, so a lost
    //     group is DETECTABLE (groups_taken x 8 + tail should equal out_count);
    //   * the boot loader runs with interrupts off and no fault path, so the
    //     exposure is bounded for this boot path - stated explicitly, not left
    //     implicit.
    // A group is frozen into the FIFO when the byte accumulator fills (8 bytes),
    // and the final partial group is pushed once the decoder asserts done; the
    // number of bytes in that last group is out_count mod 8 (out_count, 0x0C, is
    // the decoder's own byte total), so no separate OLEN register is needed and
    // no new address is used - the proven 0x00-0x1C window is respected.
    // ------------------------------------------------------------------
    localparam integer OGRP = 8;    // FIFO depth, in 8-byte groups
    localparam integer OPW  = 3;    // pointer width; OGRP == 2**OPW
    (* ram_style = "distributed" *) logic [63:0] ofifo [0:OGRP-1];
    logic [OPW-1:0] wptr, rptr;     // circular write / read pointers
    logic [3:0]     ofill;          // group entries in the FIFO, 0..OGRP
    logic [63:0]    o_stage;        // byte accumulator for the group being built
    logic [3:0]     o_cnt;          // bytes in o_stage, 0..8
    logic           o_tail_done;    // the final partial group has been pushed
    logic [31:0]    groups_taken;   // groups software popped (lost-group instrument)

    // Room for the decoder's next byte, counting the one already in flight.
    // The decoder decides to emit at cycle N when out_ready is high, but
    // out_valid rises at N+1 - by which time o_cnt has been advanced by the
    // previous pulse.  A plain (o_cnt < 8) therefore lets the decoder emit one
    // byte too many, for which the staging register has no room: the byte is
    // DROPPED silently, once per time the staging saturates, and the stream is
    // short by that many bytes with no error flagged.  Measured on the real
    // kernel prefix: 7 drops over 12 008 output bytes.  The in-flight byte must
    // be counted here.  With the FIFO, the accumulator is only a staging area
    // that empties itself into the FIFO, so this exact test still holds and the
    // extra buffering is the FIFO's, not the accumulator's.
    wire out_ready = ((o_cnt + {3'd0, out_valid}) < 4'd8);

    // Staging is non-empty exactly when a byte is presented to the decoder.
    wire in_valid = (i_cnt != 3'd0);
    wire dec_takes = in_valid && in_ready;

    wire fifo_full = (ofill == OGRP[3:0]);
    // The accumulator is full: freeze it into the FIFO and start a new group.
    wire push_full = (o_cnt == 4'd8) && !fifo_full;
    // The decoder is done and a partial group is still staged: freeze that too.
    wire push_tail = done_q && !o_tail_done && (o_cnt != 4'd0) && (o_cnt < 4'd8)
                     && !fifo_full;
    wire push      = push_full || push_tail;

    wire busy = started & ~done_q & ~error_q;
    // 0x1C status word: the group count in the low 16 bits, then the two
    // input-side flags that used to live at 0x14 (which now carries
    // groups_taken): [5] last_taken, [4] started, [3:0] groups ready.
    wire [31:0] stat_word = {26'd0, last_taken, started, ofill};

    lz4_decoder u_dec (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_data    (in_byte),
        .in_valid   (in_valid),
        .in_last    (in_last_q),
        .in_ready   (in_ready),
        .out_data   (out_data),
        .out_valid  (out_valid),
        .out_ready  (out_ready),    // counts the byte already in flight
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
            i_stage <= 32'd0; i_cnt <= 3'd0; i_len <= 32'd0;
            o_stage <= 64'd0; o_cnt <= 4'd0;
            wptr <= '0; rptr <= '0; ofill <= 4'd0;
            o_tail_done <= 1'b0; groups_taken <= 32'd0;
            done_q <= 1'b0; error_q <= 1'b0; started <= 1'b0;
            in_count <= 32'd0; out_count <= 32'd0;
            take_count <= 32'd0; last_taken <= 1'b0;
        end else begin
            if (dec_takes) begin
                i_stage    <= i_stage >> 8;   // drop the byte just consumed
                i_cnt      <= i_cnt - 3'd1;
                take_count <= take_count + 32'd1;
                last_taken <= in_last_q;      // sticky: did the end flag ever land
            end

            // Accumulate the decoder's output byte into the 8-byte group.
            if (out_valid && (o_cnt < 4'd8)) begin
                o_stage   <= o_stage | (({56'd0, out_data}) << ({o_cnt, 3'b000}));
                o_cnt     <= o_cnt + 4'd1;
                out_count <= out_count + 32'd1;
            end

            // Freeze a group into the FIFO.  Bytes above o_cnt are zero padding
            // (they are never set) so a partial final group reads back as its
            // valid bytes followed by zeros.
            if (push) begin
                ofifo[wptr] <= o_stage;
                wptr        <= wptr + 1'b1;
                o_stage     <= 64'd0;
                o_cnt       <= 4'd0;
                if (push_tail) o_tail_done <= 1'b1;
            end

            // Pop the head group when software reads the high half (0x08).  The
            // low half (0x18) never pops; only a completed pair commits.
            if (pop_req && (ofill != 4'd0)) begin
                rptr         <= rptr + 1'b1;
                groups_taken <= groups_taken + 32'd1;
            end
            // One occupancy update so a push and a pop in the same cycle do not
            // race: +1 for a freeze, -1 for a committed read.
            ofill <= ofill + (push ? 4'd1 : 4'd0)
                           - ((pop_req && (ofill != 4'd0)) ? 4'd1 : 4'd0);

            if (dec_done)  done_q  <= 1'b1;
            if (dec_error) error_q <= 1'b1;

            if (wr_hit) begin
                case (wr_reg)
                4'd0: if (s_wdata[0]) begin
                    started     <= 1'b1;
                    done_q      <= 1'b0;
                    error_q     <= 1'b0;
                    o_stage     <= 64'd0;
                    o_cnt       <= 4'd0;
                    wptr        <= '0;
                    rptr        <= '0;
                    ofill       <= 4'd0;
                    o_tail_done <= 1'b0;
                    groups_taken <= 32'd0;
                    i_stage    <= 32'd0;
                    i_cnt      <= 3'd0;
                    i_len      <= 32'd0;
                    in_count   <= 32'd0;
                    out_count  <= 32'd0;
                    take_count <= 32'd0;
                    last_taken <= 1'b0;
                end
                4'd1: begin
                    i_stage  <= s_wdata[31:0];   // four compressed bytes at once
                    i_cnt    <= 3'd4;
                    in_count <= in_count + 32'd4;
                end
                // Compressed block length.  Written once, after the start write,
                // so the wrapper can assert in_last on the right byte - there is
                // no room for a per-byte end flag in the 32-bit IN write.
                4'd5: i_len <= s_wdata[31:0];
                default: ;
                endcase
            end
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
    // Reading the high half (0x08) commits / pops the head group.
    wire         pop_req = rd_hit && (s_araddr[5:2] == 4'd2);

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
                // Every 32-bit value is REPLICATED into both halves of the 64-bit
                // read word.  The interconnect places the value in the lane the
                // address selects, so a register at an odd 32-bit offset used to
                // read back as zeros - which is exactly what hung the loader here:
                // LZ4_OSTAT at 0x1C always read 0, lz4_take8 never consumed, the
                // staging stayed full and the decoder refused input forever.
                // Replication is the standard way to serve a narrow register on a
                // wide bus, and it makes every offset work.
                case (s_araddr[5:2])
                // bit4 = "the input staging is empty, a four-byte write will be
                // kept".  With four bytes staged, this is the only condition
                // that matters: a write while staging is non-empty would drop
                // whatever the decoder has not yet consumed.  It deliberately no
                // longer asks whether the decoder is ready right now - writing
                // while it is busy inside a match is harmless, the staged bytes
                // simply wait - which is what removes one poll in four.
                // bit3 now means "a whole output group is available to read".
                4'd0: s_rdata <= {2{{27'd0, (i_cnt == 3'd0), (ofill != 4'd0),
                                     done_q, error_q, busy}}};
                // 0x0C: bytes the decoder produced.
                4'd3: s_rdata <= {2{out_count}};
                // 0x10: what the decoder consumed from the input staging, and
                // (in 0x1C) whether it ever saw the end-of-block flag.  A short
                // take_count means the wrapper lost input; a full one with the
                // flag clear means the end was never signalled to the decoder.
                4'd4: s_rdata <= {2{take_count}};
                // 0x14 read: groups software has consumed.  Together with
                // out_count this makes a lost group detectable - the instrument
                // the auto-increment trade has to keep.
                4'd5: s_rdata <= {2{groups_taken}};
                // 0x18: low half of the head group.  IDEMPOTENT - reading it does
                // not advance the FIFO, so a retried low read is harmless.
                4'd6: s_rdata <= {2{ofifo[rptr][31:0]}};
                // 0x1C: [3:0] whole groups ready, [4] started, [5] last_taken.
                // Read-only and side-effect free (the old read froze the group;
                // this one has nothing to freeze).
                4'd7: s_rdata <= {2{stat_word}};
                // 0x08: high half of the head group.  This read COMPLETES the
                // pair and pops (commits) the group - the auto-increment.  It is
                // NOT idempotent; a retry advances past the next group, which is
                // the trade the factor-of-two buys.  The staged word is read as
                // two 32-bit halves, never one 64-bit load: a 64-bit MMIO load
                // never completes on this interconnect (no RVALID, core waits
                // forever).  The high half lives at 0x08, the old byte register,
                // which is free now that the drain is group-based - deliberately
                // NOT at 0x20, because everything this peripheral has ever decoded
                // sits between 0x00 and 0x1C and an address the interconnect does
                // not route answers nothing, which hangs the core the same way.
                4'd2: s_rdata <= {2{ofifo[rptr][63:32]}};
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
