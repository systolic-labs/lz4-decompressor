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
    logic        out_avail;
    logic [7:0]  out_reg;
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
        .out_ready  (~out_avail),   // never overwrite an unread byte
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
            out_avail <= 1'b0;  out_reg <= 8'd0;
            done_q <= 1'b0; error_q <= 1'b0; started <= 1'b0;
            in_count <= 32'd0; out_count <= 32'd0;
            take_count <= 32'd0; last_taken <= 1'b0;
        end else begin
            if (dec_takes) begin
                in_pending <= 1'b0;
                take_count <= take_count + 32'd1;
                last_taken <= in_last_q;      // sticky: did the end flag ever land
            end

            if (out_valid && !out_avail) begin
                out_reg   <= out_data;
                out_avail <= 1'b1;
                out_count <= out_count + 32'd1;
            end
            if (dec_done)  done_q  <= 1'b1;
            if (dec_error) error_q <= 1'b1;

            if (wr_hit) begin
                case (wr_reg)
                4'd0: if (s_wdata[0]) begin
                    started    <= 1'b1;
                    done_q     <= 1'b0;
                    error_q    <= 1'b0;
                    out_avail  <= 1'b0;
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

            if (rd_hit && rd_reg == 4'd2) out_avail <= 1'b0;
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
                4'd0: s_rdata <= {59'd0, (~in_pending && in_ready), out_avail,
                                  done_q, error_q, busy};
                4'd2: s_rdata <= {56'd0, out_reg};
                4'd3: s_rdata <= {32'd0, out_count};      // bytes produced
                // 0x10: what the decoder consumed, and whether it ever saw the
                // end-of-block flag.  A short take_count means the wrapper lost
                // input; a full one with the flag clear means the end was never
                // signalled to the decoder.
                4'd4: s_rdata <= {32'd0, take_count};
                4'd5: s_rdata <= {30'd0, last_taken, started};
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
