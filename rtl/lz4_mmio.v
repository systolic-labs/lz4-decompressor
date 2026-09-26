// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Systolic Labs
// ============================================================================
//  lz4_mmio.v - LZ4 block decompressor as an AXI4-Lite peripheral
// ----------------------------------------------------------------------------
//  Wraps lz4_decoder so firmware can drive it byte at a time from the existing
//  loader, with no new software infrastructure: write a compressed byte, poll,
//  read a decompressed one.  The CPU stays in the loop between bytes but out of
//  the inner loop, which is what removes the instruction-fetch cost that made the
//  software decoder slow - the decoder itself is datapath, not code.
//
//  This is deliberately the simple first step.  A byte per register access is
//  slower than a DMA engine that feeds the decoder from the flash directly, but
//  it reuses the QSPI engine and AXI path that already work, and it can be
//  verified in simulation before anything is built.
//
//  Register map (word index = addr[5:2], as elsewhere in this SoC):
//    0x00  CTRL   W: bit0 = clear done/error and start a fresh block
//                 R: bit0 = busy, bit1 = error, bit2 = done, bit3 = output byte available
//    0x04  IN     W: [7:0] = byte, [8] = last byte of the block
//    0x08  OUT    R: [7:0] = decompressed byte; reading releases the decoder
//    0x0C  PROGRESS R: bits [15:0] = decompressed bytes produced (low 16 bits)
//
//  Window: 64 KiB of BRAM inside the peripheral, so nothing outside has to supply
//  memory.  Eight BRAM36 tiles on this device, against 418 free.
//
//  Handshake notes that matter, because they are where this kind of wrapper goes
//  wrong:
//    * an AXI write is a single-cycle pulse, but the decoder needs in_valid held
//      until in_ready, so the pending byte is latched and held;
//    * out_ready must not be raised while a byte is still unread, or the decoder
//      would overwrite it - so it is raised only when the holding register is
//      empty, and the decoder backpressures meanwhile;
//    * the read response must echo s_arid into s_rid and the write response must
//      echo s_awid into s_bid.  The SoC routes through axi_riscv_atomics, which
//      matches responses to requests by ID; hardcoding either to zero leaves the
//      CPU waiting forever, which on silicon looks like "banner then silence".
// ============================================================================
module lz4_mmio #(
    parameter integer AXI_AW = 64,
    parameter integer AXI_DW = 64,
    parameter integer AXI_IDW = 8
) (
    input  logic                  clk,
    input  logic                  rst_n,

    // ---- AXI4-Lite slave (full AXI4 signal set; burst/lock/cache ignored) ----
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
    // History window: 64 KiB inside the peripheral
    // ------------------------------------------------------------------
    // Without ram_style Vivado infers distributed RAM for this array, which
    // costs ~10 240 LUTs for 64 KiB - a large fraction of the device for what
    // eight BRAM36 tiles do.  Force the block RAM inference explicitly.
    (* ram_style = "block" *) logic [7:0] win [0:65535];
    logic [15:0] win_waddr, win_raddr;
    logic [7:0]  win_wdata, win_rdata;
    logic        win_we;

    // The read is REGISTERED, giving the one cycle of latency the decoder's
    // contract states.  A combinational read would be more permissive and still
    // pass simulation, but Vivado cannot infer block RAM for an asynchronous
    // read: it falls back to distributed RAM, ~10 240 LUTs for 64 KiB, against
    // eight BRAM36 tiles.  Matching the contract is what makes the memory
    // infer correctly and the timing honest.
    reg [7:0] win_rdata_q;
    assign win_rdata = win_rdata_q;

    always @(posedge clk) begin
        if (win_we) win[win_waddr] <= win_wdata;
        win_rdata_q <= win[win_raddr];
    end

    // ------------------------------------------------------------------
    // Decoder instance
    // ------------------------------------------------------------------
    logic        out_avail;       // a decompressed byte is waiting for the CPU
    logic        in_pending;      // a byte is waiting for the decoder
    logic [7:0]  in_byte;
    logic        in_last_q;
    logic        in_ready;
    logic        out_ready;
    logic [7:0]  out_data;
    logic        out_valid;
    logic        dec_done, dec_error;

    assign out_ready = ~out_avail;   // never overwrite an unread byte

    lz4_decoder u_dec (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_data    (in_byte),
        .in_valid   (in_pending),
        .in_last    (in_last_q),
        .in_ready   (in_ready),
        .out_data   (out_data),
        .out_valid  (out_valid),
        .out_ready  (out_ready),
        .win_waddr  (win_waddr),
        .win_wdata  (win_wdata),
        .win_we     (win_we),
        .win_raddr  (win_raddr),
        .win_rdata  (win_rdata),
        .done       (dec_done),
        .error      (dec_error)
    );

    // ------------------------------------------------------------------
    // Peripheral state
    // ------------------------------------------------------------------
    logic [7:0]  out_reg;
    logic        done_q, error_q;
    logic        started;
    logic [31:0] out_count;

    // busy is only meaningful once a block has been started; before that the
    // decoder is idle and reporting "busy" would invite a software deadlock.
    wire busy = started & ~done_q & ~error_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_pending <= 1'b0;
            in_byte    <= 8'd0;
            in_last_q  <= 1'b0;
            out_avail  <= 1'b0;
            out_reg    <= 8'd0;
            done_q     <= 1'b0;
            error_q    <= 1'b0;
            started    <= 1'b0;
            out_count  <= 32'd0;
        end else begin
            // -- input: hold the byte until the decoder takes it --------------
            if (in_pending && in_ready) in_pending <= 1'b0;

            // -- output: latch a produced byte if the register is free --------
            if (out_valid && !out_avail) begin
                out_reg   <= out_data;
                out_avail <= 1'b1;
                out_count <= out_count + 32'd1;
            end

            // -- decoder status is sticky until the next start ----------------
            if (dec_done)  done_q  <= 1'b1;
            if (dec_error) error_q <= 1'b1;

            // -- register writes ---------------------------------------------
            if (wr_hit) begin
                case (wr_reg)
                4'd0: begin                       // CTRL: start a fresh block
                    if (s_wdata[0]) begin
                        started    <= 1'b1;
                        done_q     <= 1'b0;
                        error_q    <= 1'b0;
                        out_count  <= 32'd0;
                        out_avail  <= 1'b0;
                        in_pending <= 1'b0;
                    end
                end
                4'd1: begin                       // IN: byte, and the last flag
                    in_byte    <= s_wdata[7:0];
                    in_last_q  <= s_wdata[8];
                    in_pending <= 1'b1;
                end
                default: ;
                endcase
            end

            // -- reading OUT releases the decoder ----------------------------
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
            s_awready <= 1'b1;
            s_wready  <= 1'b0;
            s_bvalid  <= 1'b0;
            s_bid     <= '0;
            s_bresp   <= 2'b00;
            aw_cap    <= 1'b0;
            aw_id_q   <= '0;
            wr_reg    <= 4'd0;
        end else begin
            if (s_awready && s_awvalid) begin
                wr_reg    <= s_awaddr[5:2];
                aw_id_q   <= s_awid;          // must be echoed in B
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
            s_arready <= 1'b1;
            s_rvalid  <= 1'b0;
            s_rid     <= '0;
            s_rdata   <= '0;
            s_rresp   <= 2'b00;
            s_rlast   <= 1'b0;
            rd_reg    <= 4'd0;
        end else begin
            if (s_arready && s_arvalid) begin
                s_arready <= 1'b0;
                s_rvalid  <= 1'b1;
                s_rid     <= s_arid;          // must be echoed in R
                s_rlast   <= 1'b1;
                rd_reg    <= s_araddr[5:2];
                case (s_araddr[5:2])
                4'd0: s_rdata <= {60'd0, out_avail, done_q, error_q, busy};
                4'd2: s_rdata <= {56'd0, out_reg};
                4'd3: s_rdata <= {32'd0, out_count};
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
