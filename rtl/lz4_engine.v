// ============================================================================
//  lz4_engine.v - LZ4 accelerator as an AXI4-Lite slave + AXI4 master (SoC)
// ----------------------------------------------------------------------------
//  Step 5 of doc/lz4_dma_engine.md.  This is the SoC-facing wrapper around
//  lz4_axi: a register file the CPU programmes, and an AXI4 master that reads
//  the compressed block from memory and writes the decompressed block back.
//  One decoder, one 64 KiB window, one AXI master - the CPU never touches a
//  byte of the stream.
//
//  Register map (word index = s_araddr[5:2], the proven 0x00-0x1C window):
//    0x00  CTRL   W: bit0 start (self-clearing)   R: bit0 busy, bit1 done, bit2 error
//    0x04  SRC_L  W/R  source address [31:0]      (compressed block, in DDR4)
//    0x08  SRC_H  W/R  source address [63:32]
//    0x0C  DST_L  W/R  destination address [31:0] (decompressed block, in DDR4)
//    0x10  DST_H  W/R  destination address [63:32]
//    0x14  LEN    W/R  compressed length in bytes
//    0x18  OUT    R    decompressed bytes produced
//    0x1C  RES    R    reserved reads 0
//
//  Every 32-bit value is replicated into both halves of the 64-bit read word,
//  for the same reason the MMIO wrapper does it: the interconnect places the
//  value in the lane the address selects, so a narrow register at an odd offset
//  otherwise reads back as zeros.
//
//  The read response must echo s_arid into s_rid and the write response s_awid
//  into s_bid: the SoC routes through axi_riscv_atomics, which matches responses
//  to requests by ID.
// ============================================================================
module lz4_engine #(
    parameter integer AXI_AW  = 64,
    parameter integer AXI_DW  = 64,
    parameter integer AXI_IDW = 4      // master ID width (= AxiIdWidthMaster)
) (
    input  logic                clk,
    input  logic                rst_n,

    // ---- AXI4-Lite slave: control registers ----
    input  logic [AXI_IDW-1:0]  s_awid,
    input  logic [AXI_AW-1:0]   s_awaddr,
    input  logic [7:0]          s_awlen,
    input  logic [2:0]          s_awsize,
    input  logic [1:0]          s_awburst,
    input  logic                s_awvalid,
    output logic                s_awready,
    input  logic [AXI_DW-1:0]   s_wdata,
    input  logic [AXI_DW/8-1:0] s_wstrb,
    input  logic                s_wlast,
    input  logic                s_wvalid,
    output logic                s_wready,
    output logic [AXI_IDW-1:0]  s_bid,
    output logic [1:0]          s_bresp,
    output logic                s_bvalid,
    input  logic                s_bready,
    input  logic [AXI_IDW-1:0]  s_arid,
    input  logic [AXI_AW-1:0]   s_araddr,
    input  logic [7:0]          s_arlen,
    input  logic [2:0]          s_arsize,
    input  logic [1:0]          s_arburst,
    input  logic                s_arvalid,
    output logic                s_arready,
    output logic [AXI_IDW-1:0]  s_rid,
    output logic [AXI_DW-1:0]   s_rdata,
    output logic [1:0]          s_rresp,
    output logic                s_rlast,
    output logic                s_rvalid,
    input  logic                s_rready,

    // ---- AXI4 master: the byte stream ----
    output logic [AXI_IDW-1:0]  m_arid,
    output logic [AXI_AW-1:0]   m_araddr,
    output logic [7:0]          m_arlen,
    output logic [2:0]          m_arsize,
    output logic [1:0]          m_arburst,
    output logic                m_arvalid,
    input  logic                m_arready,
    input  logic [AXI_IDW-1:0]  m_rid,
    input  logic [AXI_DW-1:0]   m_rdata,
    input  logic [1:0]          m_rresp,
    input  logic                m_rlast,
    input  logic                m_rvalid,
    output logic                m_rready,
    output logic [AXI_IDW-1:0]  m_awid,
    output logic [AXI_AW-1:0]   m_awaddr,
    output logic [7:0]          m_awlen,
    output logic [2:0]          m_awsize,
    output logic [1:0]          m_awburst,
    output logic                m_awvalid,
    input  logic                m_awready,
    output logic [AXI_DW-1:0]   m_wdata,
    output logic [AXI_DW/8-1:0] m_wstrb,
    output logic                m_wlast,
    output logic                m_wvalid,
    input  logic                m_wready,
    input  logic [AXI_IDW-1:0]  m_bid,
    input  logic [1:0]          m_bresp,
    input  logic                m_bvalid,
    output logic                m_bready
);

    // ------------------------------------------------------------------
    // Registers
    // ------------------------------------------------------------------
    logic [63:0] src_q, dst_q;
    logic [31:0] len_q;
    logic        start_q;
    logic        busy, done, error;
    logic [31:0] out_count;
    logic [31:0] dbg_rd, dbg_wr, dbg_feed, dbg_err;
    logic [63:0] dbg_sum;
    logic [2:0]  dbg_sel;

    // 0x1C selects which debug value a read returns: 0 read words, 1 written
    // words, 2 bytes fed, 3 non-OKAY responses, 4/5 the source XOR lo/hi.
    wire [31:0] dbg_val = (dbg_sel == 2'd0) ? dbg_rd   :
                          (dbg_sel == 2'd1) ? dbg_wr   :
                          (dbg_sel == 2'd2) ? dbg_feed :
                          (dbg_sel == 2'd3) ? dbg_err  :
                          (dbg_sel == 2'd4) ? dbg_sum[31:0] : dbg_sum[63:32];

    lz4_axi #(
        .AXI_AW (AXI_AW), .AXI_DW (AXI_DW), .AXI_IDW (AXI_IDW)
    ) u_axi (
        .clk(clk), .rst_n(rst_n),
        .start(start_q), .src_addr(src_q), .dst_addr(dst_q), .ilen(len_q),
        .busy(busy), .done(done), .error(error), .out_count(out_count),
        .dbg_rd(dbg_rd), .dbg_wr(dbg_wr), .dbg_feed(dbg_feed), .dbg_err(dbg_err),
        .dbg_sum(dbg_sum),
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

    // ------------------------------------------------------------------
    // AXI4-Lite write channel
    // ------------------------------------------------------------------
    logic [3:0] wr_reg;
    logic [AXI_IDW-1:0] aw_id_q;
    wire  wr_hit = s_wready && s_wvalid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_awready <= 1'b1; s_wready <= 1'b0; s_bvalid <= 1'b0;
            s_bid <= '0; s_bresp <= 2'b00; aw_id_q <= '0; wr_reg <= 4'd0;
        end else begin
            if (s_awready && s_awvalid) begin
                wr_reg    <= s_awaddr[5:2];
                aw_id_q   <= s_awid;
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
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            src_q <= 64'd0; dst_q <= 64'd0; len_q <= 32'd0; start_q <= 1'b0;
            dbg_sel <= 2'd0;
        end else begin
            start_q <= 1'b0;                       // self-clearing pulse
            if (wr_hit) case (wr_reg)
            4'd0: if (s_wdata[0]) start_q <= 1'b1;
            4'd1: src_q[31:0]  <= s_wdata[31:0];
            4'd2: src_q[63:32] <= s_wdata[31:0];
            4'd3: dst_q[31:0]  <= s_wdata[31:0];
            4'd4: dst_q[63:32] <= s_wdata[31:0];
            4'd5: len_q        <= s_wdata[31:0];
            4'd7: dbg_sel      <= s_wdata[2:0];
            default: ;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // AXI4-Lite read channel
    // ------------------------------------------------------------------
    wire rd_hit = s_arready && s_arvalid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_arready <= 1'b1; s_rvalid <= 1'b0; s_rid <= '0;
            s_rdata <= '0; s_rresp <= 2'b00; s_rlast <= 1'b0;
        end else begin
            if (s_arready && s_arvalid) begin
                s_arready <= 1'b0;
                s_rvalid  <= 1'b1;
                s_rid     <= s_arid;
                s_rlast   <= 1'b1;
                case (s_araddr[5:2])
                4'd0: s_rdata <= {2{{29'd0, error, done, busy}}};
                4'd1: s_rdata <= {2{src_q[31:0]}};
                4'd2: s_rdata <= {2{src_q[63:32]}};
                4'd3: s_rdata <= {2{dst_q[31:0]}};
                4'd4: s_rdata <= {2{dst_q[63:32]}};
                4'd5: s_rdata <= {2{len_q}};
                4'd6: s_rdata <= {2{out_count}};
                4'd7: s_rdata <= {2{dbg_val}};
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
