// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Systolic Labs
// ============================================================================
//  lz4_decoder.v - LZ4 block-format decompressor, byte-serial, plain Verilog
// ----------------------------------------------------------------------------
//  Decodes a raw LZ4 *block* (what LZ4_compress_default emits), not a frame.
//
//  Written from the published LZ4 block-format description; no code was taken
//  from any unlicensed HDL source.  lib/lz4.c in the reference implementation is
//  BSD-2-Clause and the AMD Vitis LZ4 kernels are Apache-2.0; neither is copied
//  here - they served as functional references only.  Intended for release under
//  Apache-2.0.
//
//  FORMAT.  A block is a chain of sequences.  Each begins with a token byte
//  whose high nibble is the literal count and low nibble the match length minus
//  4.  A nibble of 15 means "add the following bytes (255 = continue)".  Then
//  the literals, then a two-byte little-endian offset back into the output
//  history - unless this is the final sequence, which carries literals only.
//
//  END OF BLOCK.  There is no terminator: a block ends after the final
//  sequence's literals, so the caller signals the end by asserting in_last with
//  that last byte.  in_last is high for one cycle only and is therefore latched;
//  the block finishes when the literals run out with the latch set.  A decoder
//  that waits to observe in_last *after* the final literal never leaves S_LITERAL.
//
//  WINDOW.  64 KiB addressed by the low 16 bits of the output position, so the
//  address *is* the counter and the buffer is circular with no head/tail logic.
//
//  ADDRESSING.  Two addresses, kept deliberately apart:
//    * the write address is a register (waddr), loaded in the cycle a byte is
//      produced, because out_valid rises one cycle later and opos has already
//      advanced by then.  Deriving the write address from opos at emit time
//      stores every byte one slot late - harmless until a match reads it back.
//    * the read address is combinational: opos - offset, truncated to 16 bits.
//      That single expression is also what makes overlapping matches work, since
//      the source advances with the output.
//
//  TIMING.  One byte per clock for literals; a match byte takes two clocks, one
//  to present the read address and one to consume it, because a synchronous
//  memory returns data the cycle after its address is registered.
//
//  MEMORY CONTRACT.  win_rdata is the byte at the win_addr presented in the
//  *previous* cycle (one cycle of read latency); win_we/win_wdata write on the
//  rising edge with the write address taking priority over the read.
//
//  HANDSHAKE.  in_ready is combinational; a byte is consumed on
//  in_valid && in_ready.  The match phase stalls on out_ready rather than
//  dropping bytes.
// ============================================================================
`default_nettype none

module lz4_decoder (
    input  wire        clk,
    input  wire        rst_n,

    // ---- compressed input stream ----
    input  wire [7:0]  in_data,
    input  wire        in_valid,
    input  wire        in_last,      // high with the final byte of the block
    output wire        in_ready,

    // ---- decompressed output stream ----
    output reg  [7:0]  out_data,
    output reg         out_valid,
    input  wire        out_ready,

    // ---- history window, 64 KiB, separate read and write ports ----
    // A single-port memory cannot read and write in the same cycle, and a byte
    // is written in the very cycle its successor is read - which is what made
    // the first match byte decode and the rest come back undefined.
    output wire [15:0] win_waddr,
    output wire [7:0]  win_wdata,
    output wire        win_we,
    output wire [15:0] win_raddr,
    input  wire [7:0]  win_rdata,

    // ---- status ----
    output reg         done,
    output reg         error
);

    localparam [3:0] S_TOKEN   = 4'd0,
                     S_LITEXT  = 4'd1,
                     S_LITERAL = 4'd2,
                     S_OFF_LO  = 4'd3,
                     S_OFF_HI  = 4'd4,
                     S_MATEXT  = 4'd5,
                     S_MREAD   = 4'd6,
                     S_MWRITE  = 4'd7,
                     S_DONE    = 4'd8;

    reg [3:0]  st;
    reg [31:0] opos;        // output bytes produced; low 16 bits index the window
    reg [15:0] waddr;       // window address of the byte currently on out_data
    // Sequence lengths are 32-bit: LZ4 bounds the *distance* to 64 KiB but
    // places no bound on a sequence's *length*.  The kernel's own stream has a
    // single match of 98 576 bytes, so 16-bit counters silently overflow and
    // the copy runs off the end of the sequence.
    reg [31:0] litlen;      // literals remaining in this sequence
    reg [31:0] mlen;        // match bytes remaining
    reg [15:0] offset;      // match distance
    reg [15:0] offset_lo;   // low byte of the offset while the high byte is read
    reg        saved_last;  // set once the final input byte has been consumed

    // Write forwarding for the window.  A read-before-write BRAM returns the OLD
    // contents when the same address is read and written in one cycle, and an
    // LZ4 match with offset 1 is a run whose source is the byte just written -
    // exactly that case.  Without this every run drifts from its second byte on.
    // The read is issued a cycle before its data is used, so the forwarded byte
    // has to be latched alongside it rather than bypassed combinationally.
    reg  [7:0] rd_bypass;
    reg        rd_bypass_v;

    // ---------------------------------------------------------------------
    // Handshake: consumed on in_valid && in_ready, and every state that reads
    // an input byte asserts ready combinationally.
    // ---------------------------------------------------------------------
    // Backpressure.  in_ready used to ignore the output holding register
    // entirely, so the decoder would consume an input byte, emit its one-cycle
    // out_valid pulse, and have the wrapper drop it because out_avail was still
    // set - the byte was written to the window but never reached software, and
    // the sequence lost a byte.  On silicon the CPU's drain is far slower than
    // the decoder, so that came up constantly.  Refusing input while a byte is
    // waiting costs nothing: the decoder drains its output on its own.
    assign in_ready = (!out_ready) ? 1'b0 :
                      (st == S_TOKEN)   ? 1'b1 :
                      (st == S_LITEXT)  ? 1'b1 :
                      (st == S_OFF_LO)  ? 1'b1 :
                      (st == S_OFF_HI)  ? 1'b1 :
                      (st == S_MATEXT)  ? 1'b1 :
                      (st == S_LITERAL && litlen != 16'd0) ? 1'b1 : 1'b0;

    wire taking = in_valid && in_ready;

    // (opos - offset) truncated to 16 bits is already the circular address, and
    // it is the correct source for every byte of an overlapping match too.
    wire [15:0] rd_addr = opos[15:0] - offset;

    // The write address is the one latched when the byte was produced, and the
    // read address is presented every cycle: with separate ports they no longer
    // contend, so a byte can be written while its successor is fetched.
    assign win_we    = out_valid;
    assign win_waddr = waddr;
    assign win_wdata = out_data;
    assign win_raddr = rd_addr;

    wire [7:0] rd_eff = rd_bypass_v ? rd_bypass : win_rdata;

    // An offset reaching before the start of the output is invalid.  This must
    // compare against the FULL position, not opos[15:0]: the low 16 bits wrap
    // every 64 KiB, so past the first window a valid offset would look larger
    // than the position and be rejected - the decoder stops exactly at 65536.
    wire [32:0] offset_ext = {17'd0, offset};
    wire [32:0] opos_ext   = {1'b0, opos};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st         <= S_TOKEN;
            opos       <= 32'd0;
            waddr      <= 16'd0;
            litlen     <= 16'd0;
            mlen       <= 16'd0;
            offset     <= 16'd0;
            offset_lo  <= 16'd0;
            saved_last <= 1'b0;
            out_data   <= 8'd0;
            out_valid  <= 1'b0;
            rd_bypass  <= 8'd0;
            rd_bypass_v<= 1'b0;
            done       <= 1'b0;
            error      <= 1'b0;
        end else begin
            done      <= 1'b0;
            out_valid <= 1'b0;

            // Latch the forwarded byte when this cycle's read hits the address the
            // emitted byte is being written to.
            if (win_we && (win_waddr == win_raddr)) begin
                rd_bypass   <= out_data;
                rd_bypass_v <= 1'b1;
            end else begin
                rd_bypass_v <= 1'b0;
            end

            case (st)

            // ------------------------------------------------------- token
            S_TOKEN: begin
                if (taking) begin
                    litlen     <= {12'd0, in_data[7:4]};
                    mlen       <= {12'd0, in_data[3:0]};
                    saved_last <= in_last;
                    st         <= (in_data[7:4] == 4'd15) ? S_LITEXT : S_LITERAL;
                end
            end

            // ---------------------------------- literal-length continuation
            S_LITEXT: begin
                if (taking) begin
                    litlen     <= litlen + {8'd0, in_data};
                    saved_last <= in_last;
                    if (in_data != 8'hFF) st <= S_LITERAL;
                end
            end

            // ---------------------------------------------------- literals
            S_LITERAL: begin
                if (litlen == 32'd0) begin
                    // Sequence done; the block ends here if the last input byte
                    // was consumed in this state.
                    st <= saved_last ? S_DONE : S_OFF_LO;
                end else if (taking) begin
                    out_data   <= in_data;
                    out_valid  <= 1'b1;
                    waddr      <= opos[15:0];   // address of *this* byte
                    opos       <= opos + 32'd1;
                    litlen     <= litlen - 32'd1;
                    saved_last <= in_last;
                end
            end

            // ------------------------------------------------ match offset
            S_OFF_LO: begin
                if (taking) begin
                    offset_lo <= {8'd0, in_data};
                    st        <= S_OFF_HI;
                end
            end
            S_OFF_HI: begin
                if (taking) begin
                    offset <= {in_data, offset_lo[7:0]};
                    if ({17'd0, in_data, offset_lo[7:0]} > opos_ext) begin
                        error <= 1'b1;
                        st    <= S_DONE;
                    end else begin
                        mlen <= mlen + 32'd4;   // the nibble counts length - 4
                        st   <= (mlen == 16'd15) ? S_MATEXT : S_MREAD;
                    end
                end
            end

            // ------------------------------------ match-length continuation
            S_MATEXT: begin
                if (taking) begin
                    mlen <= mlen + {8'd0, in_data};
                    if (in_data != 8'hFF) st <= S_MREAD;
                end
            end

            // ----------------------------------------------- match: address
            // rd_addr is combinational and already presented; this cycle exists
            // only to give the memory its one cycle of read latency.
            S_MREAD: begin
                if (mlen == 32'd0) st <= S_TOKEN;
                else                st <= S_MWRITE;
            end

            // ------------------------------------------------ match: write
            S_MWRITE: begin
                if (out_ready) begin
                    out_data  <= rd_eff;        // read issued in S_MREAD, forwarded if it aliased the write
                    out_valid <= 1'b1;
                    waddr     <= opos[15:0];
                    opos      <= opos + 32'd1;
                    mlen      <= mlen - 32'd1;
                    st        <= (mlen <= 32'd1) ? S_TOKEN : S_MREAD;
                end
            end

            // -------------------------------------------------------- done
            S_DONE: begin
                done <= 1'b1;
            end

            endcase
        end
    end

endmodule

`default_nettype wire
