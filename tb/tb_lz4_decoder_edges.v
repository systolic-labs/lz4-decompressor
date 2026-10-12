// ============================================================================
//  tb_lz4_decoder_edges.v - directed bench for the decoder's edge cases
// ----------------------------------------------------------------------------
//  The vector benches feed one stream each and check it end to end.  They do not
//  reach the corners a reviewer asks about, so this bench builds its own stimuli
//  and drives them deliberately:
//
//    A. offsets 1..16, long matches.  An offset below the memory's read latency
//       is the run case: the match source is the byte being written.  The window
//       model below is read-first with one cycle of latency, exactly the
//       decoder's contract, so a missing forwarding path shows up immediately.
//    B. the same blocks with out_ready driven by an LFSR.  Backpressure that
//       arrives in the middle of a sequence must stall the read address, the
//       length counter and the output together; a one-cycle skew loses or
//       duplicates a byte.
//    C. a stream that ends on a 0xFF length-extension byte.  The extension says
//       "continue" and the stream stops: the decoder must report error, not wait
//       forever for input that will never come.
//    D. a block with offset 0.  LZ4 requires offset >= 1; accepting it silently
//       turns a corrupt stream into a plausible-looking run.
//    E. a token that declares a match and then ends after its literals.  The
//       declared match is missing, so the block is truncated.
//
//  STIMULUS TIMING.  A byte is presented for exactly one cycle and the next is
//  driven on the edge that observes its acceptance.  Driving it for a second
//  cycle would be a bench bug, not a DUT bug: in S_LITERAL the decoder is ready
//  every cycle, so a held byte is consumed twice - the run then starts with the
//  token byte appearing in the output.
//
//  Only ONE line in this file prints "RESULT": the final verdict.
//
//  iverilog -g2012 -o tb_edges tb_lz4_decoder_edges.v lz4_decoder.v && vvp tb_edges
// ============================================================================
`timescale 1ns/10ps

module tb_lz4_decoder_edges;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  [7:0] in_data  = 8'd0;
    reg        in_valid = 1'b0;
    reg        in_last  = 1'b0;
    wire       in_ready;
    wire [7:0] out_data;
    wire       out_valid;
    reg        out_ready = 1'b0;
    wire [15:0] win_waddr, win_raddr;
    wire [7:0]  win_wdata, win_rdata;
    wire        win_we;
    wire        done, error;

    lz4_decoder dut (
        .clk(clk), .rst_n(rst_n),
        .in_data(in_data), .in_valid(in_valid), .in_last(in_last), .in_ready(in_ready),
        .out_data(out_data), .out_valid(out_valid), .out_ready(out_ready),
        .win_waddr(win_waddr), .win_wdata(win_wdata), .win_we(win_we),
        .win_raddr(win_raddr), .win_rdata(win_rdata),
        .done(done), .error(error)
    );

    // 64 KiB window, read-first, one cycle of read latency: the contract the
    // decoder states.  A read and a write to the same address in one cycle
    // returns the OLD byte, which is why offset 1 needs the forwarding path.
    reg [7:0] mem [0:65535];
    reg [7:0] rd;
    always @(posedge clk) begin
        if (win_we) mem[win_waddr] <= win_wdata;
        rd <= mem[win_raddr];
    end
    assign win_rdata = rd;

    localparam integer MAXGOT = 65536;

    reg [7:0]  stim  [0:4095];
    reg [7:0]  expct [0:MAXGOT-1];
    reg [7:0]  got   [0:MAXGOT-1];
    integer    stim_len, exp_len, got_len;

    reg        bp_mode;
    reg [31:0] lfsr;
    reg        done_seen, error_seen;

    // Registered copy of the handshake, sampled at the posedge with the values
    // the DUT acted on.  The stimulus process reads it at the NEGEDGE, where the
    // update has settled: reading a registered signal right after @(posedge) in
    // another process returns the value from before that edge, which would make
    // the stimulus one cycle late and present every byte twice.
    reg taken_q;
    always @(posedge clk) taken_q <= (in_valid && in_ready);

    // Output side: backpressure policy, collection and status.  Sampling the
    // pre-edge values of out_valid/out_ready is what the DUT acted on.
    //
    // ov_pulses counts every cycle out_valid is high; hs_count counts the ones
    // that were actually accepted.  A producer that withdraws a byte without a
    // handshake shows up as ov_pulses > hs_count, and that is a lost byte - the
    // two counters are what separate a DUT fault from a bench artefact.
    integer ov_pulses, hs_count;
    always @(posedge clk) begin
        if (bp_mode) begin
            lfsr      <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
            out_ready <= lfsr[2];
        end else begin
            out_ready <= 1'b1;
        end
        if (out_valid) ov_pulses = ov_pulses + 1;
        if (out_valid && out_ready) begin
            hs_count = hs_count + 1;
            if (got_len < MAXGOT) got[got_len] = out_data;
            got_len = got_len + 1;
        end
        if (done)  done_seen  = 1'b1;
        if (error) error_seen = 1'b1;
    end

    integer fails;
    integer cases;

    task reset_dut;
        begin
            rst_n    = 1'b0;
            in_valid = 1'b0;
            in_last  = 1'b0;
            repeat (4) @(posedge clk);
            rst_n = 1'b1;
            @(posedge clk);
        end
    endtask

    // Drive the whole stimulus, then watch for done/error until `watch` cycles.
    // Returning on a watchdog expiry instead of blocking is the point: a stream
    // the decoder never leaves must be reported, not hang the run.
    task run_block(input integer len, input integer last_idx, input integer bp,
                   input integer watch);
        integer j, cyc;
        begin
            reset_dut;
            got_len = 0; done_seen = 1'b0; error_seen = 1'b0;
            ov_pulses = 0; hs_count = 0;
            lfsr = 32'h1234_5678;
            bp_mode = bp;
            j = 0; cyc = 0;
            in_data = 8'd0; in_valid = 1'b0; in_last = 1'b0;
            while (!done_seen && !error_seen && cyc < watch) begin
                @(negedge clk);
                cyc = cyc + 1;
                // the byte presented during the cycle that just ended was taken
                if (taken_q) j = j + 1;
                if (j < len) begin
                    in_data  = stim[j];
                    in_valid = 1'b1;
                    in_last  = (j == last_idx);
                end else begin
                    in_valid = 1'b0;
                    in_last  = 1'b0;
                end
            end
            in_valid = 1'b0;
            in_last  = 1'b0;
            @(negedge clk);
        end
    endtask

    // prefix literals + one match(offset, mlen) + trailing literals.
    // mlen must be >= 19 so the match nibble is 15 and the extension runs.
    task build_run(input integer plen, input integer offset, input integer mlen,
                   input integer tlen, input [7:0] seed);
        integer j, ext, nib;
        begin
            stim_len = 0;
            nib = (plen >= 15) ? 15 : plen;
            stim[stim_len] = {nib[3:0], 4'hF};   stim_len = stim_len + 1;
            if (plen >= 15) begin
                ext = plen - 15;
                while (ext >= 255) begin stim[stim_len] = 8'hFF; stim_len = stim_len + 1; ext = ext - 255; end
                stim[stim_len] = ext[7:0];       stim_len = stim_len + 1;
            end
            for (j = 0; j < plen; j = j + 1) begin
                stim[stim_len] = seed + j[7:0];  stim_len = stim_len + 1;
            end
            stim[stim_len] = offset[7:0];        stim_len = stim_len + 1;
            stim[stim_len] = offset[15:8];       stim_len = stim_len + 1;
            ext = mlen - 19;
            while (ext >= 255) begin stim[stim_len] = 8'hFF; stim_len = stim_len + 1; ext = ext - 255; end
            stim[stim_len] = ext[7:0];           stim_len = stim_len + 1;
            stim[stim_len] = {tlen[3:0], 4'h0};  stim_len = stim_len + 1;
            for (j = 0; j < tlen; j = j + 1) begin
                stim[stim_len] = 8'hE0 + j[7:0]; stim_len = stim_len + 1;
            end

            // Expected: the match copies from (position - offset), so the
            // recurrence below is exactly what the decoder must produce.
            exp_len = 0;
            for (j = 0; j < plen; j = j + 1) begin
                expct[exp_len] = seed + j[7:0];  exp_len = exp_len + 1;
            end
            if (offset > 0) begin
                for (j = 0; j < mlen; j = j + 1) begin
                    expct[exp_len] = expct[exp_len - offset]; exp_len = exp_len + 1;
                end
            end else begin
                for (j = 0; j < mlen; j = j + 1) begin
                    expct[exp_len] = 8'h00;      exp_len = exp_len + 1;
                end
            end
            for (j = 0; j < tlen; j = j + 1) begin
                expct[exp_len] = 8'hE0 + j[7:0]; exp_len = exp_len + 1;
            end
        end
    endtask

    // Compare and print one line.  `want_err` selects which of the two outcomes
    // the case is asserting.
    task report(input integer want_err);
        integer k, bad;
        begin
            cases = cases + 1;
            bad = -1;
            if (want_err) begin
                if (error_seen) begin
                    $display("    [ok]   error reported after %0d bytes out", got_len);
                end else begin
                    $display("    [BAD]  expected error, got none (done=%b, %0d bytes out)",
                             done_seen, got_len);
                    fails = fails + 1;
                end
            end else begin
                for (k = 0; k < exp_len; k = k + 1)
                    if (got[k] !== expct[k] && bad < 0) bad = k;
                if (bad >= 0) begin
                    $display("    [BAD]  byte %0d got %02x want %02x  (valid pulses %0d, accepted %0d)",
                             bad, got[bad], expct[bad], ov_pulses, hs_count);
                    fails = fails + 1;
                end else if (got_len != exp_len) begin
                    $display("    [BAD]  %0d bytes out, %0d expected  (valid pulses %0d, accepted %0d)",
                             got_len, exp_len, ov_pulses, hs_count);
                    fails = fails + 1;
                end else if (error_seen) begin
                    $display("    [BAD]  decoder raised error on a valid block");
                    fails = fails + 1;
                end else begin
                    $display("    [ok]   %0d bytes exact  (valid pulses %0d, accepted %0d)",
                             got_len, ov_pulses, hs_count);
                end
            end
        end
    endtask

    reg [1023:0] verdict;
    integer off;

    initial begin
        fails = 0; cases = 0;
        bp_mode = 0; lfsr = 32'h1; got_len = 0;

        $display("==================================================");
        $display(" A. offset sweep - an offset below the read latency");
        $display("==================================================");
        for (off = 1; off <= 16; off = off + 1) begin
            $display("  case: offset %0d, match 64, no backpressure", off);
            build_run(20, off, 64, 6, 8'h41);
            run_block(stim_len, stim_len - 1, 0, 20000);
            report(0);
        end
        $display("  case: offset 7, prefix 20 (literal extension), match 200");
        build_run(20, 7, 200, 6, 8'h41);
        run_block(stim_len, stim_len - 1, 0, 20000);
        report(0);

        $display("==================================================");
        $display(" B. the same blocks with out_ready on an LFSR");
        $display("==================================================");
        for (off = 1; off <= 8; off = off + 1) begin
            $display("  case: offset %0d, match 64, backpressure", off);
            build_run(12, off, 64, 6, 8'h41);
            run_block(stim_len, stim_len - 1, 1, 20000);
            report(0);
        end
        $display("  case: offset 3, match 200, backpressure");
        build_run(20, 3, 200, 6, 8'h41);
        run_block(stim_len, stim_len - 1, 1, 40000);
        report(0);

        $display("==================================================");
        $display(" C. stream ends on a 0xFF length extension");
        $display("==================================================");
        $display("  case: token, then a 255 extension byte, then nothing");
        stim[0] = 8'hF0; stim[1] = 8'hFF; stim_len = 2;
        run_block(stim_len, stim_len - 1, 0, 3000);
        report(1);

        $display("==================================================");
        $display(" D. offset 0 - invalid in LZ4");
        $display("==================================================");
        $display("  case: offset field 0x0000");
        build_run(12, 0, 32, 6, 8'h41);
        run_block(stim_len, stim_len - 1, 0, 5000);
        report(1);

        $display("==================================================");
        $display(" E. token declares a match, stream ends after literals");
        $display("==================================================");
        $display("  case: token 0xC5 (match nibble 5), 12 literals, no offset");
        stim[0] = {4'd12, 4'h5};
        for (off = 0; off < 12; off = off + 1) stim[1 + off] = 8'h41 + off[7:0];
        stim_len = 13;
        run_block(stim_len, stim_len - 1, 0, 3000);
        report(1);

        $display("==================================================");
        verdict = "PASS";
        if (fails != 0) verdict = "FAIL edge cases";
        $display("  %0d cases, %0d failed", cases, fails);
        $display("  RESULT: %0s", verdict);
        $display("==================================================");
        $finish;
    end

endmodule
