# lz4_decoder - LZ4 block decompressor in plain Verilog

A byte-serial LZ4 decompressor for FPGAs, written from the published block-format
description.  No code was taken from any unlicensed HDL source: `lib/lz4.c` in the
reference implementation is BSD-2-Clause and AMD's Vitis LZ4 kernels are
Apache-2.0, and both served as functional references only.

It exists because no permissively-licensed LZ4 decompressor in plain Verilog with
a standard window could be found.  A survey of GitHub, OpenCores, GitLab, Gitee
and the academic literature turned up five hardware LZ4 implementations: two are
HLS or bespoke ASIC flows, one is AGPL-3.0, one has no licence at all, and one
has a 128-byte history window so it is not standard LZ4.  So this one was written
and is released under Apache-2.0.

## Cost, measured

Synthesised for `xcku5p-ffvb676-1-i` with Vivado 2025.2:

| Resource | Used | Available |
|---|---|---|
| CLB LUTs | **165** | 216 960 |
| CLB Registers | 166 | 433 920 |
| Block RAM | 0 (window is external) | 480 |
| DSPs | 0 | 1 824 |

For comparison, AMD's published `lz4_decompress` HLS kernel uses 703 LUTs and
591 flip-flops.  The window memory is left to the caller, so the only storage is
the 64 KiB you supply - eight BRAM36 tiles, or a slice of URAM.

## Format

A raw LZ4 *block*, as emitted by `LZ4_compress_default` - not a frame.  A block is
a chain of sequences; each begins with a token byte whose high nibble is the
literal count and low nibble the match length minus 4.  A nibble of 15 means
"add the following bytes, 255 = continue".  Then the literals, then a two-byte
little-endian offset into the history - unless this is the final sequence, which
carries literals only.

There is no terminator, so the caller signals the end by asserting `in_last` with
the final byte of the block.  That byte is latched, because `in_last` is high for
one cycle only.

## Two levels

- **`lz4_decoder.v`** - the decoder itself: byte in, byte out, window external.
- **`lz4_mmio.v`** - the same decoder as an AXI4-Lite peripheral for firmware to
  drive four compressed bytes at a time, carrying its own 64 KiB window.

### Register map of the MMIO wrapper

| offset | access | meaning |
|---|---|---|
| `0x00` | W | bit0: start a fresh block (clears done/error) |
| `0x00` | R | bit0 busy, bit1 error, bit2 done, bit3 an output group is available, bit4 input staging empty |
| `0x04` | W | `[31:0]` four compressed bytes, consumed one at a time |
| `0x08` | R | high half of the head output group; **reading it pops the group** (the auto-increment) |
| `0x0C` | R | bytes the decoder produced |
| `0x10` | R | bytes the decoder consumed from the input staging |
| `0x14` | W | compressed block length (INLEN); `in_last` fires on its last byte |
| `0x14` | R | groups software has consumed (the lost-group instrument) |
| `0x18` | R | low half of the head output group; idempotent, does not pop |
| `0x1C` | R | `[3:0]` whole 8-byte groups ready, `[4]` started, `[5]` the end flag ever reached the decoder |

Firmware writes four compressed bytes per store and polls bit4 before each; the
decoder eats them one at a time.  On the output side the peripheral keeps a FIFO
of frozen 8-byte groups and the CPU drains it with an auto-increment: read `0x1C`
for the number of whole groups ready, then each group as a low half (`0x18`) and
a high half (`0x08`), the high-half read committing the group.  That is one count
read plus two data reads per group, against the old count + two data + consume
write - **2.12 accesses per 8-byte group measured when the FIFO is full, against
4.0**.  The CPU stays out of the decoder's inner loop, which is what removes the
instruction-fetch cost the software decoder paid once per instruction.  The
32-bit note is not stylistic: a 64-bit MMIO load never completes on this
interconnect, and a 64-bit store arrives as two beats whose byte strobes decide
which bytes are live.

The auto-increment is **not idempotent**: a retried or fault-interrupted `0x08`
read advances past the next group, where the old freeze-plus-consume protocol's
repeated read was harmless.  This is stated in the source; `0x14` counts the
groups software has popped, so a lost group is detectable against `0x0C`, and the
boot loader runs with interrupts off and no fault path, which bounds the exposure
for this boot path.

### Cost, measured

`lz4_mmio` on `xcku5p-ffvb676-1-i`, Vivado 2025.2 `synth_1`, with the input
staging and the in-flight output-room fix: **641 CLB LUTs, 505 registers, 16
BRAM36 tiles** - 0.30 % and 3.3 % of the device.

The auto-increment output FIFO costs **713 CLB LUTs, 991 registers, 16 BRAM36
tiles** - **+72 LUTs, +486 registers, no BRAM change** against 641/505/16.  The
486 registers are the 8 x 64-bit group FIFO (one flop per bit; Vivado kept it in
registers rather than distributing it), less the count/snapshot logic it replaced.
The window, and so the BRAM count, is untouched.

The staging cost **+80 LUTs and +57 registers** over the byte-at-a-time wrapper
(561 / 448 measured the same way at `43e79ec`): the 32-bit input staging, the
INLEN counter, and the corrected output room test.  The BRAM count does not
move.  (The 277 LUT / 291 register figure quoted in earlier revisions predates
the staged output; it is not the cost of the current design.)

The BRAM count is not an accident of the tool: it requires the window read to be
**registered**.  A combinational read still passes simulation and is more
permissive than the decoder's own contract, but Vivado cannot infer block RAM for
an asynchronous read and falls back to distributed RAM - measured, that cost
**13 926 LUTs**, fifty times more for the same function.

## Interface

```verilog
lz4_decoder u (
    .clk, .rst_n,
    .in_data, .in_valid, .in_last, .in_ready,
    .out_data, .out_valid, .out_ready,
    .win_waddr, .win_wdata, .win_we, .win_raddr, .win_rdata,
    .done, .error
);
```

The window needs **separate read and write ports**, and the read port needs one
cycle of latency.  `win_raddr` is presented continuously; the byte it addresses
appears on `win_rdata` in the following cycle.  A byte is written whenever
`out_valid` is high.

## Throughput

One byte per clock while copying literals, two per matched byte: one cycle to
present the read address and one to consume it.  At 100 MHz that is roughly
50 MB/s on typical LZ4 data - it is a bring-up decompressor, not a
high-throughput one, and closing the match phase to one byte per clock is a
pipeline change that has deliberately not been attempted before the simple
version was proven.

## Verification

Self-checking, against a real stream from the reference compressor:

```sh
# decoder alone
iverilog -g2012 -o tb_lz4 tb_lz4_decoder.v lz4_decoder.v && vvp tb_lz4
# through the AXI peripheral, the way firmware drives it
iverilog -g2012 -o tb_mmio tb_lz4_mmio.v lz4_mmio.v lz4_decoder.v && vvp tb_mmio
```

Validated on two vectors: a synthetic 64 KiB payload, and 256 KiB of a real Linux
kernel image compressed at 2.07:1.  The second matters - it contains a single
98 576-byte match with offset 1, a run whose source is the byte being written, and
it exposed four bugs the easy vector could not:

- a single-port window memory cannot read and write in the same cycle, which made
  the first match byte correct and every one after it undefined;
- the offset validity check compared against `opos[15:0]`, which wraps every
  64 KiB, so past the first window a valid offset was rejected as past the start;
- sequence lengths must be 32-bit: LZ4 bounds the *distance* to 64 KiB but places
  no bound on a sequence's *length*;
- a read-before-write BRAM returns the old contents when read and write hit the
  same address in one cycle, so an offset-1 run needs forwarding - latched, since
  the read is issued a cycle before its data is used.

Each of those is commented in the source with its reason.

**On silicon (KU5P).**  The kernel's `klen` is exact (`00a95600`), all 1 387 200
output groups are drained with none lost or duplicated (`groupes` = produced/8),
and the LZ4 stage is **5 587 ms against 7 060 ms** for the previous drain.  Raw
console: `doc/report/lz4_out_fifo_autoincrement.console.txt`.

## Licence

Apache-2.0.  See `LICENSE`.
