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
  drive byte at a time, carrying its own 64 KiB window.

### Register map of the MMIO wrapper

| offset | access | meaning |
|---|---|---|
| `0x00` | W | bit0: start a fresh block (clears done/error) |
| `0x00` | R | bit0 busy, bit1 error, bit2 done, bit3 output byte available |
| `0x04` | W | `[7:0]` compressed byte, `[8]` = last byte of the block |
| `0x08` | R | `[7:0]` decompressed byte; reading releases the decoder |
| `0x0C` | R | `[15:0]` decompressed bytes produced |

Firmware writes a byte, polls bit3, reads the byte, repeats - the CPU stays in
the loop between bytes but out of the inner loop, which is what removes the
instruction-fetch cost the software decoder paid once per instruction.

### Cost, measured

`lz4_mmio` on `xcku5p-ffvb676-1-i`: **277 CLB LUTs, 291 registers, 16 BRAM36
tiles** - 0.13 % and 3.3 % of the device.

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

## Licence

Apache-2.0.  See `LICENSE`.
