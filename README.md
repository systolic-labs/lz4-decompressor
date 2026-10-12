# lz4_decoder - LZ4 block decompressor in plain Verilog

A byte-serial LZ4 decompressor for FPGAs, written from the published block-format
description.  No code was taken from any unlicensed HDL source: `lib/lz4.c` in the
reference implementation is BSD-2-Clause and AMD's Vitis LZ4 kernels are
Apache-2.0, and both served as functional references only (see `NOTICE`).

It exists because no permissively-licensed LZ4 decompressor in plain Verilog with
a standard window could be found.  A survey of GitHub, OpenCores, GitLab, Gitee
and the academic literature turned up five hardware LZ4 implementations: two are
HLS or bespoke ASIC flows, one is AGPL-3.0, one has no licence at all, and one
has a 128-byte history window so it is not standard LZ4.  So this one was written
and is released under Apache-2.0.  The survey is published in full, with the
licence of each entry read from its own licence file, in
`doc/lz4_hardware_survey.md`.

## Getting started

Three commands and no arguments:

```sh
git clone <this repository> && cd lz4-decoder
make test          # every bench against every vector, ~40 s
make help          # the options, and the tool variables
```

Icarus Verilog is the only dependency for the tests, and nothing but Python 3 for
the vector generator.  There is no build system, no IP catalogue and no vendor
tool in the flow: the benches are plain Verilog and the vectors are committed, so
a fresh clone runs offline.

```
build/             # created by the harness, ignored by git
rtl/               # the design
tb/                # six self-checking benches
vectors/           # in.hex + exp.hex + lens.vh per vector
scripts/           # gen_vectors.py, run_tests.sh
doc/               # design note, survey, resource reports
doc/report/        # raw synthesis reports and serial captures
```

## Test vectors

Every bench reads its data the same way, and this is the whole contract:

```verilog
`include "lens.vh"                          // `IN_LEN and `EXP_LEN
$readmemh("in.hex",  input_bytes);          // the LZ4 block, one byte per line
$readmemh("exp.hex", expect_bytes);         // the reference decompression
```

A vector is therefore a directory holding those three files, and the harness
copies one into a scratch directory per run, because `$readmemh` resolves its
argument against the *current* directory and not against the source file.

Four vectors are committed, chosen for what they break rather than for size:

| vector | what it exercises | compressed | output | ratio |
|---|---|---:|---:|---:|
| `tiny` | smoke test: text, so real matches at real distances | 723 B | 4 096 B | 5.67:1 |
| `run4k` | an offset-1 run - the match source is the byte being written | 36 B | 4 016 B | 111.6:1 |
| `run40k` | the same run, long enough to outlive a 16-bit counter | 177 B | 40 016 B | 226.1:1 |
| `mixed` | text, runs and noise, output past the 64 KiB window wrap | 17 024 B | 98 304 B | 5.77:1 |

`scripts/gen_vectors.py` rebuilds them from seeds, with the standard library
only:

```sh
python3 scripts/gen_vectors.py                  # all four
python3 scripts/gen_vectors.py --list           # what each one is
python3 scripts/gen_vectors.py --from-file my.bin --name my   # your own data
```

and it can cut a large stream down to a simulatable block, which is how the
kernel-image vector below was built:

```sh
# an lz4hc stream, cut at a sequence boundary and closed with a literal run
python3 scripts/gen_vectors.py --from-lz4-stream Image.lz4hc --source Image \
        --prefix-bytes 200000 --name kernel
```

If `python-lz4` is importable the generator decompresses everything it writes
with a stock decoder and refuses to write a vector that one rejects.  That check
is why the vectors end on a full literal run of at least five bytes: the RTL
decoder is permissive and would happily accept a shorter one, but a vector only
this decoder can read cannot tell you whether a failure is the design or the
input.

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
the 64 KiB you supply - eight BRAM36 tiles, or a slice of URAM.  The raw report is
`doc/report/lz4_decoder.util.rpt`.

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

## Three levels

- **`lz4_decoder.v`** - the decoder itself: byte in, byte out, window external.
- **`lz4_mmio.v`** - **legacy**, kept because its AXI-level bench still pins the
  decoder: the decoder behind an AXI4-Lite register file, firmware feeding it
  four compressed bytes at a time.  **It is no longer instantiated in the SoC**
  (it reached 5 587 ms on the kernel) and its register map below is history.
- **`lz4_engine.v`** - **what the SoC instantiates**: the decoder behind a
  stream engine with its own AXI4 master (`lz4_axi.v`).  The CPU programmes
  source, destination and length, pulses start, and polls done; the engine reads
  the compressed block from DDR4 and writes the decompressed block back, so the
  CPU never touches a byte of the stream.  **490 ms measured on the kernel**
  against 5 587 ms for the MMIO drain.  See `doc/lz4_dma_engine.md`.

### Register map of the engine (the one the SoC uses)

| offset | access | meaning |
|---|---|---|
| `0x00` | W | bit0: start (self-clearing) |
| `0x00` | R | bit0 busy, bit1 done, bit2 error |
| `0x04` | W/R | source address `[31:0]` (compressed block, in DDR4) |
| `0x08` | W/R | source address `[63:32]` |
| `0x0C` | W/R | destination address `[31:0]` (decompressed block) |
| `0x10` | W/R | destination address `[63:32]` |
| `0x14` | W/R | compressed length in bytes |
| `0x18` | R | decompressed bytes produced |
| `0x1C` | W | debug selector |
| `0x1C` | R | the selected counter: 0 read words, 1 written words, 2 bytes fed to the decoder, 3 non-OKAY responses, 4/5 the source XOR lo/hi, 6 the order-sensitive hash of the bytes handed to the decoder |

The `0x1C` counters are not decoration: the engine's failure on silicon (a
short, non-deterministic output) was localised with them - the source XOR and
the ordered hash proved the reads and the feed were exact, which pointed at the
packer, whose acceptance condition was dropping bytes whenever the output word
FIFO filled (`doc/lz4_dma_engine.md`, "Silicon").

### Register map of the MMIO wrapper (legacy)

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
(561 / 448, measured the same way before the staging landed): the 32-bit input
staging, the INLEN counter, and the corrected output room test.  The BRAM count
does not move.  (The 277 LUT / 291 register figure quoted in earlier revisions
predates the staged output; it is not the cost of the current design.)

The BRAM count is not an accident of the tool: it requires the window read to be
**registered**.  A combinational read still passes simulation and is more
permissive than the decoder's own contract, but Vivado cannot infer block RAM for
an asynchronous read and falls back to distributed RAM - measured, that cost
**13 926 LUTs**, fifty times more for the same function.

Each of those figures is in `doc/report/` as a raw Vivado report.

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

Self-checking, against streams from the reference compressor:

```sh
make test                                    # all six benches, all four vectors
scripts/run_tests.sh tb_lz4_decoder run40k   # one bench, one vector
```

| bench | what it drives |
|---|---|
| `tb_lz4_decoder` | the decoder alone, byte in / byte out |
| `tb_lz4_stream` | the buffered stream engine, and its cycles/byte |
| `tb_lz4_axi` | the AXI4 master path against a modelled slave |
| `tb_lz4_engine` | the engine's read/feed/write path, with a slow write slave |
| `tb_lz4_engine_reg` | the engine through its register map |
| `tb_lz4_mmio` | the legacy AXI4-Lite wrapper and its access count |

Every bench checks the output byte for byte against `exp.hex` and prints
`RESULT: PASS`, `RESULT: FAIL <reason>` or `TIMEOUT`; the harness turns that into
an exit status.  The group-granular ports (AXI and MMIO) hand out eight bytes at a
time, so the vectors' outputs are whole numbers of groups - the generator aligns
them, which is what lets every bench run against every vector.

Two vectors did the real work during development: a synthetic 64 KiB payload, and
256 KiB of a real Linux kernel image compressed at 2.07:1.  The second matters -
it contains a single 98 576-byte match with offset 1, a run whose source is the
byte being written, and it exposed four bugs the easy vector could not:

- a single-port window memory cannot read and write in the same cycle, which made
  the first match byte correct and every one after it undefined;
- the offset validity check compared against `opos[15:0]`, which wraps every
  64 KiB, so past the first window a valid offset was rejected as past the start;
- sequence lengths must be 32-bit: LZ4 bounds the *distance* to 64 KiB but places
  no bound on a sequence's *length*;
- a read-before-write BRAM returns the old contents when read and write hit the
  same address in one cycle, so an offset-1 run needs forwarding - latched, since
  the read is issued a cycle before its data is used.

Each of those is commented in the source with its reason.  The kernel vector
itself is not shipped (`doc/lz4_decoder_resources.md` explains why, and how to
rebuild one from any kernel `Image`); `run4k` and `run40k` are the shipped
stand-ins for the offset-1 run, and `mixed` covers the window wrap.

**On silicon (KU5P).**  The kernel's `klen` is exact (`00a95600`), all 1 387 200
output groups are drained with none lost or duplicated (`groupes` = produced/8),
and the LZ4 stage is **5 587 ms against 7 060 ms** for the previous drain.  Raw
console: `doc/report/lz4_out_fifo_autoincrement.console.txt`.  The engine path
that replaced it measured **490 ms** on the same bitstream, with the source
checksum and ordered hash agreeing with the CPU's and Linux booting to a shell
prompt: `doc/report/lz4_engine.console.txt`,
`doc/report/lz4_engine_linux.console.txt`.

## Licence

Apache-2.0.  See `LICENSE`.

`NOTICE` records the two upstream works this design was written against - the LZ4
reference implementation (BSD-2-Clause) and AMD's Vitis LZ4 kernels
(Apache-2.0) - and states what was and was not taken from them.  No code from
either is present in this repository.
