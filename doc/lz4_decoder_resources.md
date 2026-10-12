# lz4_decoder - resources and verification

Bench report for `rtl/lz4_decoder.v` (Apache-2.0).

## Synthesis

Vivado 2025.2, part `xcku5p-ffvb676-1-i`, out-of-context synthesis
(`launch_runs synth_1`), raw report in `lz4_decoder.util.rpt`.

| Resource | Used | Available | % |
|---|---:|---:|---:|
| CLB LUTs | **165** | 216 960 | 0.08 |
| CLB Registers | 166 | 433 920 | 0.04 |
| Block RAM Tile | 0 | 480 | 0.00 |
| DSPs | 0 | 1 824 | 0.00 |
| Bonded IOB | 74 | 280 | 26.43 |

The window memory is deliberately external, so the only storage is the 64 KiB the
caller supplies - eight BRAM36 tiles, or a slice of URAM.  The IOB count is an
artefact of synthesising the module standalone; in a real integration those ports
are internal.

For comparison, AMD's published `lz4_decompress` HLS kernel on XCU200 uses 703
LUTs and 591 flip-flops and runs at 375 MHz.

## Verification

Self-checking testbenches against real streams produced by the reference
compressor (`lib/lz4.c`, BSD-2-Clause, used as a reference only - no code copied):

```sh
make test                       # every bench against every vector
scripts/run_tests.sh tb_lz4_decoder run40k   # one bench, one vector
```

A bench passes when it prints `RESULT: PASS`; a wrong length, a wrong byte and an
internal deadline all produce a distinct failure line.

The four vectors under `vectors/` are committed, so a fresh clone runs without
network or extra tooling:

| vector | what it exercises | compressed | output | ratio |
|---|---|---:|---:|---:|
| `tiny` | smoke test, real matches at real distances | 723 B | 4 096 B | 5.67:1 |
| `run4k` | an offset-1 run: the match source is the byte being written | 36 B | 4 016 B | 111.6:1 |
| `run40k` | the same run past a 16-bit length counter | 177 B | 40 016 B | 226.1:1 |
| `mixed` | text, runs and noise, output past the 64 KiB window wrap | 17 024 B | 98 304 B | 5.77:1 |

`scripts/gen_vectors.py` rebuilds them from seeds (standard library only) and can
build new ones from your own data, including the kernel-image case below.  Every
shipped vector is also accepted by a stock LZ4 decoder - the generator checks that
and refuses to write one that is not, so a failure in the bench is the design and
not the input.

During development two vectors did the real work, and they are the ones that found
the four bugs listed below:

| vector | compressed | output | result |
|---|---:|---:|---|
| synthetic payload | 7 542 B (9.01:1) | 65 536 B | PASS |
| 256 KiB of a real kernel Image | 126 749 B (2.07:1) | 262 144 B | PASS |

The kernel vector is not shipped: its 262 144 output bytes are Linux kernel code,
which has no business travelling with an Apache-2.0 RTL drop.  It is reproducible
from any kernel `Image` with `scripts/gen_vectors.py --from-lz4-stream` (see the
README), which is the part that matters - the technique, not the bytes.

The second vector was necessary: it contains a single **98 576-byte match with
offset 1**, and it exposed four bugs the easy vector could not reach:

1. **A single-port window cannot read and write in one cycle** - the first match
   byte decoded and the rest came back undefined, so the ports are separate.
2. **The offset validity check compared against `opos[15:0]`**, which wraps every
   64 KiB, so past the first window a valid offset looked like it reached before
   the start of the output.  The decoder stopped at exactly 65 542 bytes.
3. **Sequence lengths must be 32-bit.**  LZ4 bounds the *distance* to 64 KiB but
   places no bound on a sequence's *length*; the kernel's stream has a 98 576-byte
   match, and 16-bit counters overflowed silently.
4. **A read-before-write memory returns the old contents** when read and write hit
   the same address in one cycle.  An offset-1 run copies the byte just written,
   so it needs forwarding - latched, because the read is issued a cycle before its
   data is used.

## Throughput

One byte per clock for literals, two per matched byte (one cycle to present the
read address, one to consume it, because a synchronous memory returns data the
cycle after its address is registered).  At 100 MHz that is roughly 50 MB/s - 
**~0.22 s for the 11 MB kernel against the 12.5 s the software inflate costs.**

Closing the match phase to one byte per clock is a pipeline change and is
deliberately not attempted until the straightforward version had been proven.
