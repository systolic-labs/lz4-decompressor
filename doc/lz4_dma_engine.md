# LZ4 stage: engine / DMA path (design note)

**Status: implemented and VERIFIED ON SILICON - `klen` exact, Linux 6.11.11
boots to a shell prompt, LZ4 stage 490 ms measured (the FIFO path: 5 587 ms, a
factor of x11.4).  A packer acceptance bug was found, reproduced in simulation and
fixed (see "Silicon").**
This note exists so that the session implementing it does not have to rediscover
the figures or the architecture, the way the output note did for the FIFO.

> Paths in this note are relative to this repository.  A few files it refers to
> live in the SoC project this block was carved out of and are not part of this
> drop: `dma.v` (the existing copy engine, cited as a model, not a dependency) and
> the boot loader that programmes the engine.  Everything else - the RTL, the
> benches, the resource reports and the console captures - is here.

## What the measurement says

The LZ4 stage on silicon, with the auto-increment drain:

- **5 587 ms** for 11 097 600 bytes = 279 M cycles at 50 MHz = **25.2 cycles per
  output byte**.
- The decoder **alone**, fed continuously and drained with no back-pressure
  (`tb_lz4_engine.v`, three vectors, byte-exact): **1.92-2.00 cycles per byte**
  -> 11 097 600 x 2 / 50 MHz = **0.44 s** for the kernel.
- So **~23 of the 25 cycles/byte are transport**.  The output FIFO brought the
  drain from 4.0 to 2.12 accesses/group (-1.47 s); it cannot go further: the wall
  is the cost per access (~27-30 cycles) multiplied by the number of accesses, and
  what is left is mostly **input polling** (the wait loop reads CTRL every
  iteration) - which the bench's access counter does not see, but which explains
  the gap between the 2 s of counted accesses and the 5.59 s measured.

**The lever is therefore taking the CPU out of the loop, not widening registers.**

## What the spike establishes - and what it does not

Established:

- The decoder holds **2 cycles/byte** with a continuous input stream and an output
  with no back-pressure.  The failure of the "64-bit FIFO version" that "did not
  converge" was therefore **the MMIO interface, not the decoder** - that is the
  point that de-risks the engine path.
- A **real buffered engine** (`lz4_stream.v`: source port with one-cycle
  registered read, 32 B input FIFO, decoder, 32 B output FIFO, write port) holds
  **1.86-2.00 cycles/byte**, byte-exact on all five vectors.  The buffering costs
  nothing: on the real prefix, 603 818 cycles against 603 814 for the ideal feed,
  i.e. **+4 cycles over 313 584 bytes**.  That is the proof that a FIFO on both
  sides does not add the cost that was feared.
- The target: **11.1 MB x 1.92 cycles = ~0.43 s**, a factor of ~13 on the
  5 587 ms.
- The DDR4 budget will not be the limiting factor: 11.1 MB / 8 B per 64-bit beat
  = 1.39 M writes x ~5 cycles = **0.14 s**; the 2.79 MB input -> **0.035 s**.
  The decoder (0.43 s) will stay the ceiling.

Not established:

- That an AXI master inside the LZ4 block reaches that rate **inside the SoC**
  (crossbar, non-cacheable DDR4, arbitration with the CPU) - that is where the
  real risk is, and therefore the first thing to measure.
- The timing margin: the block is already at **WNS -0.023 ns** (the FIFO ate the
  last margin).  An AXI master adds logic on the same clock.

## Proposed architecture

The LZ4 block gains an **AXI4 master port**, modelled on the SoC's existing copy
engine `dma.v` (base `0x0D00_0000`, 64 bits, bursts of 16 beats) and with the same
style of command registers as it:

```
0x20  LSRC   (64-bit)  source address of the compressed bytes (DDR4)
0x28  LDST   (64-bit)  destination address of the decompressed bytes (DDR4)
0x30  LLEN   (32-bit)  compressed length
0x38  LCTRL  (W)       bit0 = start
0x3C  LSTAT  (R)       bit0 busy, bit1 done, bit2 error ; plus the counters
```

(to be housed in the window - today 0x00-0x1C is full; the block would gain a
wider address decode, or a second AXI slave.  To be settled at implementation.)

- The engine **reads** the compressed stream from DDR4 (where the loader has
  already put it, `KERNEL_GZ_ADDR = 0x1_4000_0000`) into an input FIFO, and drives
  `in_valid`.  No more 4-byte MMIO stores, no more CTRL polling.
- It **writes** the output, buffered in an output FIFO, in bursts to
  `KERNEL_ADDR = 0x1_0020_0000`.  No more MMIO drain.
- The CPU only programmes LSRC/LDST/LLEN, pulses LCTRL.start, then waits for
  LSTAT.done - three accesses, against ~3.6 M.

**The MMIO output FIFO becomes dead weight** if the engine is the boot path:
removing it or putting it behind a parameter (`ENABLE_LZ4_MMIO_DRAIN`) frees the
**+486 registers** and ~0.095 ns of margin - that is the point at which the
rollback becomes the right move, once the engine is proven.

## Decisions to make

- **Feed AND drain through the engine, not one of the two.**  The drain alone is
  worth ~1.65 s of the 5.59 s; the input alone is worth its stores plus its
  polling.  Only the pair approaches 0.45 s.
- **An AXI master inside the block, rather than two slave windows + the existing
  DMA.**  The alternative (`dma.v` copies DDR4 -> input window and output window
  -> DDR4, with the "autonomous sequencer" handshake already used for the NPU)
  reuses a proven block and avoids a new master, but requires sequencing the
  copies in chunks with fill/drain thresholds on both sides - more control logic,
  and one more coupling to instrument.  Recommendation: master inside the block,
  `dma.v` as a model and not as a dependency.
- **The CPU no longer touches the stream.**  Do not keep a "small" MMIO input
  loop: it is what dominates the remaining time.

## Risks

- **The AXI master in the crossbar**: one more master (the DMA already is one),
  arbitration, IDs, and the non-cacheable DDR4 path.  That is the real unknown -
  to be measured before anything else.
- **The timing margin is zero.**  Establish early whether the engine holds the
  clock, and plan to remove the MMIO FIFO as the fallback margin.
- **No useful intermediate silicon milestone**: a "half" engine (input only or
  output only) does not deliver the factor, so there is no half-gain to bank -
  except as a bring-up step in simulation.

## Spike plan, with acceptance criteria

1. **Done**: decoder fed/drained continuously, byte-exact, ~2 cycles/byte
   (`tb_lz4_engine.v`).
2. **Done**: throughput and exactness of the existing AXI bench kept as a baseline
   (5 587 ms measured; 2.12 accesses/group).
3. **Done**: buffered engine reading a memory (one-cycle registered read), input
   and output FIFOs, byte-exact on five vectors and **cycles/byte measured**:
   1.86-2.00 (`lz4_stream.v`, `tb_lz4_stream.v`).
4. To do: replace the memory ports with a real AXI4 master against
   `axi4_mig`/the crossbar, in simulation.
5. To do: silicon - LZ4 stage **< 1 s** and `klen` exact, without reflashing the
   kernel.

## What is not established

- That the engine holds timing (the margin is zero today).
- That the non-cacheable DDR4 path accepts the write throughput mixed with the
  input and the rest of the boot.
- The exact cost of the master plus the two FIFOs.  Measured order of magnitude:
  the copy engine `dma.v` alone (complete AXI4 master + registers + store-and-
  forward buffer, **without** the LZ4 input/output FIFOs or the decoder) is
  **692 CLB LUTs, 1601 registers, 0 BRAM** (`doc/report/dma.util.rpt`).  The LZ4
  block is at 713 / 991 / 16: an engine of that size would roughly **double** the
  LUTs and add ~1600 registers - to be offset by removing the MMIO drain
  (-486 registers) and/or by housing the FIFOs in BRAM.  That is the calculation
  to do before choosing between an integrated master and slave windows + `dma.v`.

## Silicon: the bug, and the proof

**Result.**  A fresh bitstream programmed over JTAG, the kernel read from the
unchanged flash:

- `LZ4-SORTIE klen=00a95600` - **exact** (11 097 600);
- `rd = 348547/348547`, `feed = 2788375/2788375`, `wr = 1387200/1387200`,
  `err = 0`;
- `cpu_csum = eng_csum = 52a3acfddf3ef05d` and
  `cpu_hash = eng_hash = e1e3911c` - the engine reads the source and hands it to
  the decoder **in the right order**, exactly like the CPU;
- **`OpenSBI v1.8` then `Linux version 6.11.11` boot, up to the shell prompt
  (`~ #`)**: the decompressed output is correct, not merely the right length.

**The time, decomposed** (the loader times the CPU passes and the engine
separately):

    lz4diag = 1480 ms   the two CPU passes that verify the source (diagnostic)
    lz4eng  =  490 ms   the engine alone
    gunzip  = 1970 ms   the whole phase
    klen    = 00a95600  exact

**490 ms against 5 587 ms for the auto-incremented FIFO, i.e. x11.4** - and that
is measured, not deduced (the 437 ms figure from earlier builds came from builds
whose packer was still dropping bytes).

**The bug, and why it took so long.**  The packer accepted a byte on
`out_valid && out_ready` with

    out_ready = ((onb + out_valid) < 7) || !ow_full

When the accumulator is at 6 bytes and the output FIFO is **full**, the decoder is
allowed to emit (the `< 7` term); the next cycle `out_ready` falls and the byte is
**lost**, because the decoder's `out_valid` is a **single-cycle** pulse.  The proven
formula from the MMIO wrapper **stores on room in the accumulator**
(`out_valid && onb < 8`) and uses `out_ready` only as back-pressure.

The loss rate depends on the **FIFO occupancy**: the simulation, whose drain was
fast, never filled the FIFO and was therefore **exact**; the real DDR4, slower,
filled it and lost - hence a **short and non-deterministic** length.  Reproduced by
modelling a slow B (24 cycles):

    old packer, slow drain: tiny 2316/4096,   prefix 170590/313584   FAIL
    fixed packer, slow drain: tiny 4096/4096, prefix 313584/313584   PASS

**The timing violations (`WNS -1.406 ns` on one build) were a false trail**: it is
a logic bug, present in every engine build from the first.

**The bench now carries that regime** (a slow-B write slave) so the class of bug
cannot become invisible again.

Raw: `doc/report/lz4_engine.console.txt`,
`doc/report/lz4_engine_linux.console.txt`, `doc/report/lz4_engine.util.rpt`.
