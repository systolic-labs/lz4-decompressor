# doc/report - raw evidence

These files are the measurement output behind the numbers quoted in the README
and in the design note.  They are kept verbatim, including the French banners the
capture script adds to the console logs; the tool and boot output inside them is
English.  Nothing here is needed to build or run the design - it is here so the
claims can be checked rather than believed.

## Resource reports

Vivado 2025.2, `xcku5p-ffvb676-1-i`, out-of-context synthesis (`synth_1`),
`report_utilization`.  Each `.util.rpt` names its design and date in its header.

- `lz4_decoder.util.rpt` - the decoder alone: **165 CLB LUTs, 166 registers,
  0 BRAM**.  Backs the headline cost in the README and the comparison with AMD's
  703-LUT HLS kernel.
- `lz4_mmio_prestaging.util.rpt` - the MMIO wrapper before the input staging:
  the baseline the wrapper's cost is quoted against.
- `lz4_mmio_staging.util.rpt` - the MMIO wrapper with input staging:
  **641 LUTs, 505 registers, 16 BRAM**.
- `lz4_mmio_autoinc.util.rpt` - the same wrapper with the auto-increment output
  group FIFO: **713 LUTs, 991 registers, 16 BRAM**, i.e. **+72 LUTs and
  +486 registers** for the auto-increment.
- `lz4_engine.util.rpt` - the whole engine, decoder + stream engine + AXI4
  master + both FIFOs: **1306 LUTs, 2000 registers, 16 BRAM**.
- `dma.util.rpt` - the SoC's copy engine, quoted in the design note as the size
  an integrated AXI4 master would cost: **692 LUTs, 1601 registers, 0 BRAM**.

## Console captures

Raw serial captures from the KU5P board.  Each line is prefixed with a wall-clock
offset and the baud rate the capture was on, because the boot loader talks at
625000 baud and Linux at 115200.

- `lz4_out_fifo_autoincrement.console.txt` - the auto-increment drain on
  silicon: `klen` exact at `00a95600` (11 097 600 bytes), all 1 387 200 output
  groups drained with none lost or duplicated, LZ4 stage **5 587 ms**.
- `lz4_engine.console.txt` - the engine path on silicon: the read/feed/write
  counters matching, the source checksum and ordered hash agreeing with the
  CPU's, `lz4eng = 490 ms`, and the kernel booting.
- `lz4_engine_linux.console.txt` - the same bitstream booting Linux 6.11.11 to a
  shell prompt, which is what proves the decompressed bytes are right and not
  merely the right length.

## What is not here

The design note also refers to the SoC's boot loader and to `dma.v`; those live
in the project this block was carved out of and are out of scope for this drop.
