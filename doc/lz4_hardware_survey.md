# Open-source LZ4 hardware: survey

Archived survey, September 2026.  Question: is there a permissively-licensed,
plain-RTL LZ4 decompressor to reuse, or must one be written?

Sources searched: GitHub repo and code search (including
`lz4 language:Verilog` / `VHDL` / `SystemVerilog` / `Chisel` and `lz4-hls`,
`lz4 fpga`, `lz4 ultrascale`), OpenCores API, GitLab and Gitee APIs, arXiv,
Crossref, Zenodo, grep.app, and vendor repositories.  Every licence below was read
from the licence file or the file header, not from metadata.

## The five hardware implementations that exist

| project | implements | licence | flow | usable? |
|---|---|---|---|---|
| **AMD/Xilinx Vitis Data Compression** (`Xilinx/Vitis_Libraries`, `data_compression/`, tags v2022.1-v2024.1 - removed from `main`) | compress + decompress | **Apache-2.0** | HLS C++ / Vitis | Not as RTL; good functional reference. L1 `lz4_decompress`: 703 LUT, 591 FF, 32 BRAM, 375 MHz on XCU200 (UltraScale+) |
| `FarisHijazi/LZ4-Decompressor-Verilog` | decompress, full 64 KiB window | **none - all rights reserved** | plain Verilog | **No.** No licence file, no header, GitHub `/license` 404 |
| `NiharGowdaS/lz4_compression_caravel` (`yonga_lz4_decoder`) | decompress | **Apache-2.0** | plain Verilog | **No.** History window is 128 bytes (7-bit addresses), so it is not standard LZ4 and fails on real data |
| `weslay-koo/LZ4_huffman_hardware` | compress (+ Huffman) | **AGPL-3.0** | Verilog, ISE | **No.** Network copyleft, and it is an encoder |
| Two Vitis-HLS student contest repos | compress (derived from AMD above) | Apache-2.0 | HLS | Not standalone IP |

## Categories verified empty

- GitHub `lz4 language:VHDL` - **0**
- GitHub `lz4 language:SystemVerilog` - **0**
- GitHub `lz4 language:Chisel` - **0 real hits**
- OpenCores - **0 LZ4 projects** (adjacent only: `lzrw1-compressor-core`, `openlzs`, `deflatecore`, `huffmandecoder`)
- GitLab public search - 0 HDL (all software forks); Gitee - 0; Zenodo - no RTL
- `lz4 ultrascale` / `lz4 xcku` / `lz4 kintex` / `lz4 fpga ip core` - **0**

## Adjacent, not LZ4

- `ChenJianyunp/FPGA-Snappy-Decompressor` - Apache-2.0, Verilog, but needs the IBM
  CAPI/SNAP shell on POWER9; not drop-in.
- `opencomputeproject/Project-Zipline` - MIT, but Deflate-family, not LZ4.
- 15 LZ77 Verilog repositories - **all without any licence file**, and none
  implements the LZ4 format.

## Published designs with no released code

Eleven papers with resource figures but no repository (Bartik et al. ICECS 2015;
Lee et al. IEICE 2017; Liu et al. IEEE TCE 2018; Benes et al. ReConFig 2019 on
**XCKU040** 14 076 slice-LUT / 6.08 Gbps; HybriDC 2022 on Arria 10; Chen et al.
arXiv:2409.12433 on XC7K325T 10 863 slice-LUT / 16.10 Gbps; and others).  Reported
as **no code found**, not as no code existing.

## Conclusion

**No permissively-licensed, plain-Verilog, standard-window LZ4 decompressor exists
in public repositories as of September 2026.**  The closest in structure has no
licence at all, so it cannot even be a starting point for copying.

Hence `rtl/lz4_decoder.v`, written from the published block-format description and
released under Apache-2.0.
