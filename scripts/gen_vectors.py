#!/usr/bin/env python3
"""Generate the LZ4 block vectors the testbenches read.

Every bench does `$readmemh("in.hex", ...)` and `$readmemh("exp.hex", ...)`, plus
`` `include "lens.vh" `` for the two lengths.  A vector directory therefore holds:

    in.hex    the compressed LZ4 *block*, one byte per line, two hex digits
    exp.hex   the reference decompression, one byte per line
    lens.vh   `IN_LEN / `EXP_LEN, so the benches can size their arrays

Byte-per-line on purpose: `$readmemh` on a 8-bit-wide array is what the benches
declare, and a wider word would make the truncation rules matter.

The generator needs nothing beyond the standard library.  `--from-file` and
`--from-lz4-stream` produce vectors from your own data and can cross the 64 KiB
window; the four named vectors below are rebuilt deterministically from seeds
and cover the cases the design actually broke on.

    python3 scripts/gen_vectors.py                 # all named vectors
    python3 scripts/gen_vectors.py --list
    python3 scripts/gen_vectors.py --from-file big.bin --name big
    python3 scripts/gen_vectors.py --from-lz4-stream Image.lz4hc --prefix-bytes 200000 --name kernel

Exit status is non-zero if a generated vector fails to decompress with the
reference decoder (when python-lz4 is importable).  That check is the point: a
vector that only the RTL accepts is not a vector, it is a coincidence.
"""

import argparse
import os
import sys

MIN_MATCH = 4
MAX_OFFSET = 0xFFFF
LAST_LITERALS = 5   # LZ4 requires the final literal run to be at least 5 bytes
MFLIMIT = 12        # ... and the last match to start at least 12 bytes from the end

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
VECDIR = os.path.join(ROOT, "vectors")


# --------------------------------------------------------------------------
# A pure-Python LZ4 block compressor.
#
# Greedy, single-candidate, adequate for test data.  It emits a *block*, not a
# frame, exactly like LZ4_compress_default.  The two end-of-block rules above
# are enforced so that a stock decoder accepts what we emit.
# --------------------------------------------------------------------------
def _put_length(out, n):
    """Emit an n-15 style length extension (the caller already wrote the nibble)."""
    while n >= 255:
        out.append(255)
        n -= 255
    out.append(n)


def compress_block(src):
    n = len(src)
    out = bytearray()
    anchor = 0
    table = {}
    i = 0

    while i + MFLIMIT <= n:
        key = src[i:i + 4]
        cand = table.get(key)
        table[key] = i
        match_len = 0
        if cand is not None and 0 < i - cand <= MAX_OFFSET:
            limit = n - LAST_LITERALS          # keep the trailing literals intact
            l = 4
            while l < limit - i and src[cand + l] == src[i + l]:
                l += 1
            if l >= MIN_MATCH:
                match_len = l
        if match_len == 0:
            i += 1
            continue

        lit_len = i - anchor
        token = (min(lit_len, 15) << 4) | min(match_len - MIN_MATCH, 15)
        out.append(token)
        if lit_len >= 15:
            _put_length(out, lit_len - 15)
        out += src[anchor:i]
        out.append((i - cand) & 0xFF)
        out.append(((i - cand) >> 8) & 0xFF)
        if match_len - MIN_MATCH >= 15:
            _put_length(out, match_len - MIN_MATCH - 15)

        i += match_len
        anchor = i

    lit_len = n - anchor
    token = min(lit_len, 15) << 4
    out.append(token)
    if lit_len >= 15:
        _put_length(out, lit_len - 15)
    out += src[anchor:]
    return bytes(out)


# --------------------------------------------------------------------------
# Named payloads.  Synthetic and seeded: no third-party data in the repository.
# --------------------------------------------------------------------------
_WORDS = (b"the quick brown fox jumps over the lazy dog 0123456789 "
          b"lz4 block decompressor history window offset literal match ")


def _text(n, seed):
    """Repetitive text with random insertions: real matches at real distances."""
    import random
    rng = random.Random(seed)
    out = bytearray()
    while len(out) < n:
        take = rng.randint(40, len(_WORDS))
        start = rng.randrange(0, len(_WORDS) - take + 1)
        out += _WORDS[start:start + take]
        out += bytes(rng.getrandbits(8) for _ in range(rng.randint(0, 16)))
    return bytes(out[:n])


def payload_tiny():
    """4 KiB of text.  Fast enough to run every bench on every edit."""
    return _text(4096, 3)


def payload_mixed():
    """96 KiB: text windows separated by runs and incompressible noise.

    Deliberately larger than 64 KiB so matches reach across the wrap of a
    16-bit position counter - the bug that made the offset check reject a valid
    match past the first window.
    """
    import random
    rng = random.Random(11)
    out = bytearray()
    while len(out) < 96 * 1024:
        kind = rng.randrange(4)
        if kind == 0:
            out += _text(4096, rng.randrange(1000))
        elif kind == 1:
            out += bytes([rng.getrandbits(8)]) * rng.randint(300, 1500)   # a run
        elif kind == 2:
            back = rng.randrange(1000, min(len(out), 40000) or 1001)
            if back < len(out):
                out += out[-back:][:rng.randint(500, 2000)]               # far match
        else:
            out += bytes(rng.getrandbits(8) for _ in range(rng.randint(200, 800)))
    return bytes(out[:96 * 1024])


def block_run(prefix, match_len, tail, align=8):
    """A hand-built block whose single match is an offset-1 run.

    This is the shape that broke the first silicon run: the match source is the
    byte being written, so it needs forwarding in the window memory.  The Xilinx
    and reference decoders never produce it from compressible data, so it is
    built by hand.  The final sequence carries >= LAST_LITERALS literals to stay
    inside the block format.

    `align` pads that trailing run so the decompressed length is a whole number
    of 8-byte groups.  The AXI and MMIO output ports hand out groups of eight and
    their benches count in groups, so a vector whose output is not a whole number
    of groups cannot be checked through them - the last group would be padded and
    counted as extra bytes.  Aligning here keeps every bench usable on every
    vector.
    """
    if len(tail) < LAST_LITERALS:
        raise ValueError("tail must be at least %d literals" % LAST_LITERALS)
    if align > 1:
        while (len(prefix) + match_len + len(tail)) % align:
            tail += b"."
    out = bytearray()
    lit_len = len(prefix)
    out.append((min(lit_len, 15) << 4) | 15)
    if lit_len >= 15:
        _put_length(out, lit_len - 15)
    out += prefix
    out.append(0x01)          # offset 1: the run reads the byte just written
    out.append(0x00)
    _put_length(out, match_len - MIN_MATCH - 15)

    out.append(min(len(tail), 15) << 4)
    out += tail
    expected = bytes(prefix) + bytes([prefix[-1]]) * match_len + tail
    return bytes(out), expected


def named_vectors():
    """name -> (block, expected).  Order is the run order of the harness."""
    run4k_block, run4k_exp = block_run(b"the ", 4000, b"end!\n")
    run40k_block, run40k_exp = block_run(b"the ", 40000, b"end!\n")
    tiny_payload = payload_tiny()
    mixed_payload = payload_mixed()
    return {
        # smoke test: smallest, every bench can afford it
        "tiny":    (compress_block(tiny_payload), tiny_payload),
        # the offset-1 run, short: exercises the read-before-write forwarding
        "run4k":   (run4k_block, run4k_exp),
        # the same run long enough to outlive a 16-bit length counter
        "run40k":  (run40k_block, run40k_exp),
        # real compression, output past the 64 KiB window wrap
        "mixed":   (compress_block(mixed_payload), mixed_payload),
    }


# --------------------------------------------------------------------------
# Reading back an LZ4 stream produced elsewhere (the kernel-image vector).
# --------------------------------------------------------------------------
def _rd_length(stream, ip):
    total = 0
    while stream[ip] == 255:
        total += 255
        ip += 1
    return total + stream[ip], ip + 1


def truncate_stream(stream, prefix_bytes):
    """Cut an LZ4 stream at a sequence boundary and close the block.

    Used for the 256 KiB kernel-image vector: the full stream is too slow to
    simulate, but an arbitrary byte cut leaves a partial sequence.  Walk whole
    sequences up to `prefix_bytes`, then append a final literals-only sequence
    built from the bytes the stream would have produced next.
    """
    ip = 0
    produced = 0
    n = len(stream)
    while ip < n and ip < prefix_bytes:
        token = stream[ip]
        ip += 1
        lit_len = token >> 4
        if lit_len == 15:
            extra, ip = _rd_length(stream, ip)
            lit_len += extra
        ip += lit_len
        produced += lit_len
        if ip >= n:
            break
        ip += 2
        match_len = token & 15
        if match_len == 15:
            extra, ip = _rd_length(stream, ip)
            match_len += extra
        produced += match_len + MIN_MATCH
    return stream[:ip], produced


def vector_from_stream(stream, source, prefix_bytes):
    """Close a truncated stream with literals taken from `source`."""
    head, produced = truncate_stream(stream, prefix_bytes)
    tail = source[produced:produced + LAST_LITERALS]
    if len(tail) < LAST_LITERALS:
        raise SystemExit("source is shorter than the truncated stream produces")
    block = head + bytes([len(tail) << 4]) + tail
    return block, source[:produced + len(tail)]


# --------------------------------------------------------------------------
def write_vector(name, block, expected, outdir):
    d = os.path.join(outdir, name)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "in.hex"), "w") as f:
        f.write("\n".join("%02x" % b for b in block))
    with open(os.path.join(d, "exp.hex"), "w") as f:
        f.write("\n".join("%02x" % b for b in expected))
    with open(os.path.join(d, "lens.vh"), "w") as f:
        f.write("`define IN_LEN %d\n`define EXP_LEN %d\n" % (len(block), len(expected)))
    return d


def check_with_reference(name, block, expected):
    """Cross-check against a stock decoder.  Skipped if python-lz4 is absent."""
    try:
        import lz4.block
    except ImportError:
        return None
    try:
        got = lz4.block.decompress(block, uncompressed_size=len(expected))
    except Exception as exc:                       # noqa: BLE001
        return "reference decoder rejected it: %s" % exc
    if got != expected:
        return "reference decoder produced %d bytes, expected %d" % (len(got), len(expected))
    return ""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("names", nargs="*", help="named vectors to build (default: all)")
    ap.add_argument("--list", action="store_true", help="list the named vectors and exit")
    ap.add_argument("--out", default=VECDIR, help="output directory (default: vectors/)")
    ap.add_argument("--from-file", metavar="FILE",
                    help="build one vector from FILE's own bytes (needs python-lz4 to compress)")
    ap.add_argument("--from-lz4-stream", metavar="FILE",
                    help="close a truncated LZ4 stream, e.g. an lz4hc kernel image")
    ap.add_argument("--prefix-bytes", type=int, default=200000,
                    help="how much of --from-lz4-stream to keep (default: 200000)")
    ap.add_argument("--source", metavar="FILE",
                    help="uncompressed bytes for --from-lz4-stream (the tail is taken from it)")
    ap.add_argument("--name", help="name for a one-off vector")
    args = ap.parse_args()

    vecs = named_vectors()
    if args.list:
        for k, (b, e) in vecs.items():
            print("%-8s %7d B -> %7d B  (%.2f:1)" % (k, len(b), len(e), len(e) / len(b)))
        return 0

    todo = {}
    if args.from_file:
        name = args.name or os.path.splitext(os.path.basename(args.from_file))[0]
        src = open(args.from_file, "rb").read()
        try:
            import lz4.block
        except ImportError:
            sys.exit("--from-file needs python-lz4 to compress: pip install lz4")
        block = lz4.block.compress(src, store_size=False)
        todo[name] = (block, src)
    elif args.from_lz4_stream:
        name = args.name or "stream"
        if not args.source:
            sys.exit("--from-lz4-stream needs --source <uncompressed file>")
        stream = open(args.from_lz4_stream, "rb").read()
        src = open(args.source, "rb").read()
        todo[name] = vector_from_stream(stream, src, args.prefix_bytes)
    else:
        for k in (args.names or vecs.keys()):
            if k not in vecs:
                sys.exit("unknown vector %r; --list shows the names" % k)
            todo[k] = vecs[k]

    rc = 0
    for name, (block, expected) in todo.items():
        d = write_vector(name, block, expected, args.out)
        verdict = check_with_reference(name, block, expected)
        if verdict is None:
            note = "reference check skipped (python-lz4 not installed)"
        elif verdict:
            note = "FAIL - " + verdict
            rc = 1
        else:
            note = "reference decoder agrees"
        print("%-8s %7d B -> %7d B  %s" % (name, len(block), len(expected), note))
        print("         %s" % d)
    return rc


if __name__ == "__main__":
    sys.exit(main())
