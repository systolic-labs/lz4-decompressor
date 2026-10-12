#!/usr/bin/env bash
#
# Build and run the LZ4 testbenches under Icarus Verilog.
#
#   scripts/run_tests.sh                    # every bench against every vector
#   scripts/run_tests.sh tb_lz4_axi         # one bench, every vector
#   scripts/run_tests.sh tb_lz4_axi mixed   # one bench, one vector
#   RUN_TIMEOUT=1800 scripts/run_tests.sh   # raise the per-run wall-clock cap
#
# Each bench is compiled with every file in rtl/ - the modules are independent,
# so the unused ones cost nothing but a name in the compile line and no bench can
# silently miss a dependency.  The run happens in its own directory holding that
# vector's in.hex / exp.hex / lens.vh, because the benches read them by relative
# name ($readmemh("in.hex", ...)) and `include them at compile time.
#
# A bench passes when it prints "RESULT: PASS"; it prints "RESULT: FAIL ..." for a
# wrong length or a wrong byte, and "TIMEOUT" when the simulated deadline passes
# inside the bench.  Anything else - including a non-zero exit from vvp - fails.
#
# Exit status is the number of failed runs, capped at 125.

set -u -o pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
RTL=$ROOT/rtl
TB=$ROOT/tb
VECDIR=$ROOT/vectors
WORK=${WORK:-$ROOT/build/sim}

IV=${IV:-iverilog}
VVP=${VVP:-vvp}
RUN_TIMEOUT=${RUN_TIMEOUT:-900}

ALL_BENCHES=(tb_lz4_decoder tb_lz4_mmio tb_lz4_stream tb_lz4_axi tb_lz4_engine tb_lz4_engine_reg)
ALL_VECTORS=(tiny run4k run40k mixed)

usage() {
    sed -n '2,26p' "$0" | sed 's/^# \{0,1\}//'
    printf '\nbenches: %s\nvectors: %s\n' "${ALL_BENCHES[*]}" "${ALL_VECTORS[*]}"
}

contains() { local n=$1; shift; local x; for x in "$@"; do [ "$x" = "$n" ] && return 0; done; return 1; }

benches=("${ALL_BENCHES[@]}")
vectors=("${ALL_VECTORS[@]}")

while [ $# -gt 0 ]; do
    case $1 in
        -h|--help) usage; exit 0 ;;
        *) if contains "$1" "${ALL_BENCHES[@]}"; then
               benches=("$1")
           elif contains "$1" "${ALL_VECTORS[@]}"; then
               vectors=("$1")
           else
               printf 'unknown argument: %s (see --help)\n' "$1" >&2; exit 2
           fi ;;
    esac
    shift
done

command -v "$IV"  >/dev/null || { echo "run_tests: $IV not found"  >&2; exit 2; }
command -v "$VVP" >/dev/null || { echo "run_tests: $VVP not found" >&2; exit 2; }

pass=0; fail=0; failed=()

printf '=== LZ4 testbenches: %d bench(es) x %d vector(s), timeout %ss ===\n\n' \
       "${#benches[@]}" "${#vectors[@]}" "$RUN_TIMEOUT"

for bench in "${benches[@]}"; do
    [ -f "$TB/$bench.v" ] || { printf 'missing bench: %s\n' "$TB/$bench.v" >&2; exit 2; }
    for vec in "${vectors[@]}"; do
        vdir=$VECDIR/$vec
        if [ ! -f "$vdir/in.hex" ] || [ ! -f "$vdir/exp.hex" ] || [ ! -f "$vdir/lens.vh" ]; then
            printf 'FAIL  %-18s %-7s  vector incomplete (need in.hex, exp.hex, lens.vh)\n' "$bench" "$vec"
            printf '      regenerate with: python3 scripts/gen_vectors.py %s\n' "$vec"
            fail=$((fail + 1)); failed+=("$bench/$vec")
            continue
        fi

        run=$WORK/$bench.$vec
        rm -rf "$run"; mkdir -p "$run"
        cp "$vdir"/in.hex "$vdir"/exp.hex "$vdir"/lens.vh "$run"/

        start=$SECONDS
        if ! (cd "$run" && "$IV" -g2012 -I. -o sim \
                "$TB/$bench.v" "$RTL"/*.v) >"$run/compile.log" 2>&1; then
            printf 'FAIL  %-18s %-7s  compile error\n' "$bench" "$vec"
            sed 's/^/      /' "$run/compile.log" | head -8
            fail=$((fail + 1)); failed+=("$bench/$vec (compile)")
            continue
        fi

        rc=0
        (cd "$run" && timeout "$RUN_TIMEOUT" "$VVP" sim) >"$run/run.log" 2>&1 || rc=$?
        elapsed=$((SECONDS - start))

        if grep -q 'RESULT: PASS' "$run/run.log"; then
            printf 'PASS  %-18s %-7s  %3ss\n' "$bench" "$vec" "$elapsed"
            pass=$((pass + 1))
        else
            verdict=$(grep -aoE 'RESULT: [A-Za-z-]+|TIMEOUT[^"]*' "$run/run.log" | tail -1)
            [ -n "$verdict" ] || verdict="no verdict, vvp exit $rc"
            printf 'FAIL  %-18s %-7s  %3ss  %s\n' "$bench" "$vec" "$elapsed" "$verdict"
            printf '      log: %s\n' "$run/run.log"
            fail=$((fail + 1)); failed+=("$bench/$vec")
        fi
    done
done

printf '\n=== %d passed, %d failed ===\n' "$pass" "$fail"
if [ "$fail" -gt 0 ]; then
    printf 'failed: %s\n' "${failed[*]}"
    [ "$fail" -gt 125 ] && exit 125
    exit "$fail"
fi
