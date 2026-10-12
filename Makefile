# LZ4 block decompressor - build and run the testbenches.
#
# The benches are plain Verilog and need nothing but Icarus Verilog; the vectors
# under vectors/ are committed, so `make test` works on a fresh clone with no
# arguments and no network.  `make vectors` rebuilds them from
# scripts/gen_vectors.py, which needs only the standard library.

PYTHON   ?= python3
IV       ?= iverilog
VVP      ?= vvp

# Per-run wall-clock cap for the harness.  The shipped vectors are small; raise
# this if you add a big one.
RUN_TIMEOUT ?= 900

.PHONY: all test vectors bench clean help

all: test

## Run every bench against every vector (the default).
test:
	@IV=$(IV) VVP=$(VVP) RUN_TIMEOUT=$(RUN_TIMEOUT) scripts/run_tests.sh

## Rebuild the vectors under vectors/.
vectors:
	$(PYTHON) scripts/gen_vectors.py

## List the named vectors and what they exercise.
bench:
	$(PYTHON) scripts/gen_vectors.py --list

## Remove the simulation work directory.
clean:
	rm -rf build

help:
	@sed -n 's/^## \{0,1\}//p' $(MAKEFILE_LIST)
	@printf '\nvariables: PYTHON=%s IV=%s VVP=%s RUN_TIMEOUT=%s\n' \
	         '$(PYTHON)' '$(IV)' '$(VVP)' '$(RUN_TIMEOUT)'
