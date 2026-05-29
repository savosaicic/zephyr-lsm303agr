# Host build of the DT0103 2D calibration self-test.
# Runs natively (no board) to validate the pure-C math core.

CC      ?= cc
CFLAGS  ?= -O2 -std=c11 -Wall -Wextra -Wno-unused-parameter
INCLUDES = -I../../include
SRC      = selftest.c ../../src/cal2d.c

selftest: $(SRC)
	$(CC) $(CFLAGS) $(INCLUDES) $(SRC) -o selftest -lm

run: selftest
	./selftest

clean:
	rm -f selftest

.PHONY: run clean
