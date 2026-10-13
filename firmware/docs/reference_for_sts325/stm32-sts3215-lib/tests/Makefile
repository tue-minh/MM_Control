# Host-side unit test build for the STS3215 protocol layer.
#
# Usage:
#   make            # build ./test_protocol
#   make test       # build and run
#   make clean
#
# The protocol layer (sts3215_protocol.c) has zero STM32 dependency,
# so it builds and runs with any C99 compiler on the host machine.

CC      ?= cc
CFLAGS  ?= -std=c99 -Wall -Wextra -Werror -O2 -g
LDFLAGS ?= -lm

INC_DIR := ../inc
SRC_DIR := ../src

SRCS := $(SRC_DIR)/sts3215_protocol.c test_protocol.c
BIN  := test_protocol

.PHONY: all test clean

all: $(BIN)

$(BIN): $(SRCS) $(INC_DIR)/sts3215_protocol.h $(INC_DIR)/sts3215_regs.h
	$(CC) $(CFLAGS) -I$(INC_DIR) -o $(BIN) $(SRCS) $(LDFLAGS)

test: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)
