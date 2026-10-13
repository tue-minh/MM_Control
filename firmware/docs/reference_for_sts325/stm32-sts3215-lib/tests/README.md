# Unit Tests

Host-side unit tests for the STS3215 protocol layer (`src/sts3215_protocol.c`).

This layer has **zero STM32 dependency**, e.g., it only touches `sts3215_regs.h`
and the C standard library, so it builds and runs on any desktop machine
with a C99 compiler.

The HAL layer (`src/sts3215_hal.c`) depends on `stm32g4xx_hal.h` and is
therefore out of scope for these tests; it is meant to be exercised on
target, inside an actual STM32CubeIDE project.

## What's covered

`test_protocol.c` checks, for every packet builder (`PING`, `READ`,
`WRITE`/`WRITE2B`/`WRITE_RAW`, the atomic motion command, `REG_WRITE`,
`ACTION`, `SYNC_WRITE`, `SYNC_READ`, the torque/EEPROM-lock helpers, and
`RESET`/`RESET_TURNS`):

- correct header, ID, length, instruction, and checksum bytes
- correct payload byte layout (including little-endian 16-bit fields)
- rejection of `NULL` pointers, undersized buffers, and out-of-range parameters

It also covers `STS3215_ParseReply()` (valid replies, servo-fault replies,
bad header, bad checksum, short frames, `NULL` pointers) and the unit
conversion helpers (steps↔degrees↔radians, raw current/voltage).

## Running locally

```sh
cd tests
make test
```

This builds `test_protocol` with `gcc`/`cc` by default and runs it; a
non-zero exit code means at least one check failed (see stderr for the
failing assertions). Override the compiler with:

```sh
make test CC=clang
```

No third-party test framework is used — assertions are a small header-free
`CHECK()`/`TEST()` macro pair defined directly in `test_protocol.c`.
