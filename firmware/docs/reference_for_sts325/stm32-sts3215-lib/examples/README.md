# Examples

Focused, single-purpose usage snippets for the STS3215 driver, each built
around one feature from the [Features](../README.md#features) list. They
complement the [Minimal Example](../README.md#5--minimal-example) in the
main README, which combines `PING` and a single-servo position toggle.

| Example | Demonstrates |
| ------- | ------------ |
| [`01_ping/`](./01_ping/main.c) | `PING`, checking a servo is alive on the bus |
| [`02_read_feedback/`](./02_read_feedback/main.c) | A single `READ` spanning the whole feedback block (position, speed, load, voltage, temperature) + unit conversion helpers |
| [`03_sync_write_multi_servo/`](./03_sync_write_multi_servo/main.c) | `SYNC_WRITE`, frame-accurate simultaneous motion across several servos |
| [`04_eeprom_config/`](./04_eeprom_config/main.c) | `UnlockEEPROM` / `LockEEPROM`, persisting a new servo ID to EEPROM |

## How to use

These codes aren't standalone projects — they follow the same convention as
the [main README's minimal example:](../README.md#5--minimal-example) each `main.c` is a full CubeMX-style
skeleton with `/* USER CODE BEGIN ... END */` markers, meant to be copied
into the corresponding sections of your own STM32CubeIDE-generated
`Core/Src/main.c`.

1. Follow [Usage & Integration](../README.md#usage--integration) in the
   main README first (`.ioc` configuration, file placement, includes).
2. Open the example's `main.c` and copy each `USER CODE` block into the
   matching section of your project's `main.c`.
3. Adjust servo IDs, `huart2`, and register values for your setup.

> [!WARNING]
> All examples assume a single USART named `huart2` in RS485 mode, exactly
> as configured in [section 2](../README.md#2--stm32cubeide-ioc-configuration)
> of the main README. Rename it if you used a different USART instance.
