# STS3215 Memory Table Parameters - V 3.6

## STS3215 Technical Specifications

| Parameter | Value | Parameter | Value |
| :--- | :--- | :--- | :--- |
| **No load speed (RPM)** | 50 | **Test voltage (V)** | 7.4V |
| **No load speed (step/s)** | 3400 | **No load current (mA)** | 150 |
| **Maximum Effective Angle** | 360° | **Multi-turn Support** | Yes (Note: Power failure does not save cycle count) |
| **Resolution (step)** | 4096 | **Electronic dead zone** | 0.17578° |
| **Min. Resolution Angle** | 0.087890625° | **Acceleration (deg/s²)** | 8.7890625 |

## Memory Table

| Address (HEX) | Address (DEC) | Function | Bytes | Initial Value | Storage Area | Authority | Minimum Value | Maximum Value | Unit | Analysis of Values |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `0x0` | `0` | Firmware major version number | 1 | 3 | EPROM | read | -1 | -1 | - | - |
| `0x1` | `1` | Firmware sub version number | 1 | 6 | EPROM | read | -1 | -1 | - | - |
| `0x3` | `3` | servo Main Version Number | 1 | 9 | EPROM | read | -1 | -1 | - | - |
| `0x4` | `4` | servo sub version number | 1 | 3 | EPROM | read | -1 | -1 | - | - |
| `0x5` | `5` | ID | 1 | 1 | EPROM | read&write | 0 | 253 | number | Unique identification code on the bus. Duplicate ID number is not allowed on the same bus, 254 (0xFE) is the broadcast ID, broadcast does not return a reply packet. |
| `0x6` | `6` | Baud rate | 1 | 0 | EPROM | read&write | 0 | 7 | no | 0-7 represents baud rate as follows: 1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400 |
| `0x7` | `7` | Return delay | 1 | 0 | EPROM | read&write | 0 | 254 | 2us | The minimum unit is 2us, and the maximum set return delay is 254 * 2 = 508us |
| `0x8` | `8` | Response status level | 1 | 1 | EPROM | read&write | 0 | 1 | no | 0: except for read instruction and Ping instruction, other instructions do not return reply packet; 1: Returns a reply packet for all instructions. |
| `0x9` | `9` | Minimum Angle Limitation | 2 | 0 | EPROM | read&write | 0 | 4094 | step | Set the minimum limit of motion stroke, the value is less than the maximum angle limit, and this value is 0 when the multi cycle absolute position control is carried out |
| `0xB` | `11` | Maximum Angle Limitation | 2 | 4095 | EPROM | read&write | 1 | 4095 | step | Set the maximum limit of motion stroke, which is greater than the minimum angle limit, and the value is 0 when the multi turn absolute position control is adopted. |
| `0xD` | `13` | Maximum Temperature Limit | 1 | 70 | EPROM | read&write | 0 | 100 | °C | The maximum operating temperature limit, if set to 70, the maximum temperature is 70 ℃, and the setting accuracy is 1 ℃ |
| `0xE` | `14` | Maximum input voltage | 1 | 80 | EPROM | read&write | 0 | 254 | 0.1V | If the maximum input voltage is set to 80, the maximum working voltage is limited to 8.0V and the setting accuracy is 0.1V |
| `0xF` | `15` | Minimum input voltage | 1 | 40 | EPROM | read&write | 0 | 254 | 0.1V | If the minimum input voltage is set to 40, the minimum working voltage is limited to 4.0V and the setting accuracy is 0.1V |
| `0x10` | `16` | Maximum torque | 2 | 1000 | EPROM | read&write | 0 | 1000 | 0.001 | Set the maximum output torque limit of the servo, and set 1000 = 100% * locked torque, Power on assigned to address 48 torque limit. |
| `0x12` | `18` | phase | 1 | 12 | EPROM | read&write | 0 | 254 | no | Special function byte, which cannot be modified without special requirements. See special byte bit analysis for details |
| `0x13` | `19` | Unloading condition | 1 | 44 | EPROM | read&write | 0 | 254 | no | Bit0 Bit1 bit2 bit3 bit4 bit5 corresponding bit is set to enable corresponding protection |
| `0x14` | `20` | LED Alarm condition | 1 | 47 | EPROM | read&write | 0 | 254 | no | The corresponding bit of temperature current angle overload of voltage sensor is set to 0 to close the corresponding protection. Bit0 Bit1 bit2 bit3 bit4 bit5 corresponding bit is set to enable flashing alarm. |
| `0x15` | `21` | P Proportionality coefficient | 1 | 32 | EPROM | read&write | 0 | 254 | no | Proportional factor of control motor |
| `0x16` | `22` | D Differential coefficient | 1 | 32 | EPROM | read&write | 0 | 254 | no | Differential coefficient of control motor |
| `0x17` | `23` | I Integral coefficient | 1 | 0 | EPROM | read&write | 0 | 254 | no | Integral coefficient of control motor |
| `0x18` | `24` | Minimum startup force | 2 | 16 | EPROM | read&write | 0 | 1000 | 0.001 | Set the minimum output starting torque of servo and set 1000 = 100% * locked torque |
| `0x1A` | `26` | Clockwise insensitive area | 1 | 1 | EPROM | read&write | 0 | 32 | step | The minimum unit is a minimum resolution angle |
| `0x1B` | `27` | Counterclockwise insensitive region | 1 | 1 | EPROM | read&write | 0 | 32 | step | The minimum unit is a minimum resolution angle |
| `0x1C` | `28` | Protection current | 2 | 500 | EPROM | read&write | 0 | 511 | 6.5mA | The maximum current can be set at 3255mA |
| `0x1E` | `30` | Angular resolution | 1 | 1 | EPROM | read&write | 1 | 100 | no | For the amplification factor of minimum resolution angle (degree / step), the number of control turns can be extended by modifying this value |
| `0x1F` | `31` | Position correction | 2 | 0 | EPROM | read&write | -2047 | 2047 | step | Bit11 is the direction bit, indicating the positive and negative directions. Other bits can represent the range of 0-2047 steps |
| `0x21` | `33` | Operation mode | 1 | 0 | EPROM | read&write | 0 | 2 | no | 0: position servo mode; 1: The motor is in constant speed mode, which is controlled by parameter 0x2e, and the highest bit 15 is the direction bit; 2: PWM open-loop speed regulation mode, with parameter 0x2c running time parameter control, bit11 as direction bit; 3: In step servo mode, the number of step progress is represented by parameter 0x2a, and the highest bit 15 is the direction bit. |
| `0x22` | `34` | Protective torque | 1 | 20 | EPROM | read&write | 0 | 254 | 0.01 | After entering the overload protection, the output torque, if set to 20, means 20% of the maximum torque |
| `0x23` | `35` | Protection time | 1 | 200 | EPROM | read&write | 0 | 254 | 10ms | The timing time when the current load output exceeds the overload torque and remains. If 200 is set to 2 seconds, the maximum can be set to 2.5 seconds |
| `0x24` | `36` | Overload torque | 1 | 80 | EPROM | read&write | 0 | 254 | 0.01 | The maximum torque threshold of starting overload protection time meter, if set to 80, means 80% of the maximum torque |
| `0x25` | `37` | Speed closed loop P proportional coefficient | 1 | 10 | EPROM | read&write | 0 | 254 | no | In the motor constant speed mode (mode 1), the speed loop proportional coefficient |
| `0x26` | `38` | Over current protection time | 1 | 200 | EPROM | read&write | 0 | 254 | 10ms | The maximum setting is 254 * 10ms = 2540ms |
| `0x27` | `39` | Velocity closed loop I integral coefficient | 1 | 10 | EPROM | read&write | 0 | 254 | no | In the motor constant speed mode (mode 1), the speed loop integral coefficient |
| `0x28` | `40` | Torque switch | 1 | 0 | SRAM | read&write | 0 | 2 | no | Write 0: turn off torque output; write 1: turn on torque output; write 128: current position correction is 2048 |
| `0x29` | `41` | acceleration | 1 | 0 | SRAM | read&write | 0 | 254 | 100step/s^2 | If it is set to 10, the speed will be changed by 1000 steps per second |
| `0x2A` | `42` | Target location | 2 | 0 | SRAM | read&write | -32766 | 32766 | step | Each step is a minimum resolution angle, absolute position control mode, the maximum corresponding to the maximum effective angle |
| `0x2C` | `44` | Running time | 2 | 0 | SRAM | read&write | 0 | 1000 | 0.001 | - |
| `0x2E` | `46` | running speed | 2 | 0 | SRAM | read&write | 0 | 254 | step/s | Number of steps in unit time (per second), 50 steps / second = 0.732 RPM (cycles per minute) |
| `0x30` | `48` | Torque limit | 2 | 1000 | SRAM | read&write | 0 | 1000 | 0.01 | The initial value of power on is assigned by the maximum torque (0x10), which can be modified by the user to control the output of the maximum torque |
| `0x37` | `55` | Lock mark | 1 | 0 | SRAM | read&write | 0 | 1 | no | Write 0 closes the write lock, and the value written to EPROM address is saved after power failure. Write 1 opens the write lock, and the value written to EPROM address is not saved after power failure. |
| `0x38` | `56` | current location | 2 | 0 | SRAM | read-only | -1 | -1 | step | In the absolute position control mode, the maximum value corresponds to the maximum effective angle |
| `0x3A` | `58` | Current speed | 2 | 0 | SRAM | read-only | -1 | -1 | step/s | Feedback the current speed of motor rotation, the number of steps in unit time (per second) |
| `0x3C` | `60` | Current load | 2 | 0 | SRAM | read-only | -1 | -1 | 0.001 | Voltage duty cycle of current control output drive motor |
| `0x3E` | `62` | Current voltage | 1 | 0 | SRAM | read-only | -1 | -1 | 0.1V | Current servo working voltage |
| `0x3F` | `63` | Current temperature | 1 | 0 | SRAM | read-only | -1 | -1 | °C | Current internal operating temperature of the servo |
| `0x40` | `64` | Asynchronous write flag | 1 | 0 | SRAM | read-only | -1 | -1 | no | When using asynchronous write instruction, flag bit |
| `0x41` | `65` | Servo status | 1 | 0 | SRAM | read-only | -1 | -1 | no | Bit0 Bit1 bit2 bit3 bit4 bit5 corresponding bit is set to 1, indicating that the corresponding error occurs,Voltage sensor temperature current angle overload corresponding bit 0 is no phase error. |
| `0x42` | `66` | Mobile sign | 1 | 0 | SRAM | read-only | -1 | -1 | no | When the servo is moving, it is marked as 1, and when the servo is stopped, it is 0 |
| `0x45` | `69` | Current current | 2 | 0 | SRAM | read-only | -1 | -1 | 6.5mA | The maximum measurable current is 500 * 6.5mA = 3250mA |

---

## Hexadecimal Instruction Generation Examples

This section provides pre-calculated instruction packets for common servo operations, detailing the decimal inputs, hexadecimal conversions, and the final generated instruction frame.

### 1. Turn Off Lock Protection Command

* **Input Parameters:**
  * ID = 254 (Input Range: SC 0-254)
  * Write Instruction = 3
  * First Address = 55
  * ID Data = 0 (Input Range: 0 close, 1 Open lock sign)

| Format | Header | Header | ID Number | Length | Instruction | Write First Address | ID Data | Check Code |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Hexadecimal** | `FF` | `FF` | `FE` | `04` | `03` | `37` | `00` | `C3` |
| **Decimal** | `255` | `255` | `254` | `4` | `3` | `55` | `0` | `195` |

**Generated Instruction:** `FF FF FE 04 03 37 00 C3`

---

### 2. Modify ID Number Instruction

* **Input Parameters:**
  * ID = 1 (Input Range: SC 0-254)
  * Write Instruction = 3
  * First Address = 5
  * New ID Data = 2

| Format | Header | Header | ID Number | Length | Instruction | Write First Address | ID Data | Check Code |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Hexadecimal** | `FF` | `FF` | `01` | `04` | `03` | `05` | `02` | `F0` |
| **Decimal** | `255` | `255` | `1` | `4` | `3` | `5` | `2` | `240` |

**Generated Instruction:** `FF FF 01 04 03 05 02 F0`

---

### 3. Modify Protection Conditions

* **Input Parameters:**
  * ID = 1 (Input Range: SC 0-254)
  * Write Instruction = 3
  * First Address = 19
  * Modify Protection Value = 32

| Format | Header | Header | ID Number | Length | Instruction | Write First Address | Protection Value | Check Code |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Hexadecimal** | `FF` | `FF` | `01` | `04` | `03` | `13` | `20` | `C4` |
| **Decimal** | `255` | `255` | `1` | `4` | `3` | `19` | `32` | `196` |

**Generated Instruction:** `FF FF 01 04 03 13 20 C4`

---

### 4. Current Position Correction to 2048

* **Input Parameters:**
  * ID = 1 (Input Range: SC/ST 0-254)
  * Write Instruction = 2
  * First Address = 40 (Input Range: 1 = open, 0 = close, 128 = Median automatic alignment)
  * Write Data = 128

| Format | Header | Header | ID Number | Length | Instruction | Write First Address | Write Data | Check Code |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Hexadecimal** | `FF` | `FF` | `01` | `03` | `02` | `28` | `80` | `51` |
| **Decimal** | `255` | `255` | `1` | `3` | `2` | `40` | `128` | `81` |

**Generated Instruction:** `FF FF 01 03 02 28 80 51`

---

### 5. Switching Force Enable Instruction

* **Input Parameters:**
  * ID = 253 (Input Range: SC/ST 0-254)
  * Write Instruction = 2
  * First Address = 40 (Input Range: 1 = open, 0 = close, 128 = Median automatic alignment)
  * Read Data Length = 0

| Format | Header | Header | ID Number | Length | Instruction | Write First Address | Read Data Length | Check Code |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Hexadecimal** | `FF` | `FF` | `FD` | `04` | `02` | `28` | `00` | `D4` |
| **Decimal** | `255` | `255` | `253` | `4` | `2` | `40` | `0` | `212` |

**Generated Instruction:** `FF FF FD 04 02 28 00 D4`

---

### 6. ST Series: Position & Speed Control

* **Input Parameters:**
  * ID = 2 (Input Range: SC 0-253 / ST 0-253)
  * Write Instruction = 3
  * Position = 1000 (Input Range: SC 0-1023 / ST 0-4095)
  * Time = 0 (Input Range: SC 2000)
  * Speed = 0 (Input Range: ST 0-150)

| Format | Header | Header | ID | Length | Instr | Write Address | Position (Low) | Position (High) | Time (Low) | Time (High) | Speed (Low) | Speed (High) | Check Code |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Hexadecimal** | `FF` | `FF` | `02` | `09` | `03` | `2A` | `E8` | `03` | `00` | `00` | `00` | `00` | `DC` |
| **Decimal** | `255` | `255` | `2` | `9` | `3` | `42` | `232` | `3` | `0` | `0` | `0` | `0` | `220` |

**Generated Instruction:** `FF FF 02 09 03 2A E8 03 00 00 00 00 DC`

---

### 7. ST Series: Acceleration, Position & Speed Control

* **Input Parameters:**
  * ID = 1 (Input Range: ST 0-253)
  * Write Instruction = 3
  * Acceleration = 50 (Input Range: 0-255, 8.878 °/s²)
  * Position = 1000 (Input Range: 0-4095, 0.088, Multi-loop control: -30719~+30719 ±7.5 turns)
  * empty set0 = 0
  * Speed = 1000 (Input Range: 0-1023, 0.732RPM)

| Format | Header | Header | ID | Length | Instr | Write Address | Accel Byte | Position (Low) | Position (High) | Time (Low) | Time (High) | Speed (Low) | Speed (High) | Check Code |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Hexadecimal** | `FF` | `FF` | `01` | `0A` | `03` | `29` | `32` | `E8` | `03` | `00` | `00` | `E8` | `03` | `C0` |
| **Decimal** | `255` | `255` | `1` | `10` | `3` | `41` | `50` | `232` | `3` | `0` | `0` | `232` | `3` | `192` |

**Generated Instruction:** `FF FF 01 0A 03 29 32 E8 03 00 00 E8 03 C0`

*(Note: Time low/high bytes explicitly state "no function" for this instruction configuration).*

---

### 8. Read Position Instruction (SC Series)

* **Input Parameters:**
  * ID = 1 (Input Range: SC 0-254 / ST 0-254)
  * Read Instruction = 2
  * First Address = 56
  * Number of Bytes = 2

| Format | Header | Header | ID Number | Length | Instruction | Write First Address | Number of Bytes | Check Code |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Hexadecimal** | `FF` | `FF` | `01` | `04` | `02` | `38` | `02` | `BE` |
| **Decimal** | `255` | `255` | `1` | `4` | `2` | `56` | `2` | `190` |

**Generated Instruction:** `FF FF 01 04 02 38 02 BE`

---

### 9. Number of Turns Cleared Command

* **Input Parameters:**
  * ID = 1 (Input Range: STBL 0-254)
  * Number of turns clear command = 10 (`0x0A`)

| Format | Header | Header | ID Number | Length | Instruction | Check Code |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Hexadecimal** | `FF` | `FF` | `01` | `02` | `0A` | `F2` |
| **Decimal** | `255` | `255` | `1` | `2` | `10` | `242` |

**Generated Instruction:** `FF FF 01 02 0A F2`
