# Smart Bus Servo Communication Protocol Manual

## Table of contents

| Section | Description |
| :--- | :--- |
| **[1.0 Communication Protocol Outline](#10-communication-protocol-outline)** | Overview of hardware layers, data frames, and byte order mechanics. |
| **[1.1 Instruction Packet](#11-instruction-packet)** | Structural formatting for outgoing command frames. |
| **[1.2 Reply Packet](#12-reply-packet)** | Structural formatting for incoming hardware responses and error states. |
| **[1.3 Instruction Type](#13-instruction-type)** | Registry of supported hexadecimal function codes. |
| &nbsp;&nbsp;&nbsp;[1.3.1 PING Query Status](#131-ping-query-status-instruction) | Reading the online/working state of a specific servo. |
| &nbsp;&nbsp;&nbsp;[1.3.2 READ DATA](#132-read-data) | Methodology for extracting values from the control table. |
| &nbsp;&nbsp;&nbsp;[1.3.3 WRITE DATA](#133-write-data) | Immediate execution of commands to specific memory addresses. |
| &nbsp;&nbsp;&nbsp;[1.3.4 REG WRITE](#134-reg-write) | Buffering data for synchronized execution. |
| &nbsp;&nbsp;&nbsp;[1.3.5 ACTION](#135-action-executing-asynchronous-write-instruction) | Triggering execution for all previously buffered `REG WRITE` commands. |
| &nbsp;&nbsp;&nbsp;[1.3.6 SYNC WRITE](#136-sync-write) | High-speed simultaneous control of multiple servos in one frame. |
| &nbsp;&nbsp;&nbsp;[1.3.7 SYNC READ](#137-sync-read) | Bulk querying of multiple servos to optimize bus bandwidth[cite: 2]. |
| &nbsp;&nbsp;&nbsp;[1.3.8 RESET Instruction](#138-reset-instruction) | Restoring hardware parameters to factory default values. |
| **[1.4 Memory Tables](#14-memory-tables)** | Servo-specific registry mappings and operational parameters. |

## 1.0 Communication Protocol Outline

The Serial Bus Smart Servo Communication protocol is mainly applicable to the Potentiometer series of servos and Magnetic Encoding servos.

* **Potentiometer series:** Servos adopt TTL level and single bus (a signal line time-sharing multiplexing transmission and receiving data signal) communication connection. The physical connection is three lines, including two positive and negative poles of power supply.

* **Magnetic Encoding series:** Servos adopt an ARM 32-bit single-chip computer as the main control core, and position induction adopts a 360-degree 12-bit precision magnet induction angle scheme. The communication level adopts RS485 mode with strong anti-interference capability. The communication still adopts asynchronous duplex, and the sending and receiving signals are processed asynchronously.

**Question-and-answer communication** is adopted between the controller and the servo. The controller sends out the instruction packet
 and the servo returns the response packet
.

Multiple servos are allowed in a bus control network, so each servo is assigned a unique ID number in the network. The control instruction sent by the controller contains ID information. Only the servo matching the ID number can receive the command completely and return the response information.

The communication mode is **serial asynchronous.** A frame of data is divided into 1 start bit, 8 data bits, and 1 stop bit. There are no parity bits, totaling 10 bits.

**The difference between the communication protocol of Potentiometer series and Magnetic Encoding series servos:** when some parameters of the memory table are in the range of two bytes, the two bytes respectively represent the high byte and the low byte, in which the address of **Potentiometer series** parameters is the **high byte before the low byte,** while the **Magnetic Encoding** series servo is the **low byte before the high byte.** In addition, each servo has slightly different functions, so the actual control should refer to the [memory table](#14-memory-tables) of the specific model.

---

## 1.1 Instruction Packet

**Instruction packet
 format:**

| Header | ID | Data Length | Instruction | Parameters | Checksum |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `0xFF 0xFF` | `ID` | `Length` | `Instruction` | `Parameter1 ... ParameterN` | `Checksum` |

* **Header:** Continuous receipt of two `0xFF`s indicating the arrival of data packets.
* **ID No.:** Each servo has an ID number. The ID number ranges from 0 to 253, converted to hexadecimal `0x00` to `0xFD`.
* **Broadcast ID:** The ID No. 254 (e.g. `0xFE`) is a broadcast ID. If the ID number sent by the controller is 254, all the Servos receive instructions, and no response information is returned except for PING instructions (broadcast PING instruction cannot be used when multiple servos are connected to the serial bus).
* **Data Length:** Equal to the parameter N to be sent plus 2 (Instruction + Checksum), that is: N+2.
* **Instruction:** Packet Operating Function Code (see [Section 1.3](#13-instruction-type)).
* **Parameters:** In addition to the additional control information required by the instructions, the parameters support a maximum two-byte parameter to represent a memory value. The byte order refers to the manual memory control table for servo usage (different types of servo have different byte orders).
* **Checksum:** Calculated as follows: `Checksum = ~(ID + Length + Instruction + Parameter1 + ... + ParameterN)`, where `~` symbol represents bitwise negation (e.g. taking the bitwise complement of a byte). If the sum in parentheses exceeds 255, the lowest byte will be taken.

---

## 1.2 Reply Packet

The reply packet is the servo’s response to the controller.

**Reply packet format:**

| Header | ID No. | Data Length | Current State | Parameter | Checksum |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `0xFF 0xFF` | `ID` | `Length` | `ERROR` | `Parameter1 ... ParameterN` | `Checksum` |

* The returned response packet
 contains the current status (`ERROR`) of the servo.
* If the current status of the servo is not normal, it will be reflected through this byte (the meaning of each status is detailed in the manual memory control table).
* If `ERROR` is `0`, the servo will have no error information.
* If the instruction is a read instruction (`READ DATA`), then `Parameter1 ... ParameterN` is the information that have been read.

---

## 1.3 Instruction Type

The following instructions are available for the Serial Bus Smart Servo Communication Protocol:

| Instruction | Function | Value | Parameter Length |
| :--- | :--- | :--- | :--- |
| **PING** | Query the working status | `0x01` | 0 |
| **READ DATA** | Query the Characters in the Control Table | `0x02` | 2 |
| **WRITE DATA** | Write characters into the control table | `0x03` | ≥ 1 |
| **REG WRITE** | Similar to WRITE DATA, but the control character does not act immediately after writing until the ACTION instruction arrives. | `0x04` | Not less than 2 |
| **ACTION** | Actions that trigger REG WRITE writes | `0x05` | 0 |
| **SYNC READ** | Query multiple servos at the same time | `0x82` | ≥ 3 |
| **SYNC WRITE** | For simultaneous control of multiple servos | `0x83` | Not less than 2 |
| **RESET** | Reset control table to factory value | `0x06` | 0 |

### 1.3.1 PING Query Status Instruction

* **Function:** Read the working state of the servo
* **Length:** `0x02`
* **Instruction:** `0x01`
* **Parameter:** None

The PING instruction uses the broadcast address, and the servo also returns the response information.

**Example 1:** Read the working state of the servo with ID number 1.

*Instruction frame (sent in hexadecimal):* `FF FF 01 02 01 FB`

| Header | ID | Effective data length | Instruction | Checksum |
| :--- | :--- | :--- | :--- | :--- |
| `0xFF 0xFF` | `0x01` | `0x02` | `0x01` | `0xFB` |

*Data frame returned (hexadecimal display):* `FF FF 01 02 00 FC`

| Header | ID | Effective data length | Working status | Checksum |
| :--- | :--- | :--- | :--- | :--- |
| `0xFF 0xFF` | `0x01` | `0x02` | `0x00` | `0xFC` |

### 1.3.2 READ DATA

* **Function:** Reads data from the servo memory control table
* **Length:** `0x04`
* **Instruction:** `0x02`
* **Parameter 1:** First address of read-out segment of data
* **Parameter 2:** Length of read data

**Example 2:** Read the current position of the servo with ID 1 (low byte before, high byte after). Two bytes are read from address `0x38` in the control table.

*Instruction frame (sent in hexadecimal):* `FF FF 01 04 02 38 02 BE`

| Header | ID | Effective data length | Instruction | Parameter | Checksum |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `0xFF 0xFF` | `0x01` | `0x04` | `0x02` | `0x38 0x02` | `0xBE` |

*Data frame returned (hexadecimal display):* `FF FF 01 04 00 18 05 DD`

| Header | ID | Effective data length | Working status | Parameter | Checksum |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `0xFF 0xFF` | `0x01` | `0x04` | `0x00` | `0x18 0x05` | `0xDD` |

*Read out two-byte data:* low byte L is `0x18`, high byte H is `0x05`. Two-byte synthesis of 16-bit data is `0x0518`, , the current position is represented in decimal as 1304.

### 1.3.3 WRITE DATA

* **Function:** Write data to the servo memory control table
* **Length:** N+3 (N is the parameter length)
* **Instruction:** `0x03`
* **Parameter 1:** Head address of data write segment
* **Parameter 2:** The first data written
* **Parameter 3:** Second data
* ...
* **Parameter N+1:** Number N Data

**Example 3:** Sets an ID of any number to 1. The address of the ID number is 5 in the control table, so write 1 at address 5. The ID of the sending instruction packet
 uses the broadcast ID (`0xFE`).

*Instruction frame (sent in hexadecimal):* `FF FF FE 04 03 05 01 F4`

| Header | ID | Effective data length | Instruction | Parameter | Checksum |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `0xFF 0xFF` | `0xFE` | `0x04` | `0x03` | `0x05 0x01` | `0xF4` |

Because broadcasting ID is used to send instructions, there will be no data return.

> [!WARNING]
> *The memory table EPROM has a protective lock switch, which needs to be turned off before modifying the ID, otherwise the sample ID number will not be saved when power is off. For detailed operation, please refer to the memory table or operation manual of the specific servo model number.*

**Example 4:** Controls the servo with ID = 1 to rotate to 2048 at a speed of 1000 steps per second.

In the control table, the first address of the target location is `0x2A`, so six consecutive bytes of data are written at the address `0x2A`, namely:

* Position data `0x0800` (2048),
* Time data `0x0000` (0),
* Speed data `0x03E8` (1000).

The ID of the sending instruction packet
 uses a non-broadcast ID (`0x01`).

*Instruction frame (sent in hexadecimal):* `FF FF 01 09 03 2A 00 08 00 00 E8 03 D5`

| Header | ID | Effective data length | Instruction | Parameter | Checksum |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `0xFF 0xFF` | `0x01` | `0x09` | `0x03` | `0x2A 0x00 0x08 0x00 0x00 0xE8 0x03` | `0xD5` |

*Data frame returned (hexadecimal display):* `FF FF 01 02 00 FC`

| Header | ID | Effective data length | Working status | Checksum |
| :--- | :--- | :--- | :--- | :--- |
| `0xFF 0xFF` | `0x01` | `0x02` | `0x00` | `0xFC` |

The return working status is 0, indicating that the servo has correctly received the instructions and has begun to execute them.

### 1.3.4 REG WRITE

The REG WRITE instruction is similar to WRITE DATA except that the execution time is different. When the REG WRITE instruction frame is received, the received data is stored in the buffer reserve for standby, and the Registered Instruction Register is set at 1. When the ACTION instruction is received, the stored instruction is finally executed.

* **Length:** N+3 (N is the number of data to be written)
* **Instruction:** `0x04`
* **Parameter 1:** The first address of the area where the data is to be written
* **Parameter 2:** The first data to be written
* **Parameter 3:** The second data to be written
* ...
* **Parameter N+1:** The Nth Data to Write

**Example 5:** Control ID = 1 to ID = 10 servos to rotate to 2048 position at 1000 steps per second. In the following instruction packets, only the ID = 1 is on the bus and receives the instruction and returns, and other ID are not returned on the bus.

* **ID 1 Asynchronous Write Instruction packet
:** `FF FF 01 09 04 2A 00 08 00 00 E8 03 D4`
* **ID 1 Return packet
:** `FF FF 01 02 00 FC`
* **ID 2 Asynchronous Write Instruction packet
:** `FF FF 02 09 04 2A 00 08 00 00 E8 03 D3`
* **ID 3 Asynchronous Write Instruction packet
:** `FF FF 03 09 04 2A 00 08 00 00 E8 03 D2`
* **ID 4 Asynchronous Write Instruction packet
:** `FF FF 04 09 04 2A 00 08 00 00 E8 03 D1`
* **ID 5 Asynchronous Write Instruction packet
:** `FF FF 05 09 04 2A 00 08 00 00 E8 03 D0`
* **ID 6 Asynchronous Write Instruction packet
:** `FF FF 06 09 04 2A 00 08 00 00 E8 03 CF`
* **ID 7 Asynchronous Write Instruction packet
:** `FF FF 07 09 04 2A 00 08 00 00 E8 03 CE`
* **ID 8 Asynchronous Write Instruction packet
:** `FF FF 08 09 04 2A 00 08 00 00 E8 03 CD`
* **ID 9 Asynchronous Write Instruction packet
:** `FF FF 09 09 04 2A 00 08 00 00 E8 03 CC`
* **ID 10 Asynchronous Write Instruction packet
:** `FF FF 0A 09 04 2A 00 08 00 00 E8 03 CB`

### 1.3.5 ACTION (Executing Asynchronous Write Instruction)

* **Function:** Trigger REG WRITE instruction
* **Length:** `0x02`
* **Instruction:** `0x05`
* **Parameter:** None

ACTION instructions are very useful for controlling multiple servos at the same time. When controlling multiple servos, the ACTION instruction enables the first and last servos to perform their respective actions simultaneously without delay. When the action instruction is sent to multiple servos, the broadcast ID (`0xFE`) is used, so no data frame will be returned when the instruction is sent.

**Example 6:** After issuing the asynchronous writing instructions that control ID 1 to ID 10 servos to rotate at 2048 position at a speed of 1000 steps per second, the following instruction packet
 (`FF FF FE 02 05 FA`) needs to be sent when the asynchronous writing instructions need to be executed. All servos on the bus receive this instruction and run the asynchronous writing instruction received before.

### 1.3.6 SYNC WRITE

* **Function:** Used to control multiple servos at the same time
* **ID:** `0xFE`
* **Length:** (L+1)*N+4 (where L: Length of data sent to each servo, N: Servo Number)
* **Instruction:** `0x83`
* **Parameter 1:** First address of written data
* **Parameter 2:** Length of written data (L)
* **Parameter 3:** First servo ID number
* **Parameter 4:**  the first written data of the first servo
* **Parameter 5:**  the second written data of the first servo
* ...
* **Parameter L+3:**  the L-th written data of the first servo
* **Parameter L+4:** Second Servo ID number
* **Parameter L+5:** the first written data of the second servo
* **Parameter L+6:** the second written data of the second servo
* ...
* **Parameter 2L+4:** the L-th written data of the second servo

Unlike the REG WRITE+ACTION instruction, the real-time performance is higher. A SYNC WRITE instruction can modify the control table contents of multiple servos at one time, while the REG WRITE+ACTION instruction is done step by step. Nevertheless, when using SYNC WRITE instructions, **the length of the written data must be the same as the first address of the data saved.**

**Example 7:** Writing position `0x0800`, time `0x0000`, and speed `0x03E8` for ID = 1 to ID = 4 with four servo header addresses `0x2A` (low byte before, high byte after).

*Instruction frame (sent in hexadecimal):* `FF FF FE 20 83 2A 06 01 00 08 00 00 E8 03 02 00 08 00 00 E8 03 03 00 08 00 00 E8 03 04 00 08 00 00 E8 03 58`

| Header | ID | Effective data length | Instruction | Parameter | Checksum |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `0xFF 0xFF` | `0xFE` | `0x20` | `0x83` | `0x2A 0x06 0x01 0x00 0x08 0x00 0x00 0xE8 0x03 0x02 0x00 ...` | `0x58` |

Because broadcasting ID is used to send instructions, no data is returned.

### 1.3.7 SYNC READ

* **Function:** Used to query multiple servos at the same time
* **ID:** `0xFE`
* **Length:** N+4 (where N: Servo Number)
* **Instruction:** `0x82`
* **Parameter 1:** First address of reading data
* **Parameter 2:** Length of reading data (L)
* **Parameter 3:** First servo ID number
* **Parameter 4:**  Second servo ID number
* ...
* **Parameter N+2:**  N-th servo ID number

A SYNC READ instruction can query the contents of the control
tables of multiple servos at one time. The SYNC READ instruction specifies the ID of the servos to be queried, and the order in which the
servos return the response packet
s is the ID order in the instruction
packet
. When using the SYNC READ instruction, the data length and the first address of all queries must be the same (this instruction is open to some serial bus servos).

**Example 8:** Inquires about the first address `0x38` of two servos (ID = 1 and ID = 2), including the data of current position, current speed, current load, current voltage, and current temperature. This totals 8 bytes of data (the low byte comes first and the high byte comes last).

*Synchronous reading instruction packet
 (sent in hexadecimal):* `FF FF FE 06 82 38 08 01 02 36`

| Header | ID | Effective data length | Instruction | Parameter | Checksum |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `0xFF 0xFF` | `0xFE` | `0x06` | `0x82` | `0x38 0x08 0x01 0x02` | `0x36` |

*Return packet
s (hexadecimal display):*

* `FF FF 01 0A 00 00 08 00 00 00 00 79 1E 55`
* `FF FF 02 0A 00 FF 07 00 00 00 00 77 23 53`

*Decoding the return packets:*

**ID = 1 return packet
:** `FF FF 01 0A 00 00 08 00 00 00 00 79 1E 55`

| Header | ID | Effective Data length | Working status | Parameters | Checksum |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `0xFF 0xFF` | `0x01` | `0x0A` | `0x00` | `0x00 0x08 0x00 0x00 0x00 0x00 0x79 0x1E` | `0x55` |

**ID = 2 response packet
:** `FF FF 02 0A 00 FF 07 00 00 00 00 77 23 53`

| Header | ID | Effective Data length | Working status | Parameters | Checksum |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `0xFF 0xFF` | `0x02` | `0x0A` | `0x00` | `0xFF 0x07 0x00 0x00 0x00 0x00 0x77 0x23` | `0x53` |

### 1.3.8 RESET Instruction

* **Function:** Reset the specific data in the memory control table (specific Servo type is used)
* **Length:** `0x02`
* **Instruction:** `0x06`
* **Parameter:** None

**Example 9:** Reset servo, ID number is 0.

*Instruction frame (sent in hexadecimal):* `FF FF 01 02 06 F6`

| Header | ID | Effective data length | Instruction | Checksum |
| :--- | :--- | :--- | :--- | :--- |
| `0xFF 0xFF` | `0x00` | `0x02` | `0x06` | `0xF7` |

*Returned data frame (sent in hexadecimal):* `FF FF 01 02 00 FC`

| Header | ID | Effective data length | Working condition | Checksum |
| :--- | :--- | :--- | :--- | :--- |
| `0xFF 0xFF` | `0x01` | `0x02` | `0x00` | `0xFC` |

## 1.4 Memory Tables

Refer to the memory tables below to identify the specific registry addresses, factory default initial values, read/write permissions, and unit constraints required to configure parameters for the dedicated servo.

* [STS3215 Memory table](./sts3215_memory_table.md)
