# ANENJI Inverter — Modbus RTU Communication Protocol

**Version: June 2025, v1.0**

This is an English translation of the manufacturer's Modbus RTU communication protocol
document for the ANENJI ANJ series inverter/charger.

---

## Table of Contents

1. [Communication Data Format](#1-communication-data-format)
2. [Frame Format](#2-frame-format)
   - [Register Read (Function Code 03H)](#21-register-read-function-code-03h)
   - [Register Write (Function Code 10H)](#22-register-write-function-code-10h)
3. [Device Register Map](#3-device-register-map)
   - [Read-only / Status Registers (100–299)](#31-read-only--status-registers-100299)
   - [Read/Write Settings Registers (300–461)](#32-readwrite-settings-registers-300461)
   - [Diagnostic / Log Registers (700–729)](#33-diagnostic--log-registers-700729)
4. [Fault Code Table](#4-fault-code-table)
5. [Warning Code Table](#5-warning-code-table)
6. [Power Flow Flags](#6-power-flow-flags)
7. [CRC-16/MODBUS Algorithm](#7-crc-16modbus-algorithm)

---

## 1. Communication Data Format

Data is returned word by word (WORD = 2 bytes). In each word, the **high byte comes first,
low byte second**. For consecutive 2-word (long integer) values, the **high word comes
first, low word second**.

| Data Type      | Register Count | Byte Count | Notes                                               |
|---------------|----------------|------------|-----------------------------------------------------|
| Character (ASC)| 1             | 2          | Two characters per register; padded with 0 if needed |
| Integer (Int)  | 1             | 2          | High byte first, low byte second                    |
| Long Integer   | 2             | 4          | Two registers, high word first, low word second     |
| Unsigned Int (UInt)  | 1       | 2          | High byte first                                     |
| Unsigned Long (ULong)| 2       | 4          | High word first                                     |

---

## 2. Frame Format

> **Important:** The start and end addresses of any query must align exactly with a
> complete data block. For example, the device serial number starts at register 186 with
> length 12. The query start address must not fall within 186–198, and the end address
> (start + count) must not fall in that range either.

### 2.1 Register Read (Function Code 03H)

#### Master → Slave (query frame)

| Byte | Code  | Example | Description                    |
|------|-------|---------|-------------------------------|
| 0    | Device address | 01H | Address 1–247 |
| 1    | Function code  | 03H | Read holding registers |
| 2    | Start register high byte | 00H | Upper 8 bits of register address |
| 3    | Start register low byte  | 10H | Lower 8 bits of register address |
| 4    | Register count high byte | 00H | Upper 8 bits of count |
| 5    | Register count low byte  | 02H | Lower 8 bits of count |
| 6    | CRC16 high byte          | C0H | CRC high byte |
| 7    | CRC16 low byte           | CBH | CRC low byte |

#### Slave → Master (success response)

| Byte   | Code | Description |
|--------|------|-------------|
| 0      | Device address | 1–247 |
| 1      | 03H  | Function code |
| 2      | N    | Returned byte count = requested register count × 2 |
| 3      | —    | First register data, high byte |
| 4      | —    | First register data, low byte |
| …      | …    | … |
| N+2    | —    | Last register data, low byte |
| N+3    | CRC16 high byte | All bytes before CRC are included |
| N+4    | CRC16 low byte  | |

#### Slave → Master (error response)

Returns N bytes of 0x00 in the data field instead of real data.

#### Example — Read grid voltage (reg 202) through grid average power (reg 204)

Grid voltage = 230.0 V, frequency = 50.0 Hz, average power = 1200 W

```
Master → 01 03 00 CA 00 03 25 F5
Slave  → 01 03 06 08 FC 13 88 04 B0 F7 F3
```

Decoded:
- `08FC` = 2300 → 230.0 V
- `1388` = 5000 → 50.00 Hz
- `04B0` = 1200 → 1200 W

---

### 2.2 Register Write (Function Code 10H)

#### Master → Slave (write frame)

| Byte | Code | Example | Description |
|------|------|---------|-------------|
| 0    | Device address | 01H | 1–247 |
| 1    | Function code  | 10H | Write multiple registers |
| 2    | Start register high byte | 01H | |
| 3    | Start register low byte  | 10H | |
| 4    | Register count high byte | 00H | Always 0 |
| 5    | Register count low byte  | 02H | |
| 6    | Byte count N             | — | N = register count × 2 |
| 7    | First register high byte | — | |
| 8    | First register low byte  | — | |
| …    | …                        | … | |
| N+6  | Last register low byte   | — | |
| N+7  | CRC16 high byte          | — | |
| N+8  | CRC16 low byte           | — | |

#### Slave → Master (success response)

| Byte | Code | Example | Description |
|------|------|---------|-------------|
| 0    | Device address | 01H | |
| 1    | 10H            | 10H | Function code echo |
| 2    | Start register high byte | 01H | |
| 3    | Start register low byte  | 10H | |
| 4    | Register count high byte | 00H | Always 0 |
| 5    | Register count low byte  | 02H | |
| 6    | CRC16 high byte | 41H | |
| 7    | CRC16 low byte  | F1H | |

#### Slave → Master (error response)

| Byte | Code | Description |
|------|------|-------------|
| 0    | Device address | |
| 1    | 90H            | Error indicator (function code OR 0x80) |
| 2    | Error code     | See table below |
| 3    | CRC16 high byte | |
| 4    | CRC16 low byte  | |

#### Write error codes

| Code | Meaning |
|------|---------|
| 01H  | Attempted to write a read-only register |
| 03H  | Data value out of acceptable range |
| 07H  | Register cannot be modified in the current working mode |

#### Example — Set output voltage (reg 320) to 220 V

```
Master → 01 10 01 40 00 01 02 08 98 BE 3A
Slave  → 01 10 01 40 00 01 01 E1
```

---

## 3. Device Register Map

**Column key:**
- **R** = readable (function code 03H)
- **W** = writable (function code 10H)
- **R/W** = read and write
- Data types: `Int` = signed 16-bit; `UInt` = unsigned 16-bit; `Long` = signed 32-bit (2 registers); `ULong` = unsigned 32-bit; `ASC` = ASCII text
- `Max` / `Min` = take maximum / minimum of expression
- All addresses are decimal

---

### 3.1 Read-only / Status Registers (100–299)

| Name | Unit | Type | Start | Regs | R/W | Notes |
|------|------|------|-------|------|-----|-------|
| Device fault code | — | ULong | 100 | 2 | R | 32-bit bitmask. Bit N = fault code N+1. See §4 |
| Reserved | — | — | 102 | 2 | — | |
| Unmasked warning code | — | ULong | 104 | 2 | R | 32-bit bitmask, see §5 |
| Reserved | — | — | 106 | 2 | — | |
| Masked warning code | — | ULong | 108 | 2 | R/W | 32-bit bitmask, see §5 |
| Reserved | — | — | 110 | 61 | — | |
| Device type | — | UInt | 171 | 1 | R | |
| Device name | — | ASC | 172 | 12 | R/W | Read/write as ASCII |
| Protocol number | — | UInt | 184 | 1 | R | Protocol identifier; returns 1 for this protocol |
| Reserved | — | — | 185 | 1 | — | |
| Device serial number | — | ASC | 186 | 12 | R | |
| Reserved | — | — | 198 | 2 | — | |
| Internal command register | — | UInt | 200 | 1 | — | Reserved for internal firmware use; do not access |
| Working mode | — | UInt | 201 | 1 | R | 0=Boot; 1=Standby; 2=Grid (mains); 3=Off-grid (inverting); 4=Bypass; 5=Charge; 6=Fault |
| Grid voltage (RMS) | 0.1 V | Int | 202 | 1 | R | |
| Grid frequency | 0.01 Hz | Int | 203 | 1 | R | |
| Grid average power | 1 W | Int | 204 | 1 | R | |
| Inverter output voltage (RMS) | 0.1 V | Int | 205 | 1 | R | |
| Inverter output current (RMS) | 0.1 A | Int | 206 | 1 | R | |
| Inverter output frequency | 0.01 Hz | Int | 207 | 1 | R | |
| Inverter average power | 1 W | Int | 208 | 1 | R | Positive = inverting out; negative = inverting in |
| Inverter charge power | 1 W | Int | 209 | 1 | R | |
| Output voltage (RMS) | 0.1 V | Int | 210 | 1 | R | |
| Output current (RMS) | 0.1 A | Int | 211 | 1 | R | |
| Output frequency | 0.01 Hz | Int | 212 | 1 | R | |
| Output active power | 1 W | Int | 213 | 1 | R | |
| Output apparent power | 1 VA | Int | 214 | 1 | R | |
| Battery average voltage | 0.1 V | Int | 215 | 1 | R | |
| Battery average current | 0.1 A | Int | 216 | 1 | R | |
| Battery average power | 1 W | Int | 217 | 1 | R | |
| Internal command register | — | — | 218 | 1 | — | Reserved for internal firmware use; do not access |
| PV average voltage | 0.1 V | Int | 219 | 1 | R | |
| PV average current | 0.1 A | Int | 220 | 1 | R | |
| Reserved | — | — | 221 | 2 | — | |
| PV average power | 1 W | Int | 223 | 1 | R | |
| PV charge average power | 1 W | Int | 224 | 1 | R | |
| Load percentage | 1 % | Int | 225 | 1 | R | |
| DCDC temperature | 1 °C | Int | 226 | 1 | R | |
| Inverter temperature | 1 °C | Int | 227 | 1 | R | |
| Reserved | — | — | 228 | 1 | — | |
| Battery percentage (SOC) | 1 % | UInt | 229 | 1 | R | |
| Internal command register | — | — | 230 | 1 | — | Reserved for internal firmware use; do not access |
| Power flow status | — | UInt | 231 | 1 | R | 16-bit bitmask, see §6 |
| Battery filtered average current | 0.1 A | Int | 232 | 1 | R | Positive = charging; negative = discharging |
| Inverter charge current average | 0.1 A | Int | 233 | 1 | R | |
| PV charge current average | 0.1 A | Int | 234 | 1 | R | |
| Internal command register | — | — | 235 | 1 | — | Reserved for internal firmware use; do not access |
| Internal command register | — | — | 236 | 1 | — | Reserved for internal firmware use; do not access |
| Reserved | — | — | 237 | 63 | — | |

---

### 3.2 Read/Write Settings Registers (300–461)

| Name | Unit | Type | Start | Regs | R/W | Notes |
|------|------|------|-------|------|-----|-------|
| Output mode | — | UInt | 300 | 1 | R/W | 0=Single; 1=Parallel; 2=Three-phase P1; 3=Three-phase P2; 4=Three-phase P3 |
| Output priority | — | UInt | 301 | 1 | R/W | 0=Mains→PV→Battery (UTI); 1=PV→Mains→Battery (SOL, inverter priority); 2=PV→Battery→Mains (SBU); 3=PV→Mains→Battery (SUB, mains priority) |
| Input voltage range | — | UInt | 302 | 1 | R/W | 0=APL; 1=UPS |
| Buzzer mode | — | UInt | 303 | 1 | R/W | 0=Always silent; 1=Beep on source change or specific warning/fault; 2=Beep on specific warning/fault; 3=Beep on fault only |
| Reserved | — | — | 304 | 1 | R/W | |
| LCD backlight | — | UInt | 305 | 1 | R/W | 0=Auto off; 1=Always on |
| LCD auto-return to home | — | UInt | 306 | 1 | R/W | 0=Disabled; 1=Return after 1 minute |
| Energy-saving mode | — | UInt | 307 | 1 | R/W | 0=Off; 1=On |
| Auto-restart on overload | — | UInt | 308 | 1 | R/W | 0=No; 1=Yes |
| Auto-restart on over-temp | — | UInt | 309 | 1 | R/W | 0=No; 1=Yes |
| Transfer to bypass on overload | — | UInt | 310 | 1 | R/W | 0=Disabled; 1=Enabled |
| Reserved | — | — | 311 | 2 | — | |
| Battery equalization (EQ) enable | — | UInt | 313 | 1 | R/W | 0=Disabled; 1=Enabled |
| Warning mask [I] | — | ULong | 314 | 2 | R/W | Bit=1 shows warning; Bit=0 masks it |
| Dry contact | — | UInt | 316 | 1 | R/W | 0=Normal; 1=Ground box mode |
| Reserved | — | — | 317 | 3 | — | |
| Output voltage | 0.1 V | UInt | 320 | 1 | R/W | 2200=220 V; 2300=230 V; 2400=240 V |
| Output frequency | 0.01 Hz | UInt | 321 | 1 | R/W | 5000=50 Hz; 6000=60 Hz |
| Battery type | — | UInt | 322 | 1 | R/W | 0=AGM; 1=FLD; **2=USER**; 3=Li1; 4=Li2; 5=Li3; 6=Li4 |
| Battery over-voltage protection [A] | 0.1 V | UInt | 323 | 1 | R/W | Range: (B + 1 V × J) … 16.5 V × J |
| Max charge voltage [B] | 0.1 V | UInt | 324 | 1 | R/W | Range: C … (A − 1 V) |
| Float charge voltage [C] | 0.1 V | UInt | 325 | 1 | R/W | Range: (12 V × J) … B |
| Grid-mode battery discharge recovery point [D] | 0.1 V | UInt | 326 | 1 | R/W | Range: (B − 0.5 V × J) … Max(12 V × J, E). Set to 0 = recover when full |
| Grid-mode battery low-voltage protection [E] | 0.1 V | UInt | 327 | 1 | R/W | Range: Min(14.3 V × J, D) … Max(11 V × J, F) |
| Reserved | — | — | 328 | 1 | — | |
| Off-grid battery low-voltage protection [F] | 0.1 V | UInt | 329 | 1 | R/W | Range: (10 V × J) … Min(13.5 V × J, E) |
| CV-to-float wait time | min | UInt | 330 | 1 | R/W | Range: 1–900. Set 0 = default 10 min |
| Battery charge priority | — | UInt | 331 | 1 | R/W | 0=Mains first; 1=PV first; 2=PV and mains equal; 3=PV only |
| Max charge current [G] | 0.1 A | UInt | 332 | 1 | R/W | Range: Max(10 A, H) … 80 A. **Requires battery type = USER (reg 322 = 2)** |
| Max mains charge current [H] | 0.1 A | UInt | 333 | 1 | R/W | Range: 2 A … G |
| EQ charge voltage | 0.1 V | UInt | 334 | 1 | R/W | Range: C … (A − 0.5 V × J) |
| EQ charge time | min | UInt | 335 | 1 | R/W | Range: 0–900 |
| EQ timeout exit | min | UInt | 336 | 1 | R/W | Range: 0–900 |
| EQ charge interval | day | UInt | 337 | 1 | R/W | Range: 1–90 |
| Auto mains output enable | — | UInt | 338 | 1 | R/W | 0=Mains output only after pressing power button; 1=Automatic mains output without pressing power button |
| Reserved | — | — | 339 | 2 | — | |
| Grid-mode battery discharge SOC protection [K] | 1 % | UInt | 341 | 1 | R/W | Range: 5–96 % |
| Grid-mode battery discharge SOC recovery | 1 % | UInt | 342 | 1 | R/W | Range: 10–100 % |
| Off-grid battery discharge SOC protection | 1 % | UInt | 343 | 1 | R/W | Range: 0 … Min(K, 95 %) |
| Reserved | — | — | 344 | 7 | — | |
| Max discharge current protection | 1 A | UInt | 351 | 1 | R/W | Maximum discharge current limit (single-unit mode) |
| Reserved | — | — | 352 | 3 | — | |
| Grid recognition time | s | UInt | 355 | 1 | R/W | Time the device waits after detecting mains before treating it as available. Range: 5–300 |
| Lithium activation time | s | UInt | 356 | 1 | R/W | Range: 6–300 |
| Lithium stop-charge SOC | 1 % | UInt | 357 | 1 | R/W | After successful BMS communication, stop charging when SOC reaches this value. Range: 20–100 |
| BMS follow mode | — | UInt | 358 | 1 | R/W | 0=Do not follow BMS voltage/current; 1=CV/Float voltage follows BMS (not settable), current does not follow; 2=CV/Float voltage and max charge current follow BMS (not settable); 3=Stop by SOC not voltage, current does not follow; 4=Stop by SOC not voltage, max charge current follows BMS (not settable) |
| EEPROM max charge voltage | 0.1 V | UInt | 359 | 1 | R | |
| EEPROM float charge voltage | 0.1 V | UInt | 360 | 1 | R | |
| EEPROM max charge current | 0.1 A | UInt | 361 | 1 | R | |
| Reserved | — | — | 362 | 38 | — | Includes firmware-internal entries |
| Power-on mode | — | UInt | 406 | 1 | R/W | 0=Local or remote power-on; 1=Local only; 2=Remote only |
| Reserved | — | — | 407 | 13 | — | |
| Remote switch | — | UInt | 420 | 1 | R/W | 0=Remote off; 1=Remote on |
| Internal command register | — | — | 421 | 1 | — | Reserved for internal firmware use; do not access |
| Reserved | — | — | 422 | 3 | — | |
| Force EQ charge | — | UInt | 425 | 1 | W | Write 1 to manually force one EQ charge cycle |
| Exit fault-lock state | — | UInt | 426 | 1 | W | Write 1 to exit fault-locked state (only effective while in fault mode) |
| Internal command register | — | — | 427 | 1 | — | Reserved for internal firmware use; do not access |
| Reserved | — | — | 428 | 22 | — | |
| Internal command registers | — | — | 450 | 7 | — | Reserved for internal firmware use; do not access |
| Reserved | — | — | 457 | 3 | — | |
| Clear records | — | — | 460 | 1 | W | Write 0xAA to clear run and fault logs (effective when NOT in off-grid mode) |
| Reset user parameters | — | — | 461 | 1 | W | Write 0xAA to restore user parameters to factory defaults (effective when NOT in off-grid mode) |

---

### 3.3 Diagnostic / Log Registers (700–729)

| Name | Unit | Type | Start | Regs | R/W | Notes |
|------|------|------|-------|------|-----|-------|
| Fault log storage info [K] | — | ULong | 700 | 2 | R | High 16 bits = index of latest entry; low 16 bits = total number of fault entries |
| Fault log query index [L] | — | UInt | 702 | 1 | R/W | Set the index to query (0 … total count − 1) |
| Fault record [M] | — | — | 703 | 26 | R | See fault record format (manufacturer document) |
| Run log | — | — | 729 | 16 | R | See run log format (manufacturer document) |
| Reserved | — | — | 745 | 5 | — | |

---

## 4. Fault Code Table

32-bit fault code bitmask: bit N corresponds to fault code N+1.

| Fault Code | Description |
|-----------|-------------|
| 1  | Inverter over-temperature |
| 2  | DCDC over-temperature |
| 3  | Battery over-voltage |
| 4  | PV over-temperature |
| 5  | Output short circuit |
| 6  | Inverter over-voltage |
| 7  | Output overload |
| 8  | Bus over-voltage |
| 9  | Bus soft-start timeout |
| 10 | PV over-current |
| 11 | PV over-voltage |
| 12 | Battery over-current |
| 13 | Inverter over-current |
| 14 | Bus under-voltage |
| 15 | Reserved |
| 16 | Inverter DC component too high |
| 17 | Reserved |
| 18 | Output current zero-offset too large |
| 19 | Inverter current zero-offset too large |
| 20 | Battery current zero-offset too large |
| 21 | PV current zero-offset too large |
| 22 | Inverter under-voltage |
| 23 | Inverter negative-power protection |
| 24 | Parallel system master lost |
| 25 | Parallel system sync signal abnormal |
| 26 | Battery type incompatible (parallel) |
| 27 | Parallel firmware version incompatible |

---

## 5. Warning Code Table

System warnings are a 32-bit unsigned long. Each bit corresponds to one warning.
Warnings can be masked individually using the warning mask register [I] at address 314.
Masked warnings will not appear on the LCD or be returned by command.

| Bit   | Warning |
|-------|---------|
| bit 0  | Grid zero-crossing lost |
| bit 1  | Grid waveform abnormal |
| bit 2  | Grid over-voltage |
| bit 3  | Grid under-voltage |
| bit 4  | Grid over-frequency |
| bit 5  | Grid under-frequency |
| bit 6  | PV under-voltage |
| bit 7  | Over-temperature |
| bit 8  | Battery voltage low |
| bit 9  | Battery not connected |
| bit 10 | Overload |
| bit 11 | Battery EQ charging |
| bit 12 | Battery discharge low-voltage, not yet recovered to set point |
| bit 13 | Output power derated |
| bit 14 | Fan stall (blocked) |
| bit 15 | PV energy too low to use |
| bit 16 | Parallel communication interrupted |
| bit 17 | Single/parallel output mode inconsistent |
| bit 18 | Parallel battery voltage difference too large |
| bit 19 | Lithium BMS communication abnormal |
| bit 20 | Battery discharge current exceeds set value |
| bit 21–31 | Reserved |

---

## 6. Power Flow Flags

Register 231. 16-bit unsigned integer.

| Bits  | Meaning |
|-------|---------|
| bit 0–1 | 0=PV not connected; 1=PV connected to system |
| bit 2–3 | 0=Grid (mains) not connected; 1=Grid connected |
| bit 4–5 | 0=Battery idle; 1=Battery charging; 2=Battery discharging |
| bit 6–7 | 0=No load on output; 1=Load on output |
| bit 8   | 0=Grid not charging battery; 1=Grid charging battery |
| bit 9   | 0=PV not charging battery; 1=PV charging battery |
| bit 10  | 0=Battery icon lit; 1=Battery icon off |
| bit 11  | 0=PV icon lit; 1=PV icon off |
| bit 12  | 0=Grid icon lit; 1=Grid icon off |
| bit 13  | 0=Load icon lit; 1=Load icon off |

---

## 7. CRC-16/MODBUS Algorithm

Polynomial: **CRC-16/MODBUS** — x¹⁶ + x¹⁵ + x² + 1

Python equivalent (used in the driver):

```python
import struct

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def append_crc(frame: bytes) -> bytes:
    crc = crc16_modbus(frame)
    return frame + struct.pack('<H', crc)
```

The manufacturer's C implementation uses lookup tables (`auchCRCHi[]` / `auchCRCLo[]`);
both produce identical results.

---

*Translated from the original Chinese manufacturer document (版本：2025年06月 第1.0版).*
