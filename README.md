# EMOS E0042 433 MHz decoder

Unofficial reverse-engineered decoder for the EMOS E0042 wireless temperature sensor.

## Confirmed radio/protocol details

- Frequency: 433.920 MHz
- Modulation: OOK / ASK
- Base timing: about 1955 us
- Physical symbols observed with CC1101 asynchronous RX:
  - `SS` (HIGH short + LOW short) = `0`
  - `SL` (HIGH short + LOW long) = `1`
- Logical frame length: 46 bits
- The frame is repeated about 3 times per transmission.
- Stable 20-bit frame header observed: `01111101010101010010`
- Temperature field: transmitted bits 20..31 (12 bits), reversed before conversion.
- Temperature conversion:

```text
raw = reverse_bits_12(frame[20..31])
temperature_C = raw / 40.0 - 40.0
```

Calibration examples observed during reverse engineering:

| raw | temperature |
|---:|---:|
| 2504 | 22.6 °C |
| 2500 | 22.5 °C |
| 2496 | 22.4 °C |
| 2492 | 22.3 °C |
| 2484 | 22.1 °C |

Bits 32..43 form a second related 12-bit field. In the tested unit the transmitted form was consistently related to the first field by an XOR pattern; this is retained as an observation rather than claimed as a universal device-ID rule until more sensors are tested.

Tail bits observed: `01`.

## Reference hardware

Receiver used during reverse engineering:

- ESP32-C3 SuperMini
- E07-M1101D / CC1101

Wiring used by the reference sketch:

| E07-M1101D | ESP32-C3 |
|---|---:|
| GDO2 | GPIO1 |
| GDO0 | GPIO3 |
| SCK | GPIO4 |
| MISO | GPIO5 |
| MOSI | GPIO6 |
| CS | GPIO10 |
| VCC | 3.3 V |
| GND | GND |

## Firmware

`emos_e0042_v1_0.ino` is the production-oriented reference receiver. It prints a concise human-readable result and a JSON line suitable for gateways/integration.

Example:

```json
{"model":"EMOS E0042","temperature_C":22.1,"rssi":-51.0,"quality":100.0,"raw":2484,"sync_errors":0,"signature":"01111101010101010010"}
```

## Notes

This project is independent and unofficial. EMOS is a trademark of its respective owner. No Wi-Fi credentials, passwords, tokens, addresses, or other private deployment details are included in this repository.
