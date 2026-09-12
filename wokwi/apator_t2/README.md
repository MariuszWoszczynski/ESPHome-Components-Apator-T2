# SX1276 wM-Bus T2 model for Wokwi

This is a deterministic SX1276 test double for the Apator T2 programming path.
It is deliberately not an RF propagation simulator. It emulates the SX1276
registers, FIFO and DIO1 behavior used by `wmbus_radio`, captures outgoing T2
frames and injects a simulated meter uplink and configuration responses.

## Covered behavior

- SPI mode 0 and silicon version register `0x42 = 0x12`
- standby, FSK RX and FSK TX modes
- FIFO reads and streamed FIFO writes
- `FifoEmpty`, `FifoLevel`, `PayloadReady` and `PacketSent` flags
- DIO1 falling edge when receive data becomes available
- periodic T1-format meter telegram for meter `07208205`
- write ACK after the first T2 transmission
- AES-128-CBC register `0xB0` readback after the second transmission
- TX frame dump in the Wokwi Chips Console

The default test arms a 60-second period automatically one second after boot.
A successful complete transaction prints:

```
APATOR_WOKWI_RESULT=verified requested=60 read=60
```

## Configuration attributes

Set these under the custom chip's `attrs` object in `diagram.json`:

| Attribute | Default | Meaning |
|---|---:|---|
| `meterId` | `7208205` | Numeric meter ID; leading zeroes are added as BCD |
| `version` | `5` | wM-Bus device version byte |
| `deviceType` | `7` | wM-Bus device type byte |
| `periodSeconds` | `60` | Value returned during register `0xB0` readback |
| `uplinkIntervalMs` | `3000` | Interval between simulated meter uplinks |
| `responseMode` | `0` | `0`: success, `1`: timeout, `2`: reject register |

The model currently assumes the all-zero AES-128 key used by the test YAML.

## Running

1. Compile `apator_t2_wokwi.yaml` from the repository root.
2. Copy `wokwi.toml.example` to `wokwi.toml` and adjust the two build paths if
   your ESPHome installation uses a different build directory.
3. Open this directory with Wokwi for VS Code and start the simulation.
4. Watch both the serial terminal and the Chips Console.

For the browser editor, upload `diagram.json`, `sx1276.chip.json` and
`sx1276.chip.c`, then upload the merged ESP32 firmware produced by ESPHome.

## Limits

The model validates firmware control flow and exact SPI/FIFO interaction. It
does not validate RF power, frequency error, antenna behavior, sensitivity or
compatibility with a physical Apator overlay.
