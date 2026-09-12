# Experimental Apator AT-WMBUS-16-1 T2 programming

This document records the wire format reconstructed from inkaSOID 1.50.1. It
has a deterministic host-side test vector, but has not yet been confirmed with
an AT-WMBUS-16-1 overlay. Treat it as experimental.

## Transaction sequence

The command is prepared and armed first. The radio remains in T1 receive mode
until it receives an uplink whose link-layer ID matches the selected overlay.
It then switches to the T2 other-to-meter PHY and sends the command after the
minimum response delay. This timing requirement means that periodically sending
the command without observing the overlay first is not equivalent.

Programming is not considered successful merely because the SX1276 completed
TX. The implementation follows the inkaSOID transaction:

1. wait for the selected overlay's normal T1 uplink;
2. send the write request in its T2 response window;
3. return directly to T1 RX and validate the overlay's write acknowledgement;
4. wait for the overlay's next normal T1 uplink;
5. send a read request for register `0xB0`;
6. decrypt the response and compare all five periods with the requested value.

A read-only transaction is also available. It skips the write and ACK stages,
waits for the selected overlay's normal T1 uplink, sends the register `0xB0`
read request in the T2 response window and reports all five stored periods.
The read-only operation does not modify the overlay.

Timeouts retry the current stage on a later matching uplink. An explicit error
from the overlay and a readback mismatch end the transaction immediately.

## Period register

inkaSOID writes overlay register `0xB0` with five one-byte values:

1. normal period;
2. economy-hours period;
3. economy-weekday period;
4. economy-month-day period;
5. economy-month period.

Each byte is the number of ten-second units, so accepted values are 10 through
2550 seconds in ten-second steps. The ESPHome action currently writes the same
value to all five profiles.

The register data placed in the write command is:

```text
00 FF FF 00 B0 05 PP PP PP PP PP
```

where `PP = period_seconds / 10`.

## Frame construction

- Link-layer function: `REQ_UD2` (`0x5B`).
- Manufacturer: Apator (`APA`, `01 06` on air).
- Source address/version/type used by inkaSOID: `46 00 00 00 02 03`.
- Target address: four-byte little-endian BCD meter ID, followed by
  manufacturer, version and device type.
- Transport CI: `0x5B`; access number `0x01`; status `0x00`; AES-CBC mode 5.
- Cleartext starts with `2F 2F`, followed by the padded overlay write command.
- The command and data-link blocks use the EN 13757 CRC polynomial `0x3D65`.
- AES IV consists of target manufacturer, target ID, version, device type and
  eight bytes of `0x01`.
- Format-A CRCs are inserted before Manchester encoding.

The read request uses the same envelope with overlay instruction `0x01` and a
single data byte `B0`. The write instruction is `0x02`.

## ESPHome read-only action

The `wmbus_radio.apator_read_periods` action can be attached to a template
button, which is then exposed by the ESPHome web server:

```yaml
wmbus_radio:
  id: radio_component
  # existing radio configuration...
  on_apator_read_result:
    - logger.log:
        format: "Apator periods: normal=%u h=%u wd=%u md=%u m=%u"
        args:
          - normal_period
          - economy_hours_period
          - economy_weekday_period
          - economy_month_day_period
          - economy_month_period

button:
  - platform: template
    name: "Apator odczytaj bieżące okresy"
    on_press:
      - wmbus_radio.apator_read_periods:
          id: radio_component
          meter_id: "07208205"
          attempts: 3
          power_dbm: 10
```

After pressing the button, the action remains armed until the matching normal
uplink arrives. A successful callback has result `read`; failures report the
same radio or readback timeout strings used by the programming transaction.

## Responses

Both response types are received as T1 format-A frames. Every data-link CRC and
the Apator manufacturer/ID are checked before a response is accepted.

The write acknowledgement has C-field `0x00`. Its logical byte 20 encodes the
operation in the low nibble (`2` means write) and the error in bits 4..6:

| Error | Meaning recovered from inkaSOID |
| ---: | --- |
| 0 | OK |
| 1 | wrong PIN |
| 2 | wrong instruction code |
| 3 | wrong register number |
| 4 | wrong data amount |
| 5 | wrong command CRC |
| 6 | too many parameters |

The read response has C-field `0x08`. Its encrypted application data starts at
logical byte 15. The AES-CBC IV is the response M-field/A-field/version/type
(logical bytes 2..9), followed by eight copies of the access number from byte
11. The decrypted data starts `2F 2F 0F`; the `0xB0` value contains five bytes,
again in units of ten seconds.

After TX the SX1276 is restored to RX without the normal settling delays and
FIFO reset sequence. This is required because that sequence would erase or
miss the immediate response.

## T2 other-to-meter PHY

- frequency: 868.3 MHz;
- deviation: +/-50 kHz;
- chip rate: 32.768 kchip/s;
- Manchester mapping: zero to `10`, one to `01`;
- synchronization bytes configured in the SX1276: `54 76 96`.

## Deterministic test vector

For meter ID `12345678`, period 60 seconds, version `5`, device type `7` and an
all-zero key, the logical frame before data-link CRC insertion is:

```text
365b01064600000002035b785634120106050701002005cbc6edd25eb0132bf31d2828837b68d67d246fa1b70ae9d8ab34e0c1633b9b54
```

The CRC-bearing frame is:

```text
365b0106460000000203c42f5b785634120106050701002005cbc6ede119d25eb0132bf31d2828837b68d67d246f85dfa1b70ae9d8ab34e0c1633b9b54dfb2
```

The radio payload is 127 bytes after Manchester encoding and its eight-chip
postamble. `tests/apator_t2_vector_test.cpp` checks the complete payload.
