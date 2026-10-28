# SpoolSense

A small round touchscreen that sits on the side of a Snapmaker U1 and shows how much filament is left on a spool.
A load cell under the spool feeds an HX711; an ESP32-S3 with a 1.28" round touch display shows the weight, the
toolhead, the material and a color swatch, and warns you when a spool runs low.

This is an early proof of concept: one load cell drives the display for the selected toolhead (T1-T16).
Nothing talks to the printer yet.

## What it does

- **Live weight** in grams from an HX711 load cell, with a configurable rounding step (1, 5, 10 or 50 g) and a
  spool-size setting (250 g to 2 kg). A ring around the edge fills as the spool empties.
- **Low-spool warnings**: the ring and number turn yellow, orange and red at thresholds you choose (5% steps).
- **Four swipeable pages**: weight, tare / set weight, material, color picker (hue, saturation, brightness, with
  HEX / RGB / HSV readout).
- **Materials** for each of 16 toolheads from a list of 20 filament types; you choose which types appear.
- **Settings** (tap the top of the screen): spool size, calibration, refresh rate, weight step, brightness,
  dim and sleep timers, toolhead, filament list, startup page, alert levels, and a QR code for questions.
- Settings are stored in flash and survive a restart.
- Two-step tare (tap TARE, then CONFIRM) so you do not zero the scale by accident.

## Hardware

- Waveshare ESP32-S3-Touch-LCD-1.28 (240x240 round display, CST816S touch)
- HX711 load cell amplifier and a load cell sized for a spool (a 1 kg-class bar cell was used)

Wiring to the board's breakout connector:

| HX711 | ESP32-S3 |
|-------|----------|
| DT / DOUT | GPIO 18 |
| SCK / CLK | GPIO 21 |
| VCC | 3V3 |
| GND | GND |

The pins are `HX711_DOUT` and `HX711_SCK` near the top of `SpoolSense/SpoolSense.ino`.

## Build

1. Install the Arduino IDE with the ESP32 board package and select an ESP32-S3 board with PSRAM enabled
   (the sketch uses it for the frame buffer when present).
2. Install the **HX711** library (Rob Tillaart, tested with 0.6.3) from the Library Manager.
3. Open `SpoolSense/SpoolSense.ino` and upload. The display, touch and IMU drivers are included in the folder.
4. Open the Serial Monitor at 115200 baud. Debug output goes to `Serial0` on this board.

## Calibrate

`hx711CalibrationFactor` is a placeholder (-350). To set it:

1. Power up with the scale empty (it tares once at startup), then place a known weight (1000 g by default,
   `HX711_KNOWN_CAL_WEIGHT_GRAMS`).
2. Read the `[CAL]` lines in the Serial Monitor; they print a suggested factor for the current load.
3. Put that value in `hx711CalibrationFactor` and upload again. If the weight reads negative, flip the sign or
   swap the load cell's A+ / A- wires.

## Status and plans

- One physical load cell only; the other toolheads show the same reading.
- No connection to the printer yet. The goal is to read per-toolhead spool weights and show them next to the
  filament the printer reports (for example through Moonraker/Spoolman).
- The calibration page has a placeholder for a guided known-weight calibration.

## License

MIT for the SpoolSense code. The Waveshare driver files keep their original licenses (see `LICENSE`).
