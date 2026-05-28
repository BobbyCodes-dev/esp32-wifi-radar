# ESP32 WiFi Radar

Radar-style WiFi scanner for the **LilyGo T-Display (ESP32, 135x240)**. Sweeps nearby networks onto a radar display with color-coded signal blips and a live sidebar.

## Features

- Rotating sweep line with trailing fade effect
- Blips plotted at distance from center based on RSSI (-30 dBm = near, -90 dBm = edge)
- Color coding: **green** = strong (>-60), **yellow** = medium (-60 to -75), **red** = weak (<-75)
- Sidebar cycles through detected networks showing SSID, RSSI bar, dBm, and encryption type
- Rescan every 5 seconds; blips fade between hits

## Hardware

**LilyGo T-Display (ESP32)**
- Built-in 1.14" 135x240 ST7789 display
- No external wiring needed

> **Note:** If you have the T-Display-S3 (170x320), change `W`, `H`, `RADAR_R`, `RADAR_CX`, `RADAR_CY` in the `.ino` and update your TFT_eSPI `User_Setup_Select.h` to the S3 setup file.

## Library Dependencies

| Library | Install via |
|---------|-------------|
| TFT_eSPI by Bodmer | Arduino Library Manager or PlatformIO |

**TFT_eSPI setup (required):**

Open `~/.../TFT_eSPI/User_Setup_Select.h` and uncomment:
```cpp
#include <User_Setups/Setup25_TTGO_T_Display.h>
```
Comment out any other active `#include` in that file.

## Flashing

### Arduino IDE

1. Install **ESP32 board package** via Boards Manager (search `esp32` by Espressif)
2. Install **TFT_eSPI** via Library Manager
3. Configure TFT_eSPI as above
4. Select board: `TTGO T1` (or `ESP32 Dev Module`)
5. Set partition scheme: `Huge APP (3MB No OTA)` if available
6. Open `esp32-wifi-radar.ino`, click **Upload**

### PlatformIO

```bash
pio run --target upload
```

Uses `platformio.ini` in this repo (targets `ttgo-t1`).

## File Structure

```
esp32-wifi-radar/
├── esp32-wifi-radar.ino   # Main sketch
├── platformio.ini         # PlatformIO config
└── README.md
```

## Tuning

| Constant | Default | Description |
|----------|---------|-------------|
| `SWEEP_STEP_DEG` | 3 | Degrees per frame (higher = faster sweep) |
| `SCAN_INTERVAL_MS` | 5000 | WiFi rescan interval (ms) |
| `BLIP_TTL_TICKS` | 120 | How long a blip stays lit after sweep hit |
| `TRAIL_STEPS` | 8 | Number of trailing sweep lines |
| `MAX_NETWORKS` | 20 | Max networks tracked |
