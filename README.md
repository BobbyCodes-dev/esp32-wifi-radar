# ESP32 WiFi Motion Radar

RSSI-based WiFi presence/motion detector for the **LilyGo T-Display (ESP32, 135x240)**. Connects to your home router, baselines the signal, and flags when something in the environment disturbs it — people walking past, entering a room, opening doors.

## How It Works

The ESP32 stays connected to your home router and reads the signal strength (RSSI) every 100ms. It keeps a rolling buffer of 50 samples, calculates the mean and standard deviation, and flags a motion event when the current RSSI deviates more than `SENSITIVITY` standard deviations from the mean. Radio waves bounce off people — even walking through a room shifts the multipath reflections enough to move the RSSI by several dBm.

**No extra hardware. No second ESP32. Just your existing router.**

## Display Layout

```
+------------------------------------------+
|  [RSSI oscilloscope waveform — top half] |
|  Current dBm label    --------RSSI       |
|  Cyan mean reference line                |
+------------------------------------------+  <- divider
|  [MOTION] or [CLEAR]   [confidence bar]  |
|  Events: N   Last: HH:MM:SS              |
|  mean -65.2  sd 1.43                     |
+------------------------------------------+
```

- **Top half:** live scrolling RSSI waveform (oscilloscope style, older = dimmer green, newest = bright green)
- **Cyan line:** rolling mean — the baseline the detector uses
- **Motion badge:** red `MOTION` or green `CLEAR`, holds for 3 seconds after last trigger
- **Confidence bar:** shows how close current signal is to the trigger threshold (orange/red = near trigger)
- **Stats:** event count, last event timestamp (HH:MM:SS since boot), mean and stddev

## Serial Output

Every motion event is logged over USB Serial (115200 baud):

```
[BOOT] WiFi Motion Radar running. SSID=MyNetwork  Sensitivity=2.0
[MOTION] Event #1 at 00:02:14  RSSI=-68  mean=-64.8  stddev=1.43  dev=3.20
[MOTION] Event #2 at 00:02:31  RSSI=-71  mean=-64.9  stddev=1.51  dev=6.10
```

## Setup — What to Fill In Before Flashing

Open `esp32-wifi-radar.ino` and edit these three lines at the top:

```cpp
#define WIFI_SSID       "YOUR_SSID_HERE"      // your router's network name
#define WIFI_PASSWORD   "YOUR_PASSWORD_HERE"  // your WiFi password
#define SENSITIVITY     2.0f                  // motion threshold (see below)
```

That's it. Flash and go.

## Sensitivity Tuning

| `SENSITIVITY` | Behavior |
|---------------|----------|
| `1.5` | Very sensitive — detects distant/subtle motion, more false positives |
| `2.0` | **Recommended default** — good for small-home room-to-room detection |
| `2.5` | Less sensitive — only catches significant disturbances |
| `3.0` | High threshold — dramatic movement only (walking close to the ESP32) |

**Start at `2.0`.** If you're getting too many false alerts from AC, appliances, or neighbors, increase to `2.5`. If it's missing real motion, drop to `1.5`.

The `MIN_STDDEV` constant (default `0.5`) acts as a noise floor — if the signal is dead flat, it won't trigger regardless. This prevents false positives on perfectly quiet RF environments.

## Placement Tips

- **Best placement:** between two rooms or near a doorway — motion disturbs the signal path between the ESP32 and router most when it crosses that path
- **Router line-of-sight:** the more direct the path from ESP32 to router, the more sensitive the detection
- **Keep away from:** microwaves (2.4GHz interference), metal surfaces, fish tanks
- **2.4GHz vs 5GHz:** RSSI-based sensing works on 2.4GHz only (longer range, more reflections). Connect to your 2.4GHz network.

## Flashing

### PlatformIO (recommended)

```bash
pio run --target upload
pio device monitor  # to see serial motion logs
```

### Arduino IDE

1. Install **ESP32 board package** via Boards Manager (search `esp32` by Espressif)
2. Install **TFT_eSPI** via Library Manager
3. Select board: `TTGO T1` or `ESP32 Dev Module`
4. Set partition scheme: `Huge APP (3MB No OTA)`
5. Upload `esp32-wifi-radar.ino`

> **TFT_eSPI note (Arduino IDE only):** If display doesn't work, open `~/.../TFT_eSPI/User_Setup_Select.h` and uncomment `#include <User_Setups/Setup25_TTGO_T_Display.h>`. PlatformIO handles this automatically via `build_flags`.

## Hardware

**LilyGo T-Display (ESP32)** — no external wiring needed.

| Spec | Value |
|------|-------|
| Display | 1.14" 135x240 ST7789 |
| MCU | ESP32 (240MHz dual-core) |
| WiFi | 802.11 b/g/n 2.4GHz |
| Flash | 4MB / 16MB depending on variant |

## Constants Reference

| Constant | Default | Description |
|----------|---------|-------------|
| `WIFI_SSID` | `YOUR_SSID_HERE` | **Fill in** — your network name |
| `WIFI_PASSWORD` | `YOUR_PASSWORD_HERE` | **Fill in** — your WiFi password |
| `SENSITIVITY` | `2.0` | Stddev multiplier for motion trigger |
| `SAMPLE_INTERVAL_MS` | `100` | RSSI sample rate (ms) |
| `BUFFER_SIZE` | `50` | Rolling window size (50 samples = 5 seconds) |
| `MOTION_HOLD_MS` | `3000` | How long MOTION badge stays red after event (ms) |
| `MIN_STDDEV` | `0.5` | Noise floor — below this stddev, no triggers fire |

## Limitations

- RSSI sensing is probabilistic, not deterministic — it detects disturbance, not direction or count
- Works best for detecting "someone is in this area" vs. "no one is here"
- Large metal objects, appliances, and other WiFi devices can cause false positives
- Detection range depends on your home's construction and router placement
- 5GHz networks are not suitable — use 2.4GHz

## File Structure

```
esp32-wifi-radar/
├── esp32-wifi-radar.ino   # Main sketch
├── platformio.ini         # PlatformIO config
└── README.md
```
