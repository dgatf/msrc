# MSRC Link Web

A web-based alternative for the Qt desktop configurator (`msrc_gui/`), built with vanilla JavaScript, the **Web Serial API**, and **PWA** offline support. Inspired by the Betaflight Configurator approach.

## Why

- No native app installation required — runs in Chrome/Edge (v89+).
- Works offline once loaded (Service Worker + cache).
- Identical binary protocol and `config_t` struct layout as the desktop GUI — `.cfg` files are interchangeable.

## Project structure

```
msrc_gui_web/
├── index.html              # Full UI (493 lines) — all widgets from the Qt GUI
├── css/
│   └── style.css           # Dark theme (387 lines)
├── js/
│   ├── config_struct.js    # Binary config_t serialization (308 lines)
│   ├── serial.js           # Web Serial API communication (168 lines)
│   ├── ui.js               # Bidirectional config↔UI mapping (534 lines)
│   ├── circuit.js          # Canvas-based circuit diagram (137 lines)
│   └── app.js              # Main orchestration (218 lines)
├── manifest.json           # PWA manifest
├── sw.js                   # Service Worker (cache-first)
├── res/                    # 19 PNG images (copied from msrc_gui/res/)
└── test/
    └── verify_layout.js    # Struct layout verification test
```

No build step, no bundler, no npm — just static files served by any HTTP server.

## How each file was built

### `js/config_struct.js` — Binary struct serialization

This is the most critical file. It replicates the exact memory layout of `config_t` from `include/shared.h`.

**Approach:**
1. Every field of `config_t` is listed in declaration order with its C type (`u8`, `u16`, `u32`, `f32`, `bool`, `i16`).
2. A `buildLayout()` function walks the fields applying **natural alignment rules** (matching GCC ARM / Clang defaults): each field is aligned to its own size boundary (1-byte for `u8`/`bool`, 2-byte for `u16`/`i16`, 4-byte for `u32`/`f32`).
3. Byte offsets are computed automatically — no hardcoded offsets.
4. `configEncode(obj)` → `Uint8Array` and `configDecode(Uint8Array)` → `obj` use `DataView` with **little-endian** byte order (matching ARM Cortex-M0+).
5. All enum types (`rx_protocol_t`, `esc_protocol_t`, etc.) are mirrored as JS constants with the same integer values.

**Verification:**
The computed layout was verified against the actual C++ compiled struct:

```sh
# Host: compiled include/shared.h with clang++ -std=c++11 (typed uint8_t enums)
# and printed offsetof() for 29 key fields.

# Docker: ran test/verify_layout.js
docker run --rm -v $(pwd):/app -w /app node:20-alpine node test/verify_layout.js

# Result:
# CONFIG_SIZE=224 expected=224 OK
# 29 passed, 0 failed          (offset checks)
# Roundtrip: 99 passed, 0 failed (encode→decode for all 99 fields)
```

The 29 checked offsets span the entire struct — from `version` at offset 0 through `spare20` at offset 220 — including all alignment-sensitive transitions (e.g. `bool` → `uint32_t` at `enable_gps` → `gps_baudrate`, `uint8_t` → `float` at `ina3221_filter` → `alpha_rpm`). The roundtrip test encodes the default config, decodes it, and compares every field.

### `js/serial.js` — Web Serial API

Implements the same USB CDC binary protocol as the Qt GUI (`board/project/usb.c`):

| Bytes sent | Direction | Meaning |
|---|---|---|
| `0x30 0x30` + 224 bytes | App → Board | Write config to flash |
| `0x30 0x31` | App → Board | Request current config |
| `0x30 0x32` + 224 bytes | Board → App | Config response |
| `0x30 0x33` | App → Board | Enable debug text stream |
| `0x30 0x34` | App → Board | Disable debug |
| `0x30 0x35` | App → Board | Force write default config |

Connection parameters: **115200 baud, 8N1** (matching `QSerialPort` in mainwindow.cpp).

The reader accumulates bytes and searches for the `[0x30, 0x32]` header pattern before extracting exactly `CONFIG_SIZE` bytes. In debug mode, raw bytes are decoded as UTF-8 text and forwarded to the debug panel.

### `js/ui.js` — Config ↔ UI mapping

Reimplements the two core functions from `mainwindow.cpp`:

- **`setUiFromConfig(cfg)`** — maps every `config_t` field to the corresponding DOM element. Ported from `MainWindow::setUiFromConfig()` (lines ~452–680 of mainwindow.cpp).
- **`getConfigFromUi()`** — reads all DOM widgets back into a config object. Ported from `MainWindow::getConfigFromUi()` (lines ~682–901).

All conversion formulas are preserved:

| Conversion | Formula |
|---|---|
| Averaging alpha ↔ UI elements | `alpha = 2 / (n + 1)`, `n = round(2/alpha - 1)` |
| Hall sensor sensitivity ↔ multiplier | `multiplier = 1000 / sensitivity_mV_A` |
| Shunt resistor | multiplier passed through directly |
| Airspeed VCC | config stores centivolts (`V × 100`), UI shows volts |
| HW V4/V5 ESC voltage/current multipliers | config stores raw float, UI shows `× 100000` |
| ESC combo → `esc_protocol_t` | `enum_value = combo_index + 1` (ESC_NONE = 0 = unchecked) |
| Barometer combo → `i2c_module_t` | `enum_value = combo_index + 1` (I2C_NONE = 0 = unchecked) |
| BMP280 filter | `config_value = combo_index + 1` |
| GPIO bitmask | bits 0–5 → GPIO 17–22 |

**Protocol-dependent visibility** (`updateVisibility()`) replicates `MainWindow::on_cbReceiver_currentTextChanged()`:

| Feature | Visible for protocols |
|---|---|
| Refresh rates | SmartPort, Frsky D, FPort, FBUS |
| Fuel meter | SmartPort, JetiEx, JetiEx Sensor, XBUS, HOTT, FPort, FBUS |
| Fuel pressure | SRXL, SRXL2, JetiEx, JetiEx Sensor, XBUS, HOTT |
| GPIO switches | SmartPort, FPort, FBUS |
| Airspeed | Hidden for Sanwa, GHST |
| GPS, Current, Vario | Hidden for Sanwa |
| LiPo (INA3221) | CRSF, SmartPort, FPort, FBUS, HOTT, SRXL, SRXL2, JetiEx, JetiEx Sensor |
| Averaging subsets | Sanwa: RPM+Volt+Temp only. GHST: Volt+Curr+Vario only |
| Serial Monitor | hides all sensor sections, shows baudrate/parity/stop bits/GPIO |

### `js/circuit.js` — Circuit diagram

Replicates `MainWindow::generateCircuit()` on an HTML5 Canvas:

1. Preloads all 18 PNG overlays from `res/`.
2. Draws the base board image (`rp2040_zero.png`) always.
3. Composites sensor overlays based on current UI state:
   - Each enabled sensor (voltage, current, NTC, airspeed, GPS) draws its overlay.
   - ESC overlays: `esc_rp2040_zero.png` for serial ESCs, `pwm_rp2040_zero.png` for PWM, `castle_rp2040_zero.png` for Castle Link, `smart_esc.png` for Smart ESC.
   - I2C bus: `vario_rp2040_zero.png` shared by vario, gyro, and lipo (same physical bus).
   - Receiver: `receiver_frsky_d_rp2040_zero.png` for Frsky D / CRSF / JetiEx Sensor (single-wire), `receiver_xbus_rp2040_zero.png` for XBUS (I2C), `receiver_hitec_rp2040_zero.png` for Hitec, `receiver_serial_rp2040_zero.png` for all others (UART).
   - XBUS + clock stretch → additional `clock_stretch_xbus_rp2040_zero.png`.
4. Zoom in/out buttons scale the canvas (0.5× – 3×).

The 19 PNG images in `res/` are copied verbatim from `msrc_gui/res/`.

### `js/app.js` — Application entry point

Orchestrates everything:

- **Connect/Disconnect** via Web Serial API (port selection dialog is browser-native).
- On connect: disables debug, waits 2 seconds (matching Qt GUI timing), then requests config.
- **Update button**: calls `getConfigFromUi()` → `configEncode()` → `serial.writeConfig()`.
- **Tab switching** between Configuration, Circuit, Debug.
- **Debug panel**: toggles debug mode, displays streaming text, auto-scroll toggle.
- **File Open/Save**: reads/writes raw `.cfg` binary files (just the `config_t` bytes — same format as Qt GUI's save/load).
- **Default Config**: sends `0x35` command after confirmation dialog.
- **PWA Service Worker** registration.

### `index.html` — UI structure

Every widget from the Qt GUI (`mainwindow.ui` / `mainwindow.cpp`) has a corresponding DOM element:

- **Receiver protocol** dropdown with all 19 protocols, each `<option value="N">` matching the `rx_protocol_t` enum value.
- **Protocol-specific options**: SmartPort sensor ID, XBUS clock stretch + alt packet, IBUS alt coordinates, Jeti GPS speed units, SBUS battery slot, FPort/FBUS inverted.
- **Serial Monitor**: GPIO, baudrate, stop bits, parity, timeout, inverted, format.
- **ESC**: 12 models in dropdown (values 1–12 = `esc_protocol_t`). Smart ESC consumption toggle, HW V4/V5 parameters (init delay, auto offset, voltage/current multipliers, auto detect, PWM out), RPM multipliers.
- **GPS**: protocol (UBLOX/NMEA), baudrate, rate.
- **Voltage**, **Temperature (NTC)**, **Current** (Hall / Shunt with all sub-options), **Airspeed**, **Vario** (3 barometer types + filter + auto offset), **Fuel Meter**, **Fuel Pressure** (K value dropdown), **GPIO** switches (17–22 + interval), **Gyro** (MPU6050 scales, weighting, filter), **LiPo** (INA3221 filter, cells).
- **Toggleable fieldsets**: checkbox in legend enables/disables the fieldset content — same UX as `QGroupBox::setCheckable(true)` in Qt.

### `css/style.css` — Dark theme

CSS custom properties for the color scheme (dark background, blue accents). Responsive two-column grid layout. Styled form controls matching the Betaflight Configurator aesthetic.

### `manifest.json` + `sw.js` — PWA offline

- `manifest.json`: standalone display mode, dark theme color, app icon.
- `sw.js`: on install, caches all 28 assets (HTML, CSS, 5 JS files, manifest, 19 PNGs). Cache-first strategy — the app works fully offline after first load.

## How to run

Serve the `msrc_gui_web/` folder over HTTP. Any static file server works:

```sh
cd msrc_gui_web
python3 -m http.server 8080
# Open http://localhost:8080 in Chrome or Edge
```

Or with Docker:
```sh
docker run --rm -v $(pwd)/msrc_gui_web:/usr/share/nginx/html:ro -p 8080:80 nginx:alpine
```

Then click **Connect**, select the RP2040 USB serial port, and the config will be loaded automatically.

## Browser requirements

| Platform | Browser | Transport | Status |
|----------|---------|-----------|--------|
| Windows / macOS / Linux / ChromeOS | Chrome 89+ / Edge 89+ | Web Serial API | Full support |
| Android | Chrome 61+ | WebUSB API (fallback) | Full support |
| iOS / Safari / Firefox | — | — | Not supported (banner shown) |

The app auto-detects the best available transport: **Web Serial API** on desktop, **WebUSB API** on Android. Both use the same binary protocol — no firmware changes required.

### WebUSB (Android) notes

On Android, the [Web Serial API is not available](https://caniuse.com/web-serial), so the app falls back to the **WebUSB API** to communicate with the RP2040 at the raw USB level (CDC ACM).

- The RP2040 is accessed via its default Pico SDK USB VID/PID (`0x2E8A` / `0x000A`).
- Line coding (115200/8N1) is configured via USB CDC control transfers (`SET_LINE_CODING`, `SET_CONTROL_LINE_STATE`).
- Data is exchanged over bulk IN/OUT endpoints on the CDC data interface.
- Android may require the user to grant USB permission when first connecting.

## Compatibility with msrc_gui

- `.cfg` files saved by the desktop GUI can be opened in the web app, and vice versa.
- The binary protocol is byte-identical — the web app talks to the same firmware with no changes required.
- The `config_t` struct layout (224 bytes, `CONFIG_VERSION = 2`) was verified to match the C++ compiled output by testing 29 struct field offsets and a full 99-field encode/decode roundtrip.
