# MSRC – Copilot Instructions

## Project Overview

MSRC (Multi Sensor for RC) is a telemetry system for RC models running on the **RP2040** microcontroller. It reads data from sensors (ESC, GPS, barometer, analog) and transmits it to an RC receiver using one of ~19 telemetry protocols. A companion **Qt5 desktop app** (`msrc_gui/`) configures the board over USB CDC. A Lua script (`lua/MSRC.lua`) enables wireless config from OpenTX/EdgeTX transmitters via SmartPort.

## Architecture

```
main.c  →  protocol_task (priority 3-4)  →  set_config()  →  sensor tasks (priority 2)
  ├── usb_task (priority 1)     GUI ↔ board binary protocol
  ├── led_task (priority 1)     WS2812 on GPIO 16
  └── dbg_task (priority 1)     Ring-buffer debug output
```

**Data flow:** Sensor tasks write to shared `float *` pointers (atomic on Cortex-M0+). Protocol tasks read these floats, format them per-protocol, and transmit to the receiver via UART/PIO. No mutexes are used for sensor data—relies on aligned 32-bit atomic writes.

**UART notification model:** ISRs push bytes into FreeRTOS queues, then after a packet-gap timeout call `vTaskNotifyGiveIndexedFromISR(context.uart0_notify_task_handle, 1, ...)`. Tasks block on `ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY)`.

## Key Directories

| Path | Purpose |
|------|---------|
| `board/project/protocol/` | 18 receiver protocol implementations (smartport, crsf, ibus, sbus, etc.) |
| `board/project/sensor/` | ~30 sensor drivers (ESC variants, GPS, baro, analog, fuel, GPIO) |
| `board/project/pio/` | PIO programs (.pio + .c/.h): soft UART, Castle Link, I2C, WS2812 |
| `board/project/` | Core firmware: main.c, config, UART, USB, LED, common utilities |
| `include/shared.h` | **Shared between firmware AND GUI** — all enums (`rx_protocol_t`, `esc_protocol_t`) and `config_t` struct |
| `msrc_gui/` | Qt5 desktop configuration app (MSRC Link) |
| `lua/MSRC.lua` | OpenTX/EdgeTX Lua script for wireless config via SmartPort |

## Protocol Communication Models

Protocols follow two models:

### Request/Response (SmartPort, IBUS, XBUS, Hitec, SBUS, SRXL, Multiplex, JetiEx, Sanwa, HOTT, JR DMSS)
The protocol task blocks on `ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY)` until the receiver sends a request. The task then parses the request, looks up the relevant sensor `float *` values, formats them per protocol, and sends a response.

### Periodic Push (CRSF, GHST, FBUS, FPort)
The protocol task sends telemetry frames at a fixed interval using `vTaskDelay()`, reading current sensor floats each cycle.

**SmartPort is special** — besides telemetry, it supports a maintenance/config mode for the Lua configuration script. Commands `0x30`/`0x31`/`0x32` get/set individual `config_t` fields by their SmartPort `data_id`.

## Adding a New Sensor

Follow the established pattern (see `board/project/sensor/esc_hw3.c` as reference):

1. **Header** — Define a parameter struct with config fields and `float *` output pointers:
   ```c
   typedef struct my_sensor_parameters_t {
       float alpha;
       float *value;  // allocated by protocol layer
   } my_sensor_parameters_t;
   ```
2. **Task function** — `void my_sensor_task(void *parameters)`:
   - Copy parameter struct locally: `my_sensor_parameters_t parameter = *(my_sensor_parameters_t *)parameters;`
   - Signal readiness: `xTaskNotifyGive(context.receiver_task_handle);`
   - Initialize output: `*parameter.value = 0;`
   - Loop: read hardware, apply `get_average()`, write to `*parameter.value`
3. **Wire in protocol** — In each protocol's `set_config()`, allocate `malloc(sizeof(float))` for the output pointer and create the sensor task with `xTaskCreate`. Wait for task init: `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)`.
4. **CMake** — Add the `.c` file to `board/project/sensor/CMakeLists.txt`.
5. **Stack size** — Define `STACK_MY_SENSOR` in `constants.h` as `(measured_min + STACK_EXTRA)`.

## Adding a New Protocol

Follow the pattern in `board/project/protocol/ibus.c`:

1. **`set_config()`** — Read `config_t`, create sensor tasks based on enabled sensors, build protocol-specific sensor list.
2. **Main task** — Init UART via `uart0_begin()`, call `set_config()`, enter loop (request/response or periodic push).
3. **Format function** — Convert `float *` sensor values to protocol wire format (handle byte order, scaling, units).
4. **Register in `main.c`** — Add `case RX_MY_PROTO:` to the switch, calling `xTaskCreate(my_proto_task, ...)`.
5. **Update enums** — Add to `rx_protocol_t` in `include/shared.h` (**both** `#ifdef __cplusplus` and C blocks must be updated in sync).
6. **Update GUI** — Add the protocol to `msrc_gui/mainwindow.cpp` combo box (`ui->cbReceiver->addItem(...)`) and handle protocol-specific widget visibility in `on_cbReceiver_currentTextChanged()`.

## Spektrum Smart ESC/BAT Sensor

The Smart ESC driver (`board/project/sensor/smart_esc.c`) is the most complex sensor. It **emulates a Spektrum SRXL2 receiver** to communicate with a Spektrum Smart ESC/Battery over UART1 at 115200 baud.

### How it works
1. **PWM capture** — Uses PIO (`capture_edge`, pio0) to read throttle (GPIO 12) and reverse (GPIO 13) PWM signals from the actual receiver.
2. **SRXL2 handshake** — On startup, performs SRXL2 handshake with the ESC (ID `0x40`), presenting itself as receiver ID `0x21`.
3. **Control packets** — Every 10ms (`SRXL2_INTERVAL_MS`), sends `SRXL2_PACKET_TYPE_CONTROL` packets forwarding captured throttle/reverse channels. Requests telemetry every 10th packet (`reply_id = esc_id`).
4. **Telemetry parsing** — Receives telemetry in XBUS format, parsing two device types:
   - **`XBUS_ESC_ID`**: RPM, voltage, current, FET temp, BEC voltage/current, BEC temp
   - **`XBUS_SMART_BAT` (`0x42`)**: Sub-types `0x00` (realtime: temp, current, consumption), `0x10`/`0x20`/`0x30` (individual cell voltages, up to 18), `0x80` (battery ID: cell count, cycles)

### Parameter struct (extensive)
```c
typedef struct smart_esc_parameters_t {
    bool calc_consumption;
    float rpm_multiplier;
    float alpha_rpm, alpha_voltage, alpha_current, alpha_temperature;
    float *rpm, *voltage, *current, *temperature_fet, *temperature_bec, *voltage_bec, *current_bec;
    float *temperature_bat, *current_bat, *consumption;
    float *cell[18];
    uint8_t *cells;
    uint16_t *cycles;
} smart_esc_parameters_t;
```

### Protocol wiring
In `set_config()`, Smart ESC is created at **priority 4** (higher than normal sensors) because it needs low-latency SRXL2 timing. All 18 cell pointers are individually `malloc`'d. All protocols support Smart ESC — the wiring pattern in `set_config()` allocates ~25 float pointers:
```c
if (config->esc_protocol == ESC_SMART) {
    smart_esc_parameters_t parameter;
    parameter.rpm = malloc(sizeof(float));
    // ... allocate all pointers ...
    for (uint i = 0; i < 18; i++) parameter.cell[i] = malloc(sizeof(float));
    parameter.cells = malloc(sizeof(uint8_t));
    parameter.cycles = malloc(sizeof(uint16_t));
    xTaskCreate(smart_esc_task, "smart_esc_task", STACK_SMART_ESC, (void *)&parameter, 4, &task_handle);
    context.uart1_notify_task_handle = task_handle;
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}
```

Config field: `config->smart_esc_calc_consumption` — if `true`, consumption is calculated from motor current via `get_consumption()` instead of using the Smart Battery's reported value.

## Config System

- `config_t` is defined in `include/shared.h` — shared across firmware, GUI, and Lua script.
- Stored in **RP2040 flash** at offset 768KB (`CONFIG_FLASH_TARGET_OFFSET`). Read via XIP pointer, written with interrupts disabled (`save_and_disable_interrupts()`).
- Supports migration from old flash offset (512KB → 768KB) in `config_read()`.
- Each field has a SmartPort `data_id` comment (0x5101–0x5152) used by the Lua config script.
- **When adding fields:** append before `spare` fields and decrement a spare. Bump `CONFIG_VERSION` in `config.h`. Update `config_forze_write()` defaults in `config.c`.
- Default values via compile-time `#define`s at top of `config.c` (e.g. `#define RX_PROTOCOL RX_SMARTPORT`).
- Defaults can be force-written holding GPIO 15 low at boot, or by `CONFIG_FORZE_WRITE`.

## USB Protocol (Board ↔ GUI)

Binary protocol over CDC USB serial. All messages start with header byte `0x30`:

| Bytes | Direction | Description |
|-------|-----------|-------------|
| `0x30 0x30` + `config_t` bytes | GUI→Board | Write config to flash (board blinks LED, writes, requires reset) |
| `0x30 0x31` | GUI→Board | Request current config |
| `0x30 0x32` + `config_t` bytes | Board→GUI | Config response |
| `0x30 0x33` | GUI→Board | Enable debug output (text stream over USB) |
| `0x30 0x34` | GUI→Board | Disable debug output |
| `0x30 0x35` | GUI→Board | Force write default config to flash |

The USB task (`board/project/usb.c`) polls every 1 second with `getchar_timeout_us(1000)`. Config is sent/received as raw `config_t` struct bytes — **both sides must agree on struct layout and `CONFIG_VERSION`**.

## GUI Application (MSRC Link)

Qt5/C++ desktop app in `msrc_gui/`. Key files:

| File | Purpose |
|------|---------|
| `mainwindow.cpp/.h` | Main application logic: serial comm, config UI, circuit diagram |
| `mainwindow.ui` | Qt Designer UI layout |
| `circuitdialog.cpp/.h` | Zoomable circuit diagram overlay dialog |
| `circuitdialog.ui` | Circuit dialog layout |
| `main.cpp` | App entry point |
| `msrc_gui.pro` | qmake project file |
| `res/` | PNG images for circuit diagram overlays (one per sensor/receiver type) |

### Architecture
- Uses `QSerialPort` at 115200/8N1 for USB CDC communication.
- Port list is auto-refreshed every 1 second via `QTimer`.
- On connect: disables debug, waits 2 seconds, requests config. Config arrives as `sizeof(config_t) + 2` bytes, validated by header `[0x30, 0x32]` and `config.version`.
- Two main flows: `setUiFromConfig()` (config struct → UI widgets) and `getConfigFromUi()` (UI widgets → config struct). These must be updated whenever `config_t` changes.
- **Circuit diagram**: `generateCircuit()` composites PNG overlays on a base RP2040 Zero pinout image depending on which sensors/receiver are enabled. Each sensor/receiver type has a dedicated PNG in `res/`.
- **Debug mode**: when enabled, all serial data is displayed as text in the debug panel (`ptDebug`).
- Config can be saved/loaded as `.cfg` binary files (raw `config_t` dump).
- Protocol-specific widgets are shown/hidden dynamically in `on_cbReceiver_currentTextChanged()` — e.g. fuel meter only for SmartPort/JetiEx/XBUS/HOTT/FPort/FBUS; GPIO only for SmartPort/FPort/FBUS.
- ESC combo maps directly to `esc_protocol_t` enum values (index + 1, since ESC_NONE = 0 means unchecked).

### When modifying the GUI
1. Update both `setUiFromConfig()` and `getConfigFromUi()` — they are the bidirectional mapping between `config_t` and UI.
2. For new receiver protocols: add to `cbReceiver` combo with the enum value as `userData`, update visibility logic in `on_cbReceiver_currentTextChanged()`.
3. For new sensors: add UI widgets in `mainwindow.ui`, update circuit diagram with a new PNG overlay.

## GPIO Pinout

| GPIO | Function |
|------|----------|
| 0 | UART0 TX → receiver RX |
| 1 | UART0 RX ← receiver TX |
| 2 | I2C1 SDA (receiver, with pull-up) |
| 3 | I2C1 SCL (receiver, with pull-up) |
| 4 | PWM capture / Castle PWM / UART1 TX (unused) |
| 5 | UART1 RX ← ESC TX / Castle ESC / Serial monitor |
| 6 | PIO UART RX ← GPS TX |
| 7 | Clock stretch NPN switch (XBUS) |
| 8 | I2C0 SDA (vario/sensors) |
| 9 | I2C0 SCL (vario/sensors) |
| 10 | PWM out |
| 11 | Fuel meter PWM capture |
| 12 | Smart ESC throttle PWM capture |
| 13 | Smart ESC reverse PWM capture |
| 14 | PIO UART TX → GPS RX |
| 15 | Restore default config (pull low at boot) |
| 16 | WS2812 LED |
| 17–22 | Digital GPIO switches |
| 26 | ADC0 — Voltage |
| 27 | ADC1 — Current |
| 28 | ADC2 — NTC temperature |
| 29 | ADC3 — Airspeed |

Assignments are in `board/project/constants.h` — **do not reuse pins** without checking this table.

## Lua Configuration Script

`lua/MSRC.lua` runs on OpenTX/EdgeTX transmitters for **wireless in-field configuration** via SmartPort telemetry. It reads/writes individual `config_t` fields by their `data_id` (0x5101–0x5152) using SmartPort maintenance mode (`0x30`/`0x31` frame commands). Pages: sensor ID, refresh rates, averaging, ESC config, GPS, vario, fuel meter, GPIO, analog sensors, gyro.

When adding a `config_t` field addressable via Lua: assign a `data_id` comment in `shared.h`, handle it in SmartPort maintenance mode (`smartport.c`), and add the corresponding entry in `MSRC.lua`.

## Build

```sh
# Firmware (requires Pico SDK, ARM toolchain)
cd board && mkdir build && cd build
cmake .. && make -j$(nproc)
# Output: MSRC-RP2040.uf2

# GUI (requires Qt5, including QtSerialPort)
cd msrc_gui && qmake && make
```

Version is derived from `git describe --tags`. The version string is injected as `PROJECT_VERSION` define in both firmware and GUI.

### Debugging with SWD probe
In `constants.h`, there are commented-out lines to remap UART from uart0 to uart1 for probe debugging. In `board/project/CMakeLists.txt`, toggle `pico_enable_stdio_usb` / `pico_enable_stdio_uart` (1/0 ↔ 0/1).

### Flashing
Hold BOOTSEL on the RP2040, connect USB, drag `MSRC-RP2040.uf2` to the mass storage device. Or use `picotool`.

## Code Conventions

- **C99** for firmware, **C++/Qt5** for GUI. No C++ in firmware code.
- Global `context_t context` in `main.c` holds all FreeRTOS task/queue handles—never create additional globals for IPC.
- Sensor averaging uses `get_average(alpha, prev, new)` (exponential moving average) from `common.c`. Alpha is stored in config as `float`, converted from user-facing "averaging elements" via `ALPHA(n) = 2.0 / (n + 1)`.
- Debug output via `debug(...)` macro—guarded by runtime `context.debug` flag, not compile-time. Can be toggled via USB commands.
- Timeout callbacks use `add_alarm_in_ms()` to zero sensor values on data loss.
- `#ifdef SIM_SENSORS` / `#ifdef SIM_RX` inject fake data for testing without hardware.
- PIO `.pio` files are compiled to headers via `pico_generate_pio_header()` in CMake.
- Byte order: use `swap_16()` / `swap_32()` macros from `common.h` for protocol wire formats.
- All structs used for wire protocols use `__attribute__((packed))`.

## FreeRTOS Priorities

| Priority | Tasks |
|----------|-------|
| 4 | SmartPort, FPort, FBUS, Smart ESC (latency-sensitive) |
| 3 | All other protocol tasks, timer service |
| 2 | All sensor tasks |
| 1 | USB, LED, debug |

Time slicing is **disabled** (`configUSE_TIME_SLICING 0`). Equal-priority tasks do not round-robin — each must yield explicitly. FreeRTOS tick rate is 500Hz (2ms). Task notification index 1 is used for UART data arrival (`configTASK_NOTIFICATION_ARRAY_ENTRIES = 3`).
