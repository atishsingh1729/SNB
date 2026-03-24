# SmartNeckBand – Architecture Migration Guide

## New Directory Structure

```
smart_neckband/
├── CMakeLists.txt
├── main/
│   ├── main.c                          ← clean orchestrator
│   ├── CMakeLists.txt
│   ├── sensor_processing/
│   │   ├── sensor_processing.h/.c      ← magnitudes, variance, ROM, FFT resp
│   ├── data_pipeline/
│   │   ├── data_pipeline.h/.c          ← snapshot builder, JSON, CSV logger
│   ├── streaming/
│   │   ├── streaming.h/.c              ← BLE + UDP (all outbound transport)
│   └── app/
│       └── feature_processing/
│           ├── feature_processing.h/.c ← state machine (unchanged logic)
│           └── feature_processing_feed.c ← glue: raw→SensorData copy
│
└── components/
    ├── common/
    │   ├── config.h                    ← all pin/address/tuning #defines
    │   └── sensor_types.h             ← sensor_raw_t, sensor_processed_t
    │
    ├── hal/
    │   ├── adc/   hal_adc.h/.c        ← ADC unit + calibration
    │   ├── i2c/   hal_i2c.h/.c        ← I2C bus + device handles
    │   ├── spi/   hal_spi.h/.c        ← SPI bus + flash handle
    │   └── uart/  hal_gpio.h/.c       ← GPIO (LED, BNO RST/INT)
    │
    ├── middleware/
    │   ├── bno085/  mw_bno085.h/.c    ← BNO085 SHTP driver
    │   ├── max30102/ mw_max30102.h/.c ← MAX30102 HR algorithm
    │   └── ntc/     mw_ntc.h/.c      ← NTC temp + battery ADC
    │
    ├── raw_data_reader/
    │   └── raw_data_reader.h/.c       ← SWITCHABLE DATA SOURCE ← KEY FILE
    │
    └── file_reader/
        └── file_reader.h/.c           ← SPIFFS record + replay
```

---

## File Mapping (old → new)

| Old file | New file |
|---|---|
| `components/imu/bno085.c` | `components/middleware/bno085/mw_bno085.c` |
| `components/hr/max30102.c` | `components/middleware/max30102/mw_max30102.c` |
| `components/temperature/jirs40.c` | `components/middleware/ntc/mw_ntc.c` |
| `components/ble/ble.c` | `main/streaming/streaming.c` (BLE section) |
| `components/WiFi/wifi.c` | `main/streaming/streaming.c` (UDP section) |
| `main/struct_2_json.c` | `main/data_pipeline/data_pipeline.c` |
| `main/sensor_data_processing/data_processing.c` | `main/sensor_processing/sensor_processing.c` |
| `main/sensor_data_processing/signal_list.h` | `components/common/sensor_types.h` |
| `main/Feature_processing/feature_processing.c` | `main/app/feature_processing/feature_processing.c` |
| `components/common/config.h` | `components/common/config.h` (trimmed) |
| `main/peripherial_init.c` | Split into `hal_adc`, `hal_i2c`, `hal_spi`, `hal_gpio` |

---

## How to Switch Data Source

All switching happens in **3 lines at the top of `main/main.c`**:

```c
#define DATA_SOURCE    DATA_SRC_LIVE      // normal operation
#define OVERRIDE_MASK  0                  // all sensors live
#define INJECT_FILE_PATH  "/spiffs/dog01_run1.csv"
```

### Run with real sensors (normal)
```c
#define DATA_SOURCE    DATA_SRC_LIVE
#define OVERRIDE_MASK  0
```

### Replay a recorded dog session from file
```c
#define DATA_SOURCE    DATA_SRC_FILE
#define OVERRIDE_MASK  RAW_SRC_ALL        // replace all sensors
#define INJECT_FILE_PATH  "/spiffs/dog01_run1.csv"
```

### Inject only IMU from file, keep HR sensor live
```c
#define DATA_SOURCE    DATA_SRC_FILE
#define OVERRIDE_MASK  RAW_SRC_BNO085     // only BNO replaced
```

### Feed data from PC over UDP (port 6000)
```c
#define DATA_SOURCE    DATA_SRC_UDP
#define OVERRIDE_MASK  RAW_SRC_ALL
```
Send a binary `sensor_raw_t` struct as a UDP datagram to the device IP on port 6000.

### Static test values (unit test / demo)
```c
#define DATA_SOURCE    DATA_SRC_STATIC
#define OVERRIDE_MASK  RAW_SRC_ALL
// fill rdr_cfg.static_frame fields before calling raw_data_reader_init()
```

---

## How to Record a Dog Test Session

```c
// In main.c, before tasks start:
data_pipeline_start_recording("dog01_run1");

// ... dog wears the device, data is auto-saved per tick ...

// When done (e.g. triggered by button or timer):
data_pipeline_stop_recording();
```

The file is saved to `/spiffs/dog01_run1.csv`.  
Transfer it off the device using `idf.py monitor` + SPIFFS image extraction,
or add an HTTP endpoint to serve the file over WiFi.

---

## CSV Format

Each row written by `file_reader_write_frame()`:

```
ax,ay,az, gx,gy,gz, mx,my,mz, lax,lay,laz, roll,pitch,yaw,
steps, ir,hr,hrv, body_t,amb_t, batt_v,batt_pct
```

This is the exact format expected by `raw_data_reader` in `DATA_SRC_FILE` mode.

---

## Architecture Rules (for future development)

| Layer | Can call | Cannot call |
|---|---|---|
| HAL | hardware registers only | nothing above |
| Middleware | HAL only | sensor_processing, app |
| raw_data_reader | Middleware (LIVE) or file/UDP | app layer |
| sensor_processing | raw_data_reader output | app layer |
| data_pipeline | sensor_processing output | streaming directly |
| streaming | data_pipeline output | sensor logic |
| feature_processing (app) | sensor_processing output | streaming, HAL |
