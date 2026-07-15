# Telemetry 2026 Software Architecture

## Purpose

The telemetry software provides vehicle observability for the Formula Student car. It collects local front and rear sensor data, records telemetry to an SD card, publishes selected telemetry over MQTT, and displays critical driver information on the front OLED.

The current telemetry system is not designed to directly actuate safety-critical functions. The telemetry modules are robust observers and gateways: failures in logging, display, GPS, or cloud upload must not block sensor acquisition or CAN reception.

## High-Level Architecture

```mermaid
flowchart LR
    subgraph Vehicle["Vehicle System"]
        CAN["500 kbit/s CAN bus<br/>Shared vehicle data interface"]
        FrontSensors["Front sensors<br/>FL/FR dampers, steering, MPU6050"]
        RearSensors["Rear sensors<br/>RL/RR dampers, brake pressure, gear switches"]
        DriverDisplay["OLED driver display<br/>Gear, lap, GPS speed, status"]
        SD["SD card logger<br/>CSV CAN log"]
    end

    subgraph Front["Front-Module ESP32"]
        FLoop["25 Hz main loop<br/>front analog + IMU sampling"]
        FCANRX["CAN_Task<br/>high-priority CAN receive"]
        FSD["SD_Task<br/>queued batch logging"]
        FOLED["10 Hz display update"]
        FQueue["canQueue<br/>LogMessage buffer"]
    end

    subgraph Rear["Rear-Module ESP32 / A7670"]
        RSensors["Sensor_Task<br/>25 Hz rear/brake/gear sampling"]
        RCANRX["CAN_RX_Task<br/>filtered front CAN input"]
        RModem["MQTT_Task<br/>LTE state machine + 5 Hz batches"]
        RQueue["Policy buffers<br/>queues + latest-value slots"]
        RGPS["GPS_Task<br/>1/2/5 Hz GNSS acquisition"]
        Mutex["Modem mutex<br/>one TinyGSM/TCP/MQTT owner"]
        GPS["GNSS receiver<br/>position + speed"]
    end

    subgraph Cloud["Remote Telemetry"]
        Broker["MQTT broker<br/>broker.hivemq.com / tuiracing"]
        Viewer["visual-telemetry Python dashboard<br/>Tkinter + paho-mqtt"]
    end

    FrontSensors --> FLoop
    FLoop -->|CAN_ID_FRONT_ANALOG 0x500<br/>CAN_ID_ACCEL 0x501| CAN
    CAN --> FCANRX
    FCANRX --> FQueue
    FLoop --> FQueue
    FQueue --> FSD --> SD
    FCANRX --> FOLED --> DriverDisplay

    RearSensors --> RSensors
    RSensors -->|CAN_ID_REAR_ANALOG 0x701<br/>CAN_ID_GEAR 0x700| CAN
    RSensors --> RQueue
    GPS --> RGPS
    RGPS -->|CAN_ID_GPS_POS 0x750<br/>CAN_ID_GPS_SPD 0x751| CAN
    RGPS --> RQueue
    RGPS --> Mutex
    RModem --> Mutex
    CAN --> RCANRX --> RQueue
    RQueue --> RModem --> Broker --> Viewer
```

## Main Software Modules

### Front-Module firmware

Location: `Front-Module/`

- `src/main.cpp`: initializes the front module and runs the 25 Hz main sampling loop.
- `src/setup.cpp`: initializes GPIO, SD card, OLED display, MPU6050 IMU, and ESP32 TWAI CAN.
- `src/loop.cpp`: implements CAN transmission, CAN reception, SD logging, and display update logic.
- `include/canIDs.h`: defines the shared CAN message identifiers used by both front and rear modules.
- `include/pinout.h`: defines the hardware interface pins for sensors, SD, OLED, IMU, and CAN.

Responsibilities:

- Samples front damper and steering analog channels at 25 Hz.
- Samples MPU6050 acceleration data and broadcasts it on CAN.
- Receives all CAN traffic and logs it through a FreeRTOS queue.
- Updates the driver-facing OLED display at 10 Hz.
- Stores CAN telemetry to an SD card in CSV format with session-based filenames.

### Rear-Module firmware

Location: `Rear-Module/`

- `src/main.cpp`: initializes queues, hardware, modem, MQTT, CAN, and FreeRTOS tasks.
- `src/setup.cpp`: initializes GPIO, LTE modem, MQTT client, GNSS, and ESP32 TWAI CAN.
- `src/loop.cpp`: implements rear sensor sampling, gear reading, GPS parsing, CAN receive, and MQTT upload.
- `include/canIDs.h`: mirrors the shared CAN message identifiers.
- `include/pinout.h`: defines modem, CAN, analog sensor, and gear input pins.

Responsibilities:

- Samples rear dampers, brake pressure, and gear switches at 25 Hz.
- Receives CAN traffic from the rest of the vehicle.
- Publishes telemetry to MQTT using LTE.
- Reads atomic GNSS position/speed samples in a dedicated priority-3 task at the best supported 5, 2, or 1 Hz rate.
- Re-broadcasts GPS data onto CAN for local logging and display consumers.
- Runs LTE registration, PDP activation, MQTT connection, and reconnect as a priority-4 state machine.
- Serializes all post-start modem, TCP, and MQTT calls with one mutex and reserves the final 50 ms before each GPS deadline.
- Publishes one policy-ordered MQTT batch every 200 ms without replaying continuous outage history.

### Visual telemetry dashboard

Location: `visual-telemetry/`

- `main.py`: Tkinter dashboard that subscribes to the MQTT topic, decodes CAN-style payloads, and displays live vehicle data.

Responsibilities:

- Connects to `broker.hivemq.com` on topic `tuiracing`.
- Parses messages in `timestamp,id,data` format.
- Decodes known MQTT records into gear, acceleration, gyroscope, GPS position/speed, lap time, and module health.
- Highlights stale data for selected critical values.

## Interfaces

### CAN bus

The CAN bus is the primary in-vehicle telemetry interface. Both ESP32 modules use the native TWAI driver at 500 kbit/s.

| CAN ID | Source | Data |
| --- | --- | --- |
| `0x500` | Front module | Front damper 1, front damper 2, steering |
| `0x501` | Front module | Accelerometer X/Y/Z |
| `0x502` | Front module | Gyroscope X/Y/Z |
| `0x700` | Rear module | Gear |
| `0x701` | Rear module | Rear left damper, rear right damper, brake pressure |
| `0x750` | Rear module | GPS latitude and longitude |
| `0x751` | Rear module | GPS speed in km/h |
| `0x777` | Rear module | Lap time in milliseconds |
| `0x7FF` | Front/rear modules | System health |

### MQTT telemetry

Each MQTT record uses the same payload format as the SD logger:

```text
timestamp,id,data
```

Example:

```text
123456,701,03FF0400012C
```

This format keeps cloud telemetry aligned with CAN and SD logging, making debug traces easier to compare.
One MQTT publication is attempted every 200 ms. Records are separated by semicolons and ordered as lap, gear, GPS, health, then IMU. PubSubClient uses a 512-byte packet buffer and the firmware limits its payload buffer to 480 bytes. The dashboard decodes every record in the batch.

| Signal | Local acquisition / CAN | MQTT policy |
| --- | ---: | --- |
| Front analog `0x500` | 25 Hz | Not published |
| Accelerometer `0x501` | 25 Hz | 5 Hz latest value |
| Gyroscope `0x502` | 25 Hz | 5 Hz latest value |
| Gear `0x700` | 25 Hz | Debounced transitions plus 1 Hz refresh |
| Rear analog `0x701` | 25 Hz | Not published |
| GPS `0x750/0x751` | Best supported 5/2/1 Hz | Every successful atomic sample while connected |
| Lap `0x777` | Completed lap | Persistent latest value until publish succeeds |
| Health `0x7FF` | 0.2 Hz per node | Latest value keyed by node |

The fixed policy buffers contain two atomic GPS samples and four gear transitions. IMU and health use generation-protected latest-value slots. A disconnect clears continuous queues but retains current state and the latest lap; reconnect publishes one fresh snapshot. IDs `0x500` and `0x701` remain available on CAN for local consumers and SD logging but never enter MQTT buffering.

### SD logging

The front module logs CAN messages to session-specific CSV files on the SD card. The log includes timestamp, CAN ID, and raw data bytes encoded as hexadecimal. Batching and periodic flushes are used to reduce SD write overhead while still preserving data during idle periods.

## Robustness And Functional Safety Approach

### Separation of concerns

The architecture separates time-sensitive vehicle data handling from slower or failure-prone peripherals:

- Sensor sampling and CAN reception are independent FreeRTOS tasks or timed loops.
- SD card writes are handled through `canQueue` instead of being performed directly in the CAN receive path.
- GPS acquisition and LTE/MQTT processing run in separate priority-3 and priority-4 rear tasks.
- Driver display updates run at 10 Hz instead of blocking the 25 Hz acquisition loop.

This prevents slow storage, display, network, or GPS operations from directly blocking the acquisition of vehicle state data.

### Deterministic timing

The front module uses a 25 Hz loop period for front analog/IMU sampling and a separate 10 Hz display period. The rear module uses a 25 Hz `Sensor_Task` and a deadline-driven `GPS_Task`, both with `vTaskDelayUntil`. MQTT cannot begin routine work within 50 ms of the next GPS deadline. This creates predictable acquisition timing for chassis, brake, and GNSS telemetry.

### Queue-based buffering

Both modules use bounded buffering:

- `canQueue` buffers messages for SD logging on the front module.
- The rear telemetry policy uses small signal-specific queues plus latest-value slots instead of a single FIFO.

This reduces coupling between producers and consumers. Queue overflow drops the oldest continuous value and increments diagnostics. Intentional rate reduction and latest-value coalescing are not reported as loss.

### Failure containment

The system is designed to continue operating with partial functionality:

- If the modem or network is unavailable, local CAN and sensor functionality continue.
- If MQTT is disconnected, the rear module retries without stopping the sensor task.
- If the SD card fails to initialize, the front module still initializes display, IMU, and CAN.
- If GNSS has no valid fix, invalid GPS frames are not broadcast.
- Display status flags indicate stale rear-module or GPS data.

### Traceability and diagnostics

The SD logger records raw CAN traffic in timestamped CSV form, making it possible to reconstruct vehicle data after a run. The visual dashboard also reports packet counts and marks stale data, helping the team detect missing or delayed telemetry during testing.

### Current safety boundary

The telemetry project currently observes and distributes state; it does not directly actuate throttle, brakes, steering, or shutdown controls. Safety-critical control functions should remain in dedicated vehicle systems with their own validation, plausibility checks, and fail-safe outputs. Telemetry data can support diagnostics and driver awareness, but it should not be the sole source of a safety decision unless additional safety mechanisms are added and validated.

## Suggested Answer For Documentation

Our approach to software functional safety and robustness is based on separating safety-critical control from telemetry and then ensuring the telemetry software cannot block or destabilize vehicle state acquisition. The telemetry architecture is distributed across two ESP32-based modules connected to the vehicle CAN bus. The front module samples front chassis sensors and the IMU, receives CAN traffic, logs it to an SD card, and updates the driver OLED. The rear module samples rear suspension, brake pressure, gear position, and GPS, and bridges selected telemetry to an MQTT link for remote visualization.

Time-critical functions are isolated from slow peripherals using FreeRTOS tasks, fixed-rate loops, and bounded policy buffers. Sensor acquisition runs at 25 Hz, CAN reception is handled in dedicated high-priority tasks, GPS has its own deadline-driven task, and LTE/MQTT runs below it. The front module logs through `canQueue`; the rear module rate-limits only MQTT while retaining full-rate CAN traffic.

The software also supports graceful degradation. If the LTE modem, MQTT broker, GPS fix, or SD card is unavailable, the remaining local telemetry functions continue. Raw CAN data is stored or transmitted in a consistent `timestamp,id,data` format, which improves traceability and allows post-run verification. The driver display shows the most relevant vehicle state values and can indicate missing subsystems.

At the current stage, telemetry is treated as an observer and diagnostics layer rather than an actuator for safety-critical control. Dedicated vehicle safety systems retain authority over critical control functions. This boundary reduces risk: a telemetry failure should affect visibility or logging, not core vehicle control. Future safety improvements should add explicit signal plausibility checks, watchdog reporting, and hardware validation for startup failure, communication loss, and sensor fault scenarios.
