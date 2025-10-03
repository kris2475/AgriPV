# ESP32 Multi-Sensor MPPT & Data Logging System

This sketch integrates **MPPT control for an Organic Photovoltaic (OpV) panel**, multiple **environmental and optical sensors**, **SD card logging**, and an **OLED display**. Special attention is given to MPPT behavior under low-light conditions.

---

## 1. Sensors and Devices

### C12880MA Spectrometer
- Reads spectral data (288 channels).
- Supports **adaptive integration time** to prevent saturation and optimize signal.
- Optional **white LED and 405nm laser** for illumination/experiments.

### INA226 Power Sensor
- Measures **voltage, current, and power** from the OpV panel.
- Configured with shunt resistor calibration and continuous measurement mode.

### BME680 Environmental Sensor
- Measures **temperature, pressure, humidity, and gas resistance**.
- Oversampling and gas heater configured for stability.

### GY-30 (BH1750) Illuminance Sensor
- Measures **ambient light (lux)**.
- Used to conditionally enable MPPT and logging.

---

## 2. Maximum Power Point Tracking (MPPT)
- Uses **Perturb & Observe (P&O)** algorithm to maximize power output.
- **MPPT Fix Features:**
  - **Pause at Night / Low Light:** When lux < `LUX_MPPT_THRESHOLD` (50 lux), MPPT is disabled; PWM duty cycle set to `DUTY_PARK`.
  - **Re-enable at Daybreak:** MPPT state variables (`V_old`, `I_old`, `P_old`) are reset when lux rises above the threshold.
  - **Anti-windup:** Duty cycle constrained to avoid hitting extreme min/max values.

- **PWM Control:**
  - Pin: `PWM_PIN` (GPIO 15)
  - Frequency: 1 kHz
  - Resolution: 12-bit (0–4095)

- **Duty Cycle Adjustment:** 
  - `DUTY_CYCLE_STEP` = 50
  - Perturb & Observe logic ensures proper increment/decrement based on power change.

---

## 3. SD Card Data Logging
- Saves **all sensor readings** (MPPT, BME680, GY-30, spectrometer) into **one CSV file**: `/SENSOR_LOG.CSV`.
- **Conditional Logging:** Only writes data if lux > 50, reducing unnecessary writes.
- **Timestamping:** 
  - Uses **NTP via WiFi** for accurate time.
  - Falls back to `millis()` with `UNSYNCED` label if NTP fails.
- **CSV Columns Include:**
  - Timestamp
  - MPPT data (V, I, P, duty cycle)
  - BME680 readings
  - GY-30 lux
  - Spectrometer saturation status, integration time, and 288 pixel values

---

## 4. OLED Display (128x64 SSD1306)
- Cycles through **4 pages** every 5 seconds:
  1. **Spectrometer Stats**: max pixel, integration time, voltage, SD writes, saturation
  2. **Spectrograph Plot**: graphical representation of 288 channels
  3. **MPPT Stats**: V, I, P, duty cycle
  4. **Environmental Stats**: BME680 and GY-30 readings
- Uses `Adafruit_GFX` and `Adafruit_SSD1306`.

---

## 5. Spectrometer Adaptive Gain Control
- Adjusts **integration time** dynamically:
  - If max intensity > 2500 → decrease integration.
  - If max intensity < 2000 → increase integration.
  - Ensures signal remains within ADC range (0–4095).

---

## 6. WiFi & NTP Synchronization
- Connects to WiFi using `ssid` and `password`.
- Configures timezone with DST for **British Summer Time (BST)**.
- Falls back to `millis()` if NTP fails.

---

## 7. I2C Scanner
- Detects connected devices on startup.
- Provides **serial and OLED feedback** for troubleshooting.

---

## 8. Setup & Diagnostics
- Initializes all sensors and devices sequentially.
- Performs **OPV voltage diagnostic sweep** at low, medium, and high duty cycles.
- Initializes **PWM, SD card, and OLED**.

---

## 9. Main Loop
1. **Spectrometer reading** + **adaptive gain control**.
2. **BME680 & GY-30 readings**.
3. **MPPT conditional execution** based on lux:
   - Park MPPT if low lux.
   - Resume MPPT if lux is sufficient.
4. **MPPT P&O algorithm** updates PWM duty cycle.
5. **SD card logging** every 10 seconds (lux-dependent).
6. **OLED page cycling** every 5 seconds.

---

## 10. Key Parameters

| Parameter | Value / Notes |
|-----------|---------------|
| `LUX_MPPT_THRESHOLD` | 50 lux |
| `DUTY_PARK` | 50 (PWM units) |
| `PWM_FREQUENCY` | 1000 Hz |
| `PWM_RESOLUTION_BITS` | 12-bit |
| `MPPT_INTERVAL_MS` | 1000 ms |
| `SPECTROGRAPH_CHANNELS` | 288 |
| `SATURATION_DISPLAY_THRESHOLD` | 2600 |
| `TARGET_MAX_HIGH` | 2500 |
| `TARGET_MAX_LOW` | 2000 |

---

## ✅ Summary
This sketch provides a **robust multi-sensor data acquisition system** with **intelligent MPPT control for OpV panels**, including:
- **Low-light protection**
- **Adaptive spectrometer gain**
- **OLED visualization**
- **Structured SD card logging with timestamping**

Ideal for **research and experimentation with small photovoltaic panels under varying light conditions**.
