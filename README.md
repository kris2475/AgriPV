# ☀️ OPV Research Node: MPPT & Spectroscopic Data Logger (ESP32)

A robust, multi-sensor data acquisition and power management system built on the **ESP32** platform, specifically designed for **Organic Photovoltaic (OPV)** research. It implements an advanced **Maximum Power Point Tracking (MPPT)** algorithm with a unique **low-light fix** and logs comprehensive sensor data, including full spectral scans, to an SD card with **NTP timestamping**.

## ✨ Key Features

* **Advanced MPPT with Low-Light Fix:** Implements a Perturb & Observe (P&O) MPPT algorithm for the OPV panel. Crucially, it includes an integrated fix to:
    * **Pause MPPT at Night:** Automatically halts the algorithm when ambient light (Lux < 50), setting the load to a light "parking" value to prevent the shorting of the panel in the dark.
    * **Daybreak Reset:** Resets MPPT state variables when light returns, ensuring accurate tracking from the start of the day.
* **Comprehensive Sensor Suite:** Collects data from five different sensors:
    * **INA226:** Measures high-precision voltage, current, and power of the OPV panel.
    * **C12880MA Spectrometer:** Captures detailed spectral data (288 channels) with **Adaptive Gain Control** to prevent sensor saturation.
    * **BME680:** Environmental monitoring (Temperature, Pressure, Humidity, and Gas Resistance).
    * **GY-30 (BH1750):** High-resolution ambient light measurement (Lux).
* **Reliable Data Logging:** All sensor data is consolidated into a single **CSV file** on an **SD Card**.
    * **Conditional Logging:** Data is only written when ambient light is sufficient (Lux > 50), optimizing SD card lifetime and reducing irrelevant data.
    * **Accurate Timestamping:** Connects to **WiFi/NTP** for precise date and time stamping. Falls back to an `UNSYNCED_millis()` timestamp if network connection fails.
* **Real-time OLED Diagnostics:** Displays live sensor readings, MPPT parameters, and a dynamic spectrograph plot on a 128x64 OLED display for immediate troubleshooting and monitoring.
* **I2C Diagnostic Scanner:** Runs at startup to confirm the presence and addresses of all I2C-based sensors.

---

## 🛠️ Hardware Requirements

This project is built around the following main components:

| Component | Function | Interface |
| :--- | :--- | :--- |
| **ESP32 Microcontroller** | Main processing unit | N/A |
| **INA226 Power Sensor** | OPV V/I/P Measurement | I2C |
| **C12880MA Spectrometer** | Spectral Data Acquisition | Analog/Digital (SPI-like) |
| **BME680 Environmental Sensor** | T/P/H/Gas Data | I2C |
| **GY-30 (BH1750)** | Ambient Illuminance (Lux) | I2C |
| **DFRobot MOSFET Power Controller** | Variable Load for MPPT | PWM |
| **128x64 SSD1306 OLED** | Real-time Display | I2C |
| **Micro SD Card Module** | Data Logging Storage | SPI |

---

## ⚙️ Software Setup and Configuration

### 1. Arduino IDE / PlatformIO Setup

The project requires the following libraries:

| Library | Author | Purpose |
| :--- | :--- | :--- |
| `INA226_WE` | Wolfgang Ewald | INA226 communication |
| `Adafruit_GFX` | Adafruit | Core graphics for OLED |
| `Adafruit_SSD1306` | Adafruit | OLED display driver |
| `Adafruit_BME680` | Adafruit | BME680 sensor driver |
| `WiFi.h`, `SPI.h`, `SD.h` | Standard ESP32/Arduino | Networking and Storage |

### 2. Wiring Diagram (Sketch)

The core communication busses and control lines are as follows:

| Component | ESP32 Pin (GPIO) | Notes |
| :--- | :--- | :--- |
| **I2C Bus (SDA)** | `GPIO_NUM_21` | Shared by INA226, BME680, GY-30, OLED |
| **I2C Bus (SCL)** | `GPIO_NUM_22` | Shared by INA226, BME680, GY-30, OLED |
| **SD Card CS** | `GPIO_NUM_5` | Chip Select for SD Card Module (SPI Bus) |
| **MPPT PWM Out** | `GPIO_NUM_15` | Control Signal for DFRobot MOSFET |
| **Spectrometer CLK** | `GPIO_NUM_16` | Clock Pulse Out |
| **Spectrometer ST** | `GPIO_NUM_17` | Start Pulse Out |
| **Spectrometer VIDEO** | `GPIO_NUM_34` | Analog Input (ADC1\_CH6) |

***Note:*** *As mentioned in the code, ensure **all GND pins** of the ESP32, sensors, and power module are connected to a **common ground** to prevent floating voltages and ensure accurate INA226 measurements.*

### 3. Configuration & Tuning

Key parameters are defined at the top of the sketch. Adjust these as necessary for your specific OPV panel and environment (e.g., in the file `config.h` or directly in the main sketch):

* **WiFi Credentials:**
    ```cpp
    const char* ssid = "YOUR_WIFI_SSID";
    const char* password = "YOUR_WIFI_PASSWORD";
    ```
* **MPPT Tuning:**
    ```cpp
    const float LUX_MPPT_THRESHOLD = 50.0; // Lux threshold to enable/disable MPPT
    const int DUTY_CYCLE_STEP = 50; // P&O step size (affects tracking speed)
    const unsigned long MPPT_INTERVAL_MS = 1000; // Tracking frequency
    ```
* **INA226 Calibration:**
    ```cpp
    const float RSHUNT = 10.0; // Shunt resistor value in Ohms (Must match your hardware)
    const float MAX_CURRENT_EXPECTED_AMPS = 0.1; // Max current for calibration range
    ```
* **Spectrometer Gain:**
    ```cpp
    const uint16_t TARGET_MAX_HIGH = 2500; // Max ADC value before decreasing integration time
    const uint16_t TARGET_MAX_LOW = 2000;  // Min ADC value before increasing integration time
    ```

---

## 💾 Data Output Format

The system logs a single line of data to `/SENSOR_LOG.CSV` at each sampling interval (when Lux > 50).

| Field Group | Example Fields | Description |
| :--- | :--- | :--- |
| **Timestamp** | `Timestamp` | YYYY-MM-DD HH:MM:SS (NTP Synced) or `UNSYNCED_millis()` |
| **MPPT** | `MPPT_Voltage_V`, `MPPT_Current_A`, `MPPT_Power_W`, `MPPT_DutyCycle` | OpV electrical operating point and load setting. |
| **BME680** | `BME_Temp_C`, `BME_Pressure_hPa`, `BME_Humidity_perc`, `BME_Gas_Ohm` | Environmental parameters. |
| **GY-30** | `GY30_Illuminance_Lux` | Ambient light used for MPPT conditional logic. |
| **Spectrometer** | `Spectro_Saturation_Status`, `Spectro_IntegrationTime_us` | ADC status and current gain setting. |
| **Spectrograph** | `Spectro_pix0`, `Spectro_pix1`, ..., `Spectro_pix287` | The full 288-channel raw spectral data. |

 DOI: 10.5281/zenodo.17266828



