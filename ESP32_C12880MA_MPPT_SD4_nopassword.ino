/*
This sketch is designed for an ESP32 microcontroller and a suite of sensors, incorporating
fixes for the MPPT (Maximum Power Point Tracking) algorithm, especially for low-light
conditions, as detailed in the "MPPT_Fix_Documentation.docx" file.

The primary functions of this sketch are:
1.  **Sensor Data Acquisition**: Reads data from multiple sensors:
    * **Spectrometer (C12880MA)**: Captures spectral data.
    * **INA226 Power Sensor**: Measures voltage, current, and power from an OpV (Organic Photovoltaic) panel.
    * **BME680 Environmental Sensor**: Measures temperature, pressure, humidity, and gas resistance.
    * **GY-30 (BH1750) Illuminance Sensor**: Measures ambient light levels (lux).
2.  **Maximum Power Point Tracking (MPPT)**: Implements a Perturb & Observe (P&O) algorithm to find the optimal operating point for the OpV panel.
    * **MPPT Fix Implementation**:
        * **Pause MPPT at Night**: The MPPT algorithm is now paused when the ambient light level (lux) falls below a predefined threshold (LUX_MPPT_THRESHOLD). During this time, the PWM duty cycle is set to a "parking" value (DUTY_PARK) which corresponds to a light load, preventing the panel from being shorted.
        * **Re-enable and Reset on Daybreak**: When lux levels rise above the threshold, MPPT is re-enabled, and the algorithm's state variables (V_old, I_old, P_old) are reset to ensure it starts tracking accurately from the beginning of the day.
        * **Anti-windup Logic**: Simple logic is added to prevent the duty cycle from getting stuck at extreme minimum or maximum values.
3.  **SD Card Data Logging**: All collected sensor data (MPPT, BME680, GY-30, Spectrometer) is saved to a single CSV file on an SD card.
    * **Conditional Logging**: Data is *only* written to the SD card when the ambient light level (GY-30 sensor) is greater than 50 lux. This prevents logging irrelevant data during dark conditions, optimizing SD card usage.
    * **Timestamping**: WiFi is used to connect to an NTP server to get accurate date and time, which is then used to timestamp each data entry in the CSV file. If WiFi connection or NTP synchronization fails, a fallback timestamp using `millis()` will be used, marked as "UNSYNCED".
4.  **Diagnostic I2C Scanner**: Identifies connected I2C devices at startup for troubleshooting.
5.  **Adaptive Gain Control**: For the spectrometer, this feature dynamically adjusts the integration time to prevent saturation and optimize signal quality.
6.  **OLED Display**: Cycles through different pages on a connected 128x64 OLED screen to display real-time sensor data, including spectrometer statistics, spectrograph plots, MPPT parameters, and environmental readings.

This sketch aims to provide a robust and efficient data acquisition and MPPT system for OpV research, especially under varying light conditions, while ensuring reliable data logging with accurate timestamps.
*/

#include <Wire.h>          // Required for I2C communication (for INA226, OLED, BME680, GY-30)
#include <INA226_WE.h>     // Library for the INA226 sensor (Wolfgang Ewald)
#include <Adafruit_GFX.h>  // Core graphics library for OLED
#include <Adafruit_SSD1306.h> // Library for SSD1306 OLED displays
#include <SPI.h>           // Required for SD card communication
#include <SD.h>            // SD card library
#include <WiFi.h>          // Required for WiFi connection for NTP
#include <time.h>          // Required for time functions (localtime_r, strftime)
#include <Adafruit_BME680.h> // Library for BME680 sensor
#include <Adafruit_Sensor.h> // Required for some sensor types (used by BME680)

// --- WiFi Credentials ---
const char* ssid          = "";
const char* password      = "";

// NTP Server Configuration
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0; // GMT offset in seconds (e.g., 0 for UTC, 3600 for GMT+1)
const int  daylightOffset_sec = 3600; // Daylight saving offset in seconds (e.g., 3600 for +1 hour)

// --- I2C Pin Definitions (Common for INA226, OLED, BME680, GY-30) ---
#define SDA_PIN GPIO_NUM_21 // ESP32 I2C SDA pin
#define SCL_PIN GPIO_NUM_22 // ESP32 I2C SCL pin

// --- PWM Pin for DFRobot MOSFET Power Controller Module ---
#define PWM_PIN GPIO_NUM_15 // ESP32 GPIO pin for PWM control of MOSFET

// --- INA226 Calibration Parameters ---
const float RSHUNT = 10.0; // Shunt resistor value in Ohms
const float MAX_CURRENT_EXPECTED_AMPS = 0.1; // Max current expected in Amps (100 mA)
INA226_WE ina226; // Create an INA226 sensor object

// --- BME680 Sensor Object ---
Adafruit_BME680 bme; // I2C address 0x77 (default) or 0x76

// --- GY-30 (BH1750) Sensor Address ---
#define GY30_ADDRESS 0x23 // Common I2C address for BH1750 (can also be 0x5C)

// --- OLED Display Configuration ---
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_I2C_ADDRESS 0x3D // Common I2C address for 128x64 OLED (can also be 0x3C)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Create an SSD1306 display object

// --- PWM Configuration for DFRobot MOSFET ---
const int PWM_FREQUENCY = 1000; // Hz (1KHz, as supported by DFRobot MOSFET module)
const int PWM_RESOLUTION_BITS = 12; // 12-bit resolution for PWM (0-4095 steps)
// Note: PWM_CHANNEL is no longer used with analogWrite, but kept for clarity if reverting
// const int PWM_CHANNEL = 0; // ESP32 LEDC channel for PWM (0-15 available)

// --- MPPT Parameters for Perturb and Observe (P&O) Algorithm ---
float V_old = 0.0;           // Previous voltage measurement from OPV
float I_old = 0.0;           // Previous current measurement from OPV
float P_old = 0.0;           // Previous power measurement from OPV
int current_duty_cycle = 100; // Initial PWM duty cycle (started much lower to allow voltage to build)
const int DUTY_CYCLE_STEP = 50; // Step size for perturbing the duty cycle
unsigned long last_mppt_time = 0;
const unsigned long MPPT_INTERVAL_MS = 1000; // Run MPPT algorithm every 1 second

// --- MPPT Fix Specific Parameters ---
bool mpptEnabled = true; // Flag to enable/disable MPPT
const float LUX_MPPT_THRESHOLD = 50.0; // Lux threshold to enable/disable MPPT
const int DUTY_PARK = 50; // Duty cycle to set when MPPT is parked (light load)

// --- C12880MA Spectrometer Pin Definitions ---
#define C12880MA_CLK_PIN    16 // Clock signal output to C12880MA
#define C12880MA_ST_PIN     17 // Start pulse output to C12880MA
#define C12880MA_VIDEO_PIN  34 // Analog input for video signal from C12880MA (ADC1_CH6)
#define C12880MA_EOS_PIN    25 // End-of-Scan input from C12880MA (optional for sync)
#define WHITE_LED_PIN       26 // Control for integrated white LED on breakout board
#define LASER_404_PIN       27 // Control for integrated 405nm laser on breakout board

// --- SD Card Chip Select Pin ---
#define SD_CS_PIN           5 // Chip Select for the SD card module

// --- Consolidated SD Card Filename ---
const char* ALL_SENSORS_DATA_FILENAME = "/SENSOR_LOG.CSV";

#define SPECTROGRAPH_CHANNELS 288 // Number of data points from C12880MA
uint16_t spectrographData[SPECTROGRAPH_CHANNELS]; // Array to hold spectrograph data (16-bit)

// --- OLED Layout Definitions (for spectrograph display) ---
const int LEFT_MARGIN = 25;   // Space for Y-axis labels
const int RIGHT_MARGIN = 5;   // Small right padding
const int TOP_MARGIN = 15;    // Space for title
const int BOTTOM_MARGIN = 15; // Space for X-axis labels
const int PLOT_LEFT = LEFT_MARGIN;
const int PLOT_RIGHT = SCREEN_WIDTH - RIGHT_MARGIN;
const int PLOT_TOP = TOP_MARGIN;
const int PLOT_BOTTOM = SCREEN_HEIGHT - BOTTOM_MARGIN;
const int PLOT_WIDTH = PLOT_RIGHT - PLOT_LEFT;
const int PLOT_HEIGHT = PLOT_BOTTOM - PLOT_TOP;

// --- Spectrometer Gain Control Parameters ---
volatile unsigned long currentIntegrationTimeUs = 11; // Start with 11us as per C12880MA min
const uint16_t ADC_MAX_VALUE = 4095; // 12-bit ADC theoretical max value (0-4095)
const uint16_t SATURATION_DISPLAY_THRESHOLD = 2600; // Trigger "SATURATED!" at or above this value
const uint16_t TARGET_MAX_HIGH = 2500; // If currentMaxIntensity > 2500, decrease gain
const uint16_t TARGET_MAX_LOW = 2000;  // If below 2000, increase gain
const unsigned long MIN_INTEGRATION_TIME_US = 11;      // Minimum integration time (11 microseconds)
const unsigned long MAX_INTEGRATION_TIME_US = 1000000; // Maximum integration time (1 second)
const float ESP32_ADC_MAX_VOLTAGE_11DB = 2.45; // Reference voltage for ESP32 ADC with 11db attenuation

// --- Global variables for SD card and spectrometer ---
volatile unsigned int sdCardWriteCount = 0; // For all sensor data
uint16_t currentMaxIntensity = 0; // Max intensity from last spectrometer scan
bool sdCardInitialized = false; // Flag to track SD card status

// --- Global variables for environmental sensors ---
float bmeTemp, bmePressure, bmeHumidity, bmeGasResistance;
float gy30Illuminance;

// --- Global for OLED page cycling ---
unsigned long lastPageChangeTime = 0;
int currentPage = 0; // 0: Spectrometer Stats, 1: Spectrograph, 2: MPPT Stats, 3: Environmental Stats

// --- Function Prototypes ---
void displayI2CDevices(); // New function for I2C scan
void readSpectrometer();
void adaptiveGainControl();
void displaySpectrometerStatsPage();
void displaySpectrographPage();
void displayMPPTPage(float V, float I, float P, int dc);
void displayEnvironmentalPage(); // New function for BME680/GY-30 data
void writeCSVHeader(); // Modified for consolidated data
void saveAllSensorDataToCSV(float mpptV, float mpptI, float mpptP, int mpptDC,
                            float bmeT, float bmeP, float bmeH, float bmeG,
                            float gy30Lux); // Modified for all data

// --- Function Implementations ---

// Function to scan for and display I2C devices
void displayI2CDevices() {
  display.clearDisplay(); // Use Adafruit_SSD1306 display object
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("I2C Scan...");
  display.display();

  Serial.println("\nI2C Scanner");
  Serial.println("Scanning I2C addresses:");
  int nDevices = 0;

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("   ACK");

      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("I2C Devices Found:");
      display.setCursor(0, 10);
      display.print("0x");
      if (address < 16) display.print("0");
      display.print(address, HEX);
      display.println(" Found!");
      display.display();
      delay(500); // Small delay to show each found device
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("No I2C Devices!");
    display.display();
    delay(1000);
  } else {
    Serial.println("I2C Scan Complete.\n");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(nDevices);
    display.println(" I2C Devices Found!");
    display.display();
    delay(1500);
  }
}


// Function to read spectrograph data from C12880MA
void readSpectrometer() {
    const int delayTime = 1; // microseconds delay for CLK pulses (minimal, can be adjusted)

    analogSetPinAttenuation(C12880MA_VIDEO_PIN, ADC_11db); // Set attenuation for the video pin
    analogReadResolution(12); // Set ADC resolution to 12 bits (0-4095) for ESP32

    // --- C12880MA Spectrometer Readout Sequence ---
    digitalWrite(C12880MA_ST_PIN, HIGH);
    delayMicroseconds(10); // ST pulse width (adjust based on datasheet, e.g., 10us minimum)
    digitalWrite(C12880MA_ST_PIN, LOW);

    // Dummy clocks (pre-integration phase, typically 15 pulses)
    for (int i = 0; i < 15; i++) {
        digitalWrite(C12880MA_CLK_PIN, HIGH);
        delayMicroseconds(delayTime);
        digitalWrite(C12880MA_CLK_PIN, LOW);
        delayMicroseconds(delayTime);
    }

    // Integration time (gain control)
    delayMicroseconds(currentIntegrationTimeUs);

    digitalWrite(C12880MA_CLK_PIN, HIGH); // Final clock pulse before video readout
    delayMicroseconds(delayTime);
    digitalWrite(C12880MA_CLK_PIN, LOW);
    delayMicroseconds(delayTime);

    // Read actual pixel data (288 pixels)
    currentMaxIntensity = 0; // Reset for the current scan
    for (int i = 0; i < SPECTROGRAPH_CHANNELS; i++) {
        digitalWrite(C12880MA_CLK_PIN, HIGH);
        delayMicroseconds(delayTime);
        spectrographData[i] = analogRead(C12880MA_VIDEO_PIN); // Reads 0-4095
        if (spectrographData[i] > currentMaxIntensity) {
            currentMaxIntensity = spectrographData[i];
        }
        digitalWrite(C12880MA_CLK_PIN, LOW);
        delayMicroseconds(delayTime);
    }

    digitalWrite(C12880MA_ST_PIN, HIGH); // Signal end of read cycle
    for (int i = 0; i < 7; i++) { // Post-readout dummy clocks
        digitalWrite(C12880MA_CLK_PIN, HIGH);
        delayMicroseconds(delayTime);
        digitalWrite(C12880MA_CLK_PIN, LOW);
        delayMicroseconds(delayTime);
    }
    digitalWrite(C12880MA_CLK_PIN, HIGH); // Final clock pulse
    delayMicroseconds(delayTime);

    digitalWrite(C12880MA_ST_PIN, LOW);
    digitalWrite(C12880MA_CLK_PIN, LOW);
}

// Adaptive Gain Control Logic (adjusts sensor integration time)
void adaptiveGainControl() {
    unsigned long newIntegrationTimeUs = currentIntegrationTimeUs;

    if (currentMaxIntensity >= ADC_MAX_VALUE) {
        newIntegrationTimeUs = MIN_INTEGRATION_TIME_US;
    } else if (currentMaxIntensity > TARGET_MAX_HIGH) {
        newIntegrationTimeUs = currentIntegrationTimeUs / 2;
    } else if (currentMaxIntensity < TARGET_MAX_LOW) {
        newIntegrationTimeUs = currentIntegrationTimeUs * 2;
    }

    newIntegrationTimeUs = constrain(newIntegrationTimeUs, MIN_INTEGRATION_TIME_US, MAX_INTEGRATION_TIME_US);

    if (newIntegrationTimeUs != currentIntegrationTimeUs) {
        currentIntegrationTimeUs = newIntegrationTimeUs;
    }
}

// Function to display the text-based spectrometer stats page on OLED
void displaySpectrometerStatsPage() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.print("Max Pixel:");
    display.print(currentMaxIntensity);

    display.setCursor(0, 10);
    display.print("Int. Time:");
    display.print(currentIntegrationTimeUs);
    display.print("us");

    float videoVoltage = (float)currentMaxIntensity * (ESP32_ADC_MAX_VOLTAGE_11DB / ADC_MAX_VALUE) * 2.0;
    display.setCursor(0, 20);
    display.print("Video V:");
    display.print(videoVoltage, 2);
    display.print("V");

    display.setCursor(0, 30);
    display.print("SD Writes:");
    if (sdCardInitialized) {
      display.print(sdCardWriteCount);
    } else {
      display.print("N/A (No SD)");
    }

    display.setTextSize(1);
    display.setCursor(0, 40);
    if (currentMaxIntensity >= SATURATION_DISPLAY_THRESHOLD) {
        display.print("SATURATED!");
    } else {
        display.print("UNSATURATED");
    }

    display.display();
}

// Function to display the spectrograph plot page on OLED
void displaySpectrographPage() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.print("Pixel Data ");
    display.print(currentIntegrationTimeUs);
    display.print("us");

    uint16_t graphScaleMax = currentMaxIntensity;
    if (graphScaleMax == 0) {
        graphScaleMax = ADC_MAX_VALUE;
    }

    for (int x = 0; x < PLOT_WIDTH; x++) {
        int spectrographIndex = map(x, 0, PLOT_WIDTH - 1, 0, SPECTROGRAPH_CHANNELS - 1);
        uint16_t value = spectrographData[spectrographIndex];
        int barHeight = map(value, 0, graphScaleMax, 0, PLOT_HEIGHT);
        barHeight = constrain(barHeight, 0, PLOT_HEIGHT);
        display.drawFastVLine(PLOT_LEFT + x, PLOT_BOTTOM - barHeight, barHeight, SSD1306_WHITE);
    }

    display.setCursor(0, PLOT_TOP);
    display.print(graphScaleMax);
    display.setCursor(0, PLOT_BOTTOM - 8);
    display.print(0);

    display.setCursor(PLOT_LEFT, PLOT_BOTTOM + 2);
    display.print("0");
    String maxPixelStr = String(SPECTROGRAPH_CHANNELS - 1);
    display.setCursor(PLOT_RIGHT - (maxPixelStr.length() * 6), PLOT_BOTTOM + 2);
    display.print(maxPixelStr);
    String midPixelStr = String(SPECTROGRAPH_CHANNELS / 2);
    int midX = PLOT_LEFT + PLOT_WIDTH / 2;
    display.setCursor(midX - (midPixelStr.length() * 3), PLOT_BOTTOM + 2);
    display.print(midPixelStr);

    display.setTextSize(1);
    display.setCursor(0, 56);
    if (currentMaxIntensity >= SATURATION_DISPLAY_THRESHOLD) {
        display.print("SATURATED!");
    } else {
        display.print("UNSATURATED");
    }
    display.display();
}

// Function to display MPPT data on OLED
void displayMPPTPage(float V, float I, float P, int dc) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.println("MPPT Data:");

    display.setCursor(0, 16);
    display.print("V: ");
    display.print(V, 4);
    display.println(" V");

    display.setCursor(0, 32);
    display.print("I: ");
    display.print(I * 1000000, 2);
    display.println(" uA");

    display.setCursor(0, 48);
    display.print("P: ");
    display.print(P * 1000000, 2);
    display.println(" uW");

    display.setCursor(0, 56); // Place DC lower to avoid overlap
    display.print("DC: ");
    display.print(dc);
    display.print("/");
    display.println((1 << PWM_RESOLUTION_BITS) - 1); // Max value for the set resolution

    display.display();
}

// New function to display BME680 and GY-30 data on OLED
void displayEnvironmentalPage() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.println("Env. Sensors:");

    display.setCursor(0, 10);
    display.print("Temp: ");
    display.print(bmeTemp, 1);
    display.println(" C");

    display.setCursor(0, 20);
    display.print("Pres: ");
    display.print(bmePressure / 100.0, 1); // hPa
    display.println(" hPa");

    display.setCursor(0, 30);
    display.print("Hum: ");
    display.print(bmeHumidity, 1);
    display.println(" %");

    display.setCursor(0, 40);
    display.print("Gas: ");
    display.print(bmeGasResistance / 1000.0, 1); // kOhms
    display.println(" kOhm");

    display.setCursor(0, 50);
    display.print("Lux: ");
    display.print(gy30Illuminance, 1);
    display.println(" lx");

    display.display();
}


// Write CSV header for the consolidated data file
void writeCSVHeader() {
    if (!sdCardInitialized) return;

    File dataFile = SD.open(ALL_SENSORS_DATA_FILENAME, FILE_APPEND);
    if (dataFile) {
        if (dataFile.size() == 0) {
            // Common Timestamp
            dataFile.print("Timestamp,");
            // MPPT Data
            dataFile.print("MPPT_Voltage_V,MPPT_Current_A,MPPT_Power_W,MPPT_DutyCycle,");
            // BME680 Data
            dataFile.print("BME_Temp_C,BME_Pressure_hPa,BME_Humidity_perc,BME_Gas_Ohm,");
            // GY-30 Data
            dataFile.print("GY30_Illuminance_Lux,");
            // Spectrometer Status
            dataFile.print("Spectro_Saturation_Status,Spectro_IntegrationTime_us");
            // Spectrometer Pixels
            for (int i = 0; i < SPECTROGRAPH_CHANNELS; i++) {
                dataFile.print(",Spectro_pix");
                dataFile.print(i);
            }
            dataFile.println();
            Serial.println("Consolidated CSV Header written.");
        }
        dataFile.close();
    } else {
        Serial.println("Error opening consolidated data file to check/write header.");
    }
}

// Function to save all sensor data to SD card as CSV
void saveAllSensorDataToCSV(float mpptV, float mpptI, float mpptP, int mpptDC,
                            float bmeT, float bmeP, float bmeH, float bmeG,
                            float gy30Lux) {
    if (!sdCardInitialized) {
        Serial.println("SD card not initialized. Skipping data save.");
        return;
    }

    File dataFile = SD.open(ALL_SENSORS_DATA_FILENAME, FILE_APPEND);
    if (dataFile) {
        time_t now;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);
        char timestampStr[20];

        if (timeinfo.tm_year < (2000 - 1900)) { // Check if time is unsynced (before year 2000)
            snprintf(timestampStr, sizeof(timestampStr), "UNSYNCED_%lu", millis());
        } else {
            strftime(timestampStr, sizeof(timestampStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
        }

        dataFile.print(timestampStr);
        dataFile.print(",");
        // MPPT Data
        dataFile.print(mpptV, 4);
        dataFile.print(",");
        dataFile.print(mpptI, 6);
        dataFile.print(",");
        dataFile.print(mpptP, 6);
        dataFile.print(",");
        dataFile.print(mpptDC);
        dataFile.print(",");
        // BME680 Data
        dataFile.print(bmeT, 1);
        dataFile.print(",");
        dataFile.print(bmeP / 100.0, 1); // Convert Pa to hPa for logging
        dataFile.print(",");
        dataFile.print(bmeH, 1);
        dataFile.print(",");
        dataFile.print(bmeG, 0); // Gas resistance as whole number
        dataFile.print(",");
        // GY-30 Data
        dataFile.print(gy30Lux, 1);
        dataFile.print(",");
        // Spectrometer Status
        if (currentMaxIntensity >= SATURATION_DISPLAY_THRESHOLD) {
            dataFile.print("SATURATED");
        } else {
            dataFile.print("UNSATURATED");
        }
        dataFile.print(",");
        dataFile.print(currentIntegrationTimeUs);

        // Spectrometer Pixels
        for (int i = 0; i < SPECTROGRAPH_CHANNELS; i++) {
            dataFile.print(",");
            dataFile.print(spectrographData[i]);
        }
        dataFile.println();
        dataFile.close();
        Serial.println("All Sensor Data saved successfully.");
        sdCardWriteCount++; // Increment the counter on successful write
    } else {
        Serial.println("Error opening consolidated data file for writing.");
    }
}


void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("Initializing Combined MPPT & Spectrometer System...");

    // --- Initialize I2C Bus (Common for INA226, OLED, BME680, GY-30) ---
    Wire.begin(SDA_PIN, SCL_PIN);
    display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS); // Initialize OLED before scan
    display.display();
    delay(100);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("System Initializing...");
    display.display();
    delay(1000);

    // --- Scan and display I2C devices ---
    displayI2CDevices();

    // --- INA226 Sensor Setup ---
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("INA226 Init...");
    display.display();
    if (!ina226.init()) {
      Serial.println("Failed to find INA226 sensor! Check wiring.");
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("INA226 Error!");
      display.display();
      while (1) { delay(10); } // Halt if sensor not found
    }
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("INA226 Found.");
    display.display();
    delay(500);

    ina226.setResistorRange(RSHUNT, MAX_CURRENT_EXPECTED_AMPS);
    Serial.print("INA226 calibrated with RSHUNT: "); Serial.print(RSHUNT, 2); Serial.println(" Ohms");
    Serial.print("Max Expected Current: "); Serial.print(MAX_CURRENT_EXPECTED_AMPS, 3); Serial.println(" Amps (used for range selection reference)");
    ina226.setConversionTime(CONV_TIME_1100);
    ina226.setAverage(AVERAGE_16);
    ina226.setMeasureMode(CONTINUOUS);
    Serial.println("INA226 configured for continuous measurement.");

    // --- BME680 Sensor Setup ---
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("BME680 Init...");
    display.display();
    if (!bme.begin()) {
      Serial.println("Could not find a valid BME680 sensor, check wiring!");
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("BME680 Error!");
      display.display();
      while (1);
    }
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320 C for 150 ms
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("BME680 Found.");
    display.display();
    delay(500);

    // --- GY-30 (BH1750) Sensor Setup ---
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("GY-30 Init...");
    display.display();
    Wire.beginTransmission(GY30_ADDRESS);
    if (Wire.endTransmission() == 0) { // 0 indicates success
      Serial.println("GY-30 (BH1750) found at 0x23.");
      // Send power on command and continuous high-resolution mode ONCE
      Wire.beginTransmission(GY30_ADDRESS);
      Wire.write(0x01); // Power On
      Wire.endTransmission();
      Wire.beginTransmission(GY30_ADDRESS);
      Wire.write(0x10); // Continuous High Resolution Mode (0.5 lux resolution)
      Wire.endTransmission();
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("GY-30 Found.");
      display.display();
      delay(500);
    } else {
      Serial.println("Could not find GY-30 (BH1750) sensor at 0x23, check wiring!");
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("GY-30 Error!");
      display.display();
      while (1);
    }


    // --- IMPORTANT: Ensure all GND pins are connected together (ESP32, INA226, DFRobot MOSFET, OPV Negative, Spectrometer, BME680, GY-30) ---
    Serial.println("Please ensure all GND pins (ESP32, INA226, DFRobot MOSFET, OPV Negative, Spectrometer, BME680, GY-30) are connected to a common ground.");

    // --- SD Card Initialization ---
    Serial.print("Initializing SD card...");
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("SD Init...");
    display.display();
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD card initialization failed or not present!");
        sdCardInitialized = false;
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("SD Error!");
        display.display();
        delay(1000);
    } else {
      Serial.println("SD card initialized.");
      sdCardInitialized = true;
      writeCSVHeader(); // Write consolidated header
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("SD Card Found.");
      display.display();
      delay(500);
    }

    // --- WiFi & NTP Setup ---
    Serial.print("Connecting to WiFi: "); Serial.println(ssid);
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("WiFi Connecting...");
    display.display();
    WiFi.begin(ssid, password);
    int wifi_retries = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_retries < 20) {
        delay(500); Serial.print("."); wifi_retries++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected.");
        Serial.print("IP Address: "); Serial.println(WiFi.localIP());
        // Set timezone to BST (British Summer Time)
        // BST is UTC+1, with a daylight saving rule.
        // The rule "BST-1BST,M3.5.0/1,M10.5.0/2" means:
        // BST: Timezone abbreviation for standard time. Offset is -1 hour (meaning UTC+1).
        // BST: Timezone abbreviation for daylight saving time (also BST).
        // M3.5.0/1: DST starts on the 5th Sunday of March at 1:00 AM.
        // M10.5.0/2: DST ends on the 5th Sunday of October at 2:00 AM.
        setenv("TZ", "BST-1BST,M3.5.0/1,M10.5.0/2", 1);
        tzset();
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        Serial.println("Waiting for NTP time synchronization...");
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("NTP Sync...");
        display.display();
        time_t now = time(nullptr);
        int ntp_retries = 0;
        while (time(nullptr) < 1672531200 && ntp_retries < 10) { // Check if year is before 2023 (arbitrary past date for sync check)
            delay(500); Serial.print("*"); ntp_retries++;
        }
        now = time(nullptr);
        if (now >= 1672531200) {
            Serial.println("\nNTP time synchronized.");
            struct tm timeinfo; localtime_r(&now, &timeinfo);
            Serial.print("Current time: "); Serial.println(asctime(&timeinfo));
            display.clearDisplay();
            display.setCursor(0,0);
            display.println("NTP Synced.");
            display.display();
            delay(500);
        } else {
            Serial.println("\nNTP time synchronization failed. Timestamps may be inaccurate.");
            display.clearDisplay();
            display.setCursor(0,0);
            display.println("NTP Failed!");
            display.display();
            delay(1000);
        }
    } else {
      Serial.println("\nWiFi connection failed. Timestamps will be based on uptime or be 'UNSYNCED'.");
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("WiFi Failed!");
      display.display();
      delay(1000);
    }

    // --- Spectrometer Pin Setup ---
    pinMode(C12880MA_CLK_PIN, OUTPUT); digitalWrite(C12880MA_CLK_PIN, LOW);
    pinMode(C12880MA_ST_PIN, OUTPUT); digitalWrite(C12880MA_ST_PIN, LOW);
    pinMode(WHITE_LED_PIN, OUTPUT); digitalWrite(WHITE_LED_PIN, LOW);
    pinMode(LASER_404_PIN, OUTPUT); digitalWrite(LASER_404_PIN, LOW);
    pinMode(C12880MA_EOS_PIN, INPUT_PULLUP);
    Serial.println("Spectrometer pins configured.");

    // --- PWM Setup for DFRobot MOSFET Control (using analogWrite) ---
    analogWriteResolution(PWM_PIN, PWM_RESOLUTION_BITS); // Set 12-bit resolution for the specific pin
    analogWriteFrequency(PWM_PIN, PWM_FREQUENCY); // Set PWM frequency globally for analogWrite
    Serial.print("Initial duty cycle: "); Serial.print(current_duty_cycle); Serial.println(".");

    // --- Diagnostic Sweep for OPV Voltage ---
    Serial.println("\n--- Starting OPV Voltage Diagnostic Sweep ---");
    float diag_V, diag_I, diag_P;

    int low_duty_cycle = 50;
    analogWrite(PWM_PIN, low_duty_cycle);
    Serial.print("Setting DC to "); Serial.print(low_duty_cycle); Serial.println(" (High Resistance)...");
    delay(2000);
    diag_V = ina226.getBusVoltage_V(); diag_I = ina226.getCurrent_mA() / 1000.0; diag_P = diag_V * diag_I;
    Serial.print("   Readings @ Low DC: V="); Serial.print(diag_V, 4); Serial.print("V, I="); Serial.print(diag_I * 1000000, 2); Serial.print("uA, P="); Serial.print(diag_P * 1000000, 2); Serial.println("uW");

    int mid_duty_cycle = 2048;
    analogWrite(PWM_PIN, mid_duty_cycle);
    Serial.print("Setting DC to "); Serial.print(mid_duty_cycle); Serial.println(" (Medium Resistance)...");
    delay(2000);
    diag_V = ina226.getBusVoltage_V(); diag_I = ina226.getCurrent_mA() / 1000.0; diag_P = diag_V * diag_I;
    Serial.print("   Readings @ Mid DC: V="); Serial.print(diag_V, 4); Serial.print("V, I="); Serial.print(diag_I * 1000000, 2); Serial.print("uA, P="); Serial.print(diag_P * 1000000, 2); Serial.println("uW");

    int high_duty_cycle = 4000;
    analogWrite(PWM_PIN, high_duty_cycle);
    Serial.print("Setting DC to "); Serial.print(high_duty_cycle); Serial.println(" (Low Resistance)...");
    delay(2000);
    diag_V = ina226.getBusVoltage_V(); diag_I = ina226.getCurrent_mA() / 1000.0; diag_P = diag_V * diag_I;
    Serial.print("   Readings @ High DC: V="); Serial.print(diag_V, 4); Serial.print("V, I="); Serial.print(diag_I * 1000000, 2); Serial.print("uA, P="); Serial.print(diag_P * 1000000, 2); Serial.println("uW");
    Serial.println("--- Diagnostic Sweep Complete ---");

    analogWrite(PWM_PIN, current_duty_cycle); // Set initial duty cycle for MPPT loop
    Serial.println("System Initialized. Starting main loop...");
    delay(100);
}

void loop() {
    unsigned long current_time = millis();

    // --- Spectrometer Reading and Gain Control (Moved to top as it's not light-dependent) ---
    readSpectrometer();
    adaptiveGainControl();

    // --- BME680 Readings ---
    if (bme.performReading()) {
      bmeTemp = bme.temperature;
      bmePressure = bme.pressure;
      bmeHumidity = bme.humidity;
      bmeGasResistance = bme.gas_resistance;
    } else {
      // Serial.println("Failed to read BME680 data."); // Commented out to reduce serial spam
    }

    // --- GY-30 (BH1750) Readings ---
    if (Wire.requestFrom(GY30_ADDRESS, 2) == 2) {
      uint16_t rawLux = Wire.read();
      rawLux = (rawLux << 8) | Wire.read();
      gy30Illuminance = rawLux / 1.2; // Convert raw to Lux
      // Serial.print("GY-30 - Lux: "); Serial.println(gy30Illuminance); // Commented out to reduce serial spam
    } else {
      // Serial.println("Failed to read GY-30 data."); // Commented out to reduce serial spam
      gy30Illuminance = -1.0; // Indicate error
    }

    // --- Debugging: Print current Lux and MPPT state ---
    Serial.print("Time: "); Serial.print(current_time);
    Serial.print(", Lux: "); Serial.print(gy30Illuminance, 1);
    Serial.print(", MPPT Enabled: "); Serial.print(mpptEnabled ? "YES" : "NO");

    // --- MPPT Fix: Conditional MPPT Execution based on Lux ---
    if (gy30Illuminance <= LUX_MPPT_THRESHOLD) {
      if (mpptEnabled) {
        Serial.println(", ACTION: Parking MPPT due to low lux.");
        analogWrite(PWM_PIN, DUTY_PARK); // Park the MOSFET at a light load
        V_old = 0.0; // Reset MPPT state variables
        I_old = 0.0;
        P_old = 0.0;
        mpptEnabled = false;
        Serial.print("  -> MPPT parked. Duty Cycle: "); Serial.println(DUTY_PARK);
      } else {
        Serial.println(", STATUS: MPPT remains parked (low lux).");
      }
    } else {
      if (!mpptEnabled) {
        Serial.println(", ACTION: Re-enabling MPPT (lux ok).");
        // Re-enable MPPT and reset state
        analogWrite(PWM_PIN, DUTY_PARK); // Start with a light load
        V_old = 0.0;
        I_old = 0.0;
        P_old = 0.0;
        mpptEnabled = true;
        Serial.print("  -> MPPT re-enabled. Starting Duty Cycle: "); Serial.println(DUTY_PARK);
      } else {
        Serial.println(", STATUS: MPPT is active (lux ok).");
      }
    }


    // --- MPPT Algorithm and Readings (Only runs if mpptEnabled is true) ---
    if (mpptEnabled && (current_time - last_mppt_time >= MPPT_INTERVAL_MS)) {
      last_mppt_time = current_time;

      float V_new = ina226.getBusVoltage_V();
      float I_new = ina226.getCurrent_mA() / 1000.0;
      float P_new = V_new * I_new;

      // Ensure values are not negative
      if (P_new < 0) P_new = 0;
      if (I_new < 0) I_new = 0;
      if (V_new < 0) V_new = 0;

      // Perturb and Observe Logic (CORRECTED)
      if (P_new > P_old) { // Power increased - continue in the same direction
        if (V_new > V_old) { // Voltage increased (implies previous step was DC decrease)
          current_duty_cycle -= DUTY_CYCLE_STEP; // Continue decreasing DC
        } else { // Voltage decreased (implies previous step was DC increase)
          current_duty_cycle += DUTY_CYCLE_STEP; // Continue increasing DC
        }
      } else { // P_new <= P_old (Power decreased - reverse direction)
        if (V_new > V_old) { // Voltage increased (implies previous step was DC decrease)
          current_duty_cycle += DUTY_CYCLE_STEP; // Reverse to increasing DC
        } else { // Voltage decreased (implies previous step was DC increase)
          current_duty_cycle -= DUTY_CYCLE_STEP; // Reverse to decreasing DC
        }
      }

      // Anti-windup and boundary checks for duty cycle
      int max_duty_cycle = (1 << PWM_RESOLUTION_BITS) - 1; // 4095 for 12-bit
      current_duty_cycle = constrain(current_duty_cycle, DUTY_CYCLE_STEP, max_duty_cycle - DUTY_CYCLE_STEP);
      // Ensure it doesn't get stuck at 0 or max, allowing for perturbation

      analogWrite(PWM_PIN, current_duty_cycle);

      // --- Debugging: Print detailed MPPT data ---
      Serial.print("  MPPT Step -> V_old: "); Serial.print(V_old, 4);
      Serial.print("V, I_old: "); Serial.print(I_old, 6);
      Serial.print("A, P_old: "); Serial.print(P_old, 6);
      Serial.print("W, V_new: "); Serial.print(V_new, 4);
      Serial.print("V, I_new: "); Serial.print(I_new, 6);
      Serial.print("A, P_new: "); Serial.print(P_new, 6);
      Serial.print("W, New DC: "); Serial.println(current_duty_cycle);

      V_old = V_new;
      I_old = I_new;
      P_old = P_new;
    }


    // --- Automatic SD card write for ALL sensor data every 10 seconds, but only when lux > 50 ---
    static unsigned long lastSensorDataSaveTime = 0;
    if (current_time - lastSensorDataSaveTime >= 10000) {
        if (gy30Illuminance > LUX_MPPT_THRESHOLD) { // Use the same lux threshold for logging
            saveAllSensorDataToCSV(V_old, I_old, P_old, current_duty_cycle,
                                   bmeTemp, bmePressure, bmeHumidity, bmeGasResistance,
                                   gy30Illuminance);
            Serial.println("  -> Data logged to SD card.");
        } else {
            Serial.println("  -> Lux is too low (< 50). Skipping data log to SD card.");
        }
        lastSensorDataSaveTime = current_time;
    }


    // --- OLED Page Cycling Logic ---
    if (current_time - lastPageChangeTime >= 5000) {
      currentPage = (currentPage + 1) % 4; // Cycle between 0, 1, 2, 3
      lastPageChangeTime = current_time;
    }

    // --- Display relevant page on OLED ---
    if (currentPage == 0) {
      displaySpectrometerStatsPage();
    } else if (currentPage == 1) {
      displaySpectrographPage();
    } else if (currentPage == 2) {
      displayMPPTPage(V_old, I_old, P_old, current_duty_cycle);
    } else { // currentPage == 3
      displayEnvironmentalPage();
    }

    delay(50);
}









