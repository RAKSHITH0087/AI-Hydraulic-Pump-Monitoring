#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <U8g2lib.h>

// =====================================================
// WIFI & API CONFIGURATION
// =====================================================
// Enter your WiFi network credentials:
const char* WIFI_SSID     = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// Backend API URL:
// Replace with the address of your own backend API.
const char* API_URL       = "http://YOUR_SERVER_IP:8443/api/telemetry";

// Replace with the same private API key configured in your backend.
const char* API_KEY       = "YOUR_API_KEY";

// How often to post data to the API over WiFi (milliseconds)
const unsigned long API_POST_INTERVAL_MS = 250; // 4 times per second

// =====================================================
// PIN CONFIGURATION
// =====================================================

#define I2C_SDA 21
#define I2C_SCL 22

#define DS18B20_PIN 4

// =====================================================
// SH1106 OLED
// =====================================================

U8G2_SH1106_128X64_NONAME_F_HW_I2C display(
  U8G2_R0,
  U8X8_PIN_NONE
);

// =====================================================
// DS18B20 (Single Temperature Sensor)
// =====================================================

OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

float temperatureC = -127.0;

// =====================================================
// MPU6050
// =====================================================

#define MPU6050_ADDR 0x68

#define ACCEL_XOUT_H 0x3B
#define TEMP_OUT_H   0x41
#define GYRO_XOUT_H  0x43

#define PWR_MGMT_1   0x6B
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG  0x1B

// =====================================================
// MPU6050 CALIBRATION OFFSETS
// =====================================================

const float ACCEL_OFFSET_X = 0.009;
const float ACCEL_OFFSET_Y = -0.003;
const float ACCEL_OFFSET_Z = -0.05;

const float GYRO_OFFSET_X = -1.004212;
const float GYRO_OFFSET_Y = -0.718951;
const float GYRO_OFFSET_Z = 0.542194;

// =====================================================
// SENSOR SCALE
// =====================================================

const float ACCEL_SCALE = 16384.0;
const float GYRO_SCALE  = 131.0;

// =====================================================
// SAMPLING & TIMING
// =====================================================

const unsigned long SAMPLE_PERIOD_US = 10000;   // 100 Hz MPU sampling

unsigned long nextSampleTime = 0;
unsigned long sampleCounter = 0;
unsigned long lastRateTime = 0;
float mpuDieTempC = 25.0;

// =====================================================
// SENSOR DATA STRUCTURE
// =====================================================

struct SensorData
{
  float accelX;
  float accelY;
  float accelZ;

  float gyroX;
  float gyroY;
  float gyroZ;

  float filteredX;
  float filteredY;
  float filteredZ;
};

// =====================================================
// FREE-RTOS DUAL CORE DATA SHARING
// =====================================================
// Core 1 runs the 100 Hz sampling loop without interruption.
// Core 0 runs the WiFi HTTP transmission task in the background.

struct TelemetrySnapshot
{
  SensorData sensor;
  float tempC;
  float dieTempC;
  bool valid;
};

portMUX_TYPE sensorMux = portMUX_INITIALIZER_UNLOCKED;
TelemetrySnapshot latestSnapshot = { {0}, -127.0, 25.0, false };

bool wifiConnected = false;
String wifiIP = "Connecting...";

// =====================================================
// BUTTERWORTH 1-20 Hz BANDPASS (Fs = 100 Hz)
// =====================================================

// ---------------- Section 1 ----------------
const float BP1_B0 = 0.19061660;
const float BP1_B1 = 0.38123320;
const float BP1_B2 = 0.19061660;
const float BP1_A1 = -0.42353116;
const float BP1_A2 = 0.22570987;

// ---------------- Section 2 ----------------
const float BP2_B0 = 1.00000000;
const float BP2_B1 = -2.00000000;
const float BP2_B2 = 1.00000000;
const float BP2_A1 = -1.91155124;
const float BP2_A2 = 0.91565330;

// =====================================================
// BIQUAD STATE
// =====================================================

struct BiquadState
{
  float z1;
  float z2;
};

BiquadState filterState[3][2];

// =====================================================
// BIQUAD PROCESSING
// =====================================================

float processBiquad(
  float input,
  BiquadState &state,
  float b0,
  float b1,
  float b2,
  float a1,
  float a2
)
{
  float output = b0 * input + state.z1;

  state.z1 =
    b1 * input -
    a1 * output +
    state.z2;

  state.z2 =
    b2 * input -
    a2 * output;

  return output;
}

// =====================================================
// BANDPASS FILTER
// =====================================================

float bandpassFilter(
  float input,
  BiquadState &section1,
  BiquadState &section2
)
{
  float output1 = processBiquad(
    input,
    section1,
    BP1_B0,
    BP1_B1,
    BP1_B2,
    BP1_A1,
    BP1_A2
  );

  float output2 = processBiquad(
    output1,
    section2,
    BP2_B0,
    BP2_B1,
    BP2_B2,
    BP2_A1,
    BP2_A2
  );

  return output2;
}

// =====================================================
// RESET FILTER
// =====================================================

void resetFilters()
{
  for (int axis = 0; axis < 3; axis++)
  {
    for (int section = 0; section < 2; section++)
    {
      filterState[axis][section].z1 = 0.0;
      filterState[axis][section].z2 = 0.0;
    }
  }
}

// =====================================================
// READ 16-BIT MPU REGISTER
// =====================================================

int16_t read16(byte reg)
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0)
    return 0;

  Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)2);

  if (Wire.available() < 2)
    return 0;

  int16_t value = Wire.read() << 8;
  value |= Wire.read();

  return value;
}

// =====================================================
// READ ACCELEROMETER
// =====================================================

void readAcceleration(
  float &x,
  float &y,
  float &z
)
{
  int16_t rawX = read16(ACCEL_XOUT_H);
  int16_t rawY = read16(ACCEL_XOUT_H + 2);
  int16_t rawZ = read16(ACCEL_XOUT_H + 4);

  x = rawX / ACCEL_SCALE;
  y = rawY / ACCEL_SCALE;
  z = rawZ / ACCEL_SCALE;
}

// =====================================================
// READ GYROSCOPE
// =====================================================

void readGyroscope(
  float &x,
  float &y,
  float &z
)
{
  int16_t rawX = read16(GYRO_XOUT_H);
  int16_t rawY = read16(GYRO_XOUT_H + 2);
  int16_t rawZ = read16(GYRO_XOUT_H + 4);

  x = rawX / GYRO_SCALE;
  y = rawY / GYRO_SCALE;
  z = rawZ / GYRO_SCALE;
}

// =====================================================
// READ MPU6050 DIE TEMPERATURE
// =====================================================

float readMPUDieTemp()
{
  int16_t raw = read16(TEMP_OUT_H);
  return (raw / 340.0) + 36.53;
}

// =====================================================
// INITIALIZE MPU6050
// =====================================================

void initializeMPU6050()
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00); // Wake MPU6050
  Wire.endTransmission();

  delay(100);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0x00); // Accelerometer ±2g
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x00); // Gyroscope ±250 °/s
  Wire.endTransmission();

  delay(100);
}

// =====================================================
// CHECK MPU6050
// =====================================================

bool checkMPU6050()
{
  Wire.beginTransmission(MPU6050_ADDR);
  return (Wire.endTransmission() == 0);
}

// =====================================================
// READ MPU6050 DATA
// =====================================================

void readSensor(SensorData &data)
{
  float rawAX, rawAY, rawAZ;
  float rawGX, rawGY, rawGZ;

  readAcceleration(rawAX, rawAY, rawAZ);
  readGyroscope(rawGX, rawGY, rawGZ);

  data.accelX = rawAX - ACCEL_OFFSET_X;
  data.accelY = rawAY - ACCEL_OFFSET_Y;
  data.accelZ = rawAZ - ACCEL_OFFSET_Z;

  data.gyroX = rawGX - GYRO_OFFSET_X;
  data.gyroY = rawGY - GYRO_OFFSET_Y;
  data.gyroZ = rawGZ - GYRO_OFFSET_Z;

  data.filteredX = bandpassFilter(data.accelX, filterState[0][0], filterState[0][1]);
  data.filteredY = bandpassFilter(data.accelY, filterState[1][0], filterState[1][1]);
  data.filteredZ = bandpassFilter(data.accelZ, filterState[2][0], filterState[2][1]);
}

// =====================================================
// OLED DISPLAY
// =====================================================

void displaySensorData(SensorData &data)
{
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);

  // ---------------- Line 1: Temp & WiFi Status ----------------
  display.setCursor(0, 9);
  if (temperatureC <= -100.0)
  {
    display.print("TEMP: ERR");
  }
  else
  {
    display.print("T:");
    display.print(temperatureC, 1);
    display.print("C");
  }

  display.setCursor(68, 9);
  if (wifiConnected)
  {
    display.print("[WiFi OK]");
  }
  else
  {
    display.print("[No WiFi]");
  }

  // ---------------- Line 2: Accel X & Y ----------------
  display.setCursor(0, 20);
  display.print("AX:");
  display.print(data.accelX, 2);
  display.print(" AY:");
  display.print(data.accelY, 2);

  // ---------------- Line 3: Accel Z ----------------
  display.setCursor(0, 31);
  display.print("AZ:");
  display.print(data.accelZ, 2);

  // ---------------- Line 4: Gyro X & Y ----------------
  display.setCursor(0, 42);
  display.print("GX:");
  display.print(data.gyroX, 1);
  display.print(" GY:");
  display.print(data.gyroY, 1);

  // ---------------- Line 5: Gyro Z ----------------
  display.setCursor(0, 53);
  display.print("GZ:");
  display.print(data.gyroZ, 1);

  // ---------------- Line 6: Filtered Accel ----------------
  display.setCursor(0, 64);
  display.print("F:");
  display.print(data.filteredX, 2);
  display.print(",");
  display.print(data.filteredY, 2);
  display.print(",");
  display.print(data.filteredZ, 2);

  display.sendBuffer();
}

// =====================================================
// SERIAL OUTPUT
// =====================================================

void printSensorData(SensorData &data)
{
  Serial.print("TEMP: ");
  if (temperatureC <= -100.0)
  {
    Serial.print("ERROR");
  }
  else
  {
    Serial.print(temperatureC, 2);
    Serial.print(" C");
  }

  Serial.print(" | A: ");
  Serial.print(data.accelX, 3);
  Serial.print(", ");
  Serial.print(data.accelY, 3);
  Serial.print(", ");
  Serial.print(data.accelZ, 3);

  Serial.print(" | F: ");
  Serial.print(data.filteredX, 3);
  Serial.print(", ");
  Serial.print(data.filteredY, 3);
  Serial.print(", ");
  Serial.print(data.filteredZ, 3);

  Serial.print(" | G: ");
  Serial.print(data.gyroX, 2);
  Serial.print(", ");
  Serial.print(data.gyroY, 2);
  Serial.print(", ");
  Serial.print(data.gyroZ, 2);

  Serial.print(" | WiFi: ");
  Serial.println(wifiConnected ? "Connected" : "Disconnected");
}

// =====================================================
// FREERTOS BACKGROUND TASK: WIFI & API TELEMETRY (CORE 0)
// =====================================================

void wifiTelemetryTask(void *pvParameters)
{
  Serial.print("[WiFi Task] Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  HTTPClient http;
  http.setReuse(true);
  http.setTimeout(1500);

  while (true)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      if (!wifiConnected)
      {
        wifiConnected = true;
        wifiIP = WiFi.localIP().toString();
        Serial.print("[WiFi] Connected! IP address: ");
        Serial.println(wifiIP);
      }

      // Read safely from the shared snapshot
      TelemetrySnapshot snap;
      portENTER_CRITICAL(&sensorMux);
      snap = latestSnapshot;
      portEXIT_CRITICAL(&sensorMux);

      if (snap.valid)
      {
        char jsonPayload[320];
        snprintf(
          jsonPayload,
          sizeof(jsonPayload),
          "{\"temperature_c\":%.2f,\"sensor_id\":\"ds18b20-0\",\"mpu_temp_c\":%.2f,"
          "\"accel\":{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f},"
          "\"gyro\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}}",
          snap.tempC,
          snap.dieTempC,
          snap.sensor.accelX,
          snap.sensor.accelY,
          snap.sensor.accelZ,
          snap.sensor.gyroX,
          snap.sensor.gyroY,
          snap.sensor.gyroZ
        );

        if (http.begin(API_URL))
        {
          http.addHeader("Content-Type", "application/json");
          http.addHeader("X-API-Key", API_KEY);

          int httpResponseCode = http.POST((uint8_t*)jsonPayload, strlen(jsonPayload));
          if (httpResponseCode <= 0)
          {
            Serial.print("[API Error] HTTP POST failed: ");
            Serial.println(http.errorToString(httpResponseCode).c_str());
          }
          http.end();
        }
      }
    }
    else
    {
      wifiConnected = false;
      wifiIP = "Connecting...";
    }

    vTaskDelay(pdMS_TO_TICKS(API_POST_INTERVAL_MS));
  }
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("======================================");
  Serial.println("ESP32 SENSOR SYSTEM (WIFI & API ENABLED)");
  Serial.println("DS18B20 + MPU6050 + SH1106 OLED");
  Serial.println("======================================");

  // ===================================================
  // I2C
  // ===================================================
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  delay(100);

  // ===================================================
  // OLED
  // ===================================================
  Serial.println("Initializing SH1106 OLED...");
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(10, 25, "SENSOR SYSTEM");
  display.drawStr(10, 40, "Connecting WiFi...");
  display.sendBuffer();

  // ===================================================
  // DS18B20 (Non-blocking conversion setup)
  // ===================================================
  Serial.println("Initializing DS18B20...");
  sensors.begin();
  // Non-blocking mode ensures the 100 Hz MPU loop does not freeze
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();

  Serial.print("DS18B20 sensors found: ");
  Serial.println(sensors.getDeviceCount());

  // ===================================================
  // MPU6050
  // ===================================================
  Serial.println("Checking MPU6050 at 0x68...");
  if (checkMPU6050())
  {
    Serial.println("MPU6050 detected.");
    initializeMPU6050();
    Serial.println("MPU6050 initialized.");
  }
  else
  {
    Serial.println("ERROR: MPU6050 not detected! Check wiring.");
  }

  // ===================================================
  // RESET FILTER
  // ===================================================
  resetFilters();

  // ===================================================
  // START WIFI TELEMETRY TASK ON CORE 0
  // ===================================================
  // Pinning WiFi transmission to Core 0 leaves Core 1 dedicated
  // to the 100 Hz sampling and Butterworth filter!
  xTaskCreatePinnedToCore(
    wifiTelemetryTask,
    "WiFiTelemetryTask",
    8192,
    NULL,
    1,
    NULL,
    0 // Core 0
  );

  // ===================================================
  // TIMING
  // ===================================================
  nextSampleTime = micros();
  lastRateTime = millis();

  Serial.println();
  Serial.println("======================================");
  Serial.println("SYSTEM READY");
  Serial.println("Sampling rate: 100 Hz");
  Serial.println("Filter: 1-20 Hz");
  Serial.println("======================================");
}

// =====================================================
// MAIN LOOP (CORE 1)
// =====================================================

void loop()
{
  unsigned long currentTime = micros();

  // ===================================================
  // 100 Hz MPU SAMPLE TIMING
  // ===================================================
  if ((long)(currentTime - nextSampleTime) < 0)
  {
    return;
  }

  nextSampleTime += SAMPLE_PERIOD_US;

  // ===================================================
  // READ MPU6050
  // ===================================================
  SensorData sensor;
  readSensor(sensor);
  sampleCounter++;

  // ===================================================
  // READ DS18B20 & MPU DIE TEMP
  // ===================================================
  // Read temperature periodically without blocking 100 Hz loop
  static unsigned long lastTemperatureTime = 0;
  if (millis() - lastTemperatureTime >= 750)
  {
    lastTemperatureTime = millis();

    float temp = sensors.getTempCByIndex(0);
    if (temp == DEVICE_DISCONNECTED_C || temp < -100.0)
    {
      temperatureC = -127.0;
    }
    else
    {
      temperatureC = temp;
    }

    // Trigger next asynchronous temperature conversion
    sensors.requestTemperatures();

    // Read MPU6050 die temperature
    mpuDieTempC = readMPUDieTemp();
  }

  // Update shared telemetry snapshot for Core 0 background WiFi task
  portENTER_CRITICAL(&sensorMux);
  latestSnapshot.sensor = sensor;
  latestSnapshot.tempC = temperatureC;
  latestSnapshot.dieTempC = mpuDieTempC;
  latestSnapshot.valid = true;
  portEXIT_CRITICAL(&sensorMux);

  // ===================================================
  // OLED UPDATE (Every 100 ms)
  // ===================================================
  static unsigned long lastDisplayTime = 0;
  if (millis() - lastDisplayTime >= 100)
  {
    lastDisplayTime = millis();
    displaySensorData(sensor);
  }

  // ===================================================
  // SERIAL OUTPUT (10 Hz)
  // ===================================================
  if (sampleCounter % 10 == 0)
  {
    printSensorData(sensor);
  }

  // ===================================================
  // SAMPLE RATE REPORTING
  // ===================================================
  if (millis() - lastRateTime >= 1000)
  {
    float actualRate = (sampleCounter * 1000.0) / (millis() - lastRateTime);
    Serial.print("Actual sample rate: ");
    Serial.print(actualRate, 2);
    Serial.println(" Hz");

    sampleCounter = 0;
    lastRateTime = millis();
  }
}