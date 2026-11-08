#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// --- Hardware Definitions ---
#define IMU_ADDR       0x69
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT 0x3B

#define I2C_SDA        21
#define I2C_SCL        22

Adafruit_BME280 bme;

// --- Calibration Constants ---
float accOffsetX = 0.0f, accOffsetY = 0.0f, accOffsetZ = 0.0f;
float gyrOffsetX = 0.0f, gyrOffsetY = 0.0f, gyrOffsetZ = 0.0f;
float gravityRawMag = 16384.0f;
float groundPressure_hPa = 1013.25f;

// --- Filtered Sensor States ---
float az_lpf = 0.0f;
float baro_lpf = 0.0f;
float baroWindow[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
int baroWinIdx = 0;

// ==========================================
// 1. Roll & Pitch Kalman Filter
// ==========================================
struct KalmanAngle {
  float angle = 0.0f;
  float bias  = 0.0f;
  float P[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};

  float Q_angle   = 0.001f;
  float Q_bias    = 0.003f;
  float R_measure = 0.03f;

  float update(float newAngle, float newRate, float dt) {
    float rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    float S  = P[0][0] + R_measure;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;

    float y = newAngle - angle;
    angle += K0 * y;
    bias  += K1 * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K0 * P00_temp;
    P[0][1] -= K0 * P01_temp;
    P[1][0] -= K1 * P00_temp;
    P[1][1] -= K1 * P01_temp;

    return angle;
  }
};

KalmanAngle kfRoll;
KalmanAngle kfPitch;
float kfYaw = 0.0f;

// ==========================================
// 2. Non-Oscillating Altitude Kalman Filter
// ==========================================
struct KalmanAltitude {
  float z = 0.0f;    // Altitude (m)
  float v = 0.0f;    // Vertical velocity (m/s)
  float P[2][2] = {{0.1f, 0.0f}, {0.0f, 0.1f}};

  // Lower Q_accel + tighter R_baro prevents acceleration overshoot & oscillation
  float Q_accel = 0.08f;
  float R_baro  = 0.04f;

  void predict(float accel_z, float dt) {
    // Pure kinematic prediction (no artificial velocity decay that causes braking undershoot)
    z += v * dt + 0.5f * accel_z * dt * dt;
    v += accel_z * dt;

    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;

    float Q00 = 0.25f * dt4 * Q_accel;
    float Q01 = 0.5f  * dt3 * Q_accel;
    float Q11 = dt2   * Q_accel;

    float P00 = P[0][0] + dt * (P[1][0] + P[0][1]) + dt2 * P[1][1] + Q00;
    float P01 = P[0][1] + dt * P[1][1] + Q01;
    float P10 = P[1][0] + dt * P[1][1] + Q01;
    float P11 = P[1][1] + Q11;

    P[0][0] = P00; P[0][1] = P01;
    P[1][0] = P10; P[1][1] = P11;
  }

  void update(float z_baro) {
    float y = z_baro - z;
    float S = P[0][0] + R_baro;

    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;

    z += K0 * y;
    v += K1 * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K0 * P00_temp;
    P[0][1] -= K0 * P01_temp;
    P[1][0] -= K1 * P00_temp;
    P[1][1] -= K1 * P01_temp;
  }
};

KalmanAltitude kfAlt;

// --- Helper: 5-Sample Median Filter to Reject Barometer Spikes ---
float median5(float *arr) {
  float tmp[5];
  for (int i = 0; i < 5; i++) tmp[i] = arr[i];
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (tmp[j] < tmp[i]) {
        float t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t;
      }
    }
  }
  return tmp[2];
}

float readRelativeBaroMeters(float pressure_hPa) {
  if (pressure_hPa <= 300.0f || isnan(pressure_hPa)) return baro_lpf;
  return 44330.0f * (1.0f - pow(pressure_hPa / groundPressure_hPa, 0.190295f));
}

void calibrateSensors() {
  Serial.println("\n[CALIBRATION] Warming up BME280 (3 seconds)... Keep flat on table.");
  for (int i = 0; i < 150; i++) {
    bme.readPressure();
    delay(20);
  }

  Serial.println("[CALIBRATION] Sampling IMU & Barometer baseline...");
  const int samples = 600;
  double aX = 0, aY = 0, aZ = 0;
  double gX = 0, gY = 0, gZ = 0;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(REG_ACCEL_XOUT);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)14);

    if (Wire.available() >= 14) {
      aX += (int16_t)((Wire.read() << 8) | Wire.read());
      aY += (int16_t)((Wire.read() << 8) | Wire.read());
      aZ += (int16_t)((Wire.read() << 8) | Wire.read());
      Wire.read(); Wire.read();
      gX += (int16_t)((Wire.read() << 8) | Wire.read());
      gY += (int16_t)((Wire.read() << 8) | Wire.read());
      gZ += (int16_t)((Wire.read() << 8) | Wire.read());
    }
    delay(2);
  }

  accOffsetX = (float)(aX / samples);
  accOffsetY = (float)(aY / samples);
  gravityRawMag = (float)(aZ / samples); // Exact 1g LSB reading on Z
  accOffsetZ = gravityRawMag - 16384.0f;

  gyrOffsetX = (float)(gX / samples);
  gyrOffsetY = (float)(gY / samples);
  gyrOffsetZ = (float)(gZ / samples);

  double pSum = 0.0;
  int valid = 0;
  for (int i = 0; i < 100; i++) {
    float p = bme.readPressure() / 100.0f;
    if (p > 300.0f && p < 1200.0f) {
      pSum += p;
      valid++;
    }
    delay(10);
  }
  if (valid > 0) groundPressure_hPa = (float)(pSum / valid);

  for (int i = 0; i < 5; i++) baroWindow[i] = 0.0f;
  baro_lpf = 0.0f;
  az_lpf   = 0.0f;
  kfAlt.z  = 0.0f;
  kfAlt.v  = 0.0f;
  kfYaw    = 0.0f;

  Serial.printf("[CALIBRATION] Ready! Ground Pressure: %.2f hPa\n", groundPressure_hPa);
}

unsigned long prevMicros = 0;
int baroCounter = 0;
int stillCounter = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  Wire.setTimeOut(50);

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();

  if (!bme.begin(0x76, &Wire)) {
    if (!bme.begin(0x77, &Wire)) {
      Serial.println("[ERROR] BME280 not found!");
      while (1) delay(100);
    }
  }

  // Optimal low-lag indoor altimeter configuration
  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X1,   // Fast temp
                  Adafruit_BME280::SAMPLING_X8,   // Clean pressure without heavy conversion delay
                  Adafruit_BME280::SAMPLING_NONE, // No humidity needed
                  Adafruit_BME280::FILTER_X4,     // Fast IIR filter (prevents lag oscillation)
                  Adafruit_BME280::STANDBY_MS_0_5);

  calibrateSensors();
  prevMicros = micros();
}

void loop() {
  // Press 'z' in Serial Monitor to re-zero ground level at any time
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'z' || c == 'Z') {
      groundPressure_hPa = bme.readPressure() / 100.0f;
      for (int i = 0; i < 5; i++) baroWindow[i] = 0.0f;
      baro_lpf = 0.0f;
      kfAlt.z  = 0.0f;
      kfAlt.v  = 0.0f;
      kfYaw    = 0.0f;
    }
  }

  unsigned long currentMicros = micros();
  float dt = (currentMicros - prevMicros) / 1000000.0f;
  if (dt <= 0.0f || dt > 0.05f) dt = 0.004f;
  prevMicros = currentMicros;

  // 1. Read IMU
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(REG_ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)14);

  if (Wire.available() >= 14) {
    int16_t rawAX = (Wire.read() << 8) | Wire.read();
    int16_t rawAY = (Wire.read() << 8) | Wire.read();
    int16_t rawAZ = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();
    int16_t rawGX = (Wire.read() << 8) | Wire.read();
    int16_t rawGY = (Wire.read() << 8) | Wire.read();
    int16_t rawGZ = (Wire.read() << 8) | Wire.read();

    float ax = (rawAX - accOffsetX) / 16384.0f;
    float ay = (rawAY - accOffsetY) / 16384.0f;
    float az = (rawAZ - accOffsetZ) / 16384.0f;

    float gx = (rawGX - gyrOffsetX) / 131.0f;
    float gy = (rawGY - gyrOffsetY) / 131.0f;
    float gz = (rawGZ - gyrOffsetZ) / 131.0f;

    // 2. Roll, Pitch, Yaw Estimation
    float rollAcc  = atan2(ay, az) * 180.0f / PI;
    float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;

    float rollEst  = kfRoll.update(rollAcc, gx, dt);
    float pitchEst = kfPitch.update(pitchAcc, gy, dt);

    if (fabs(gz) > 0.15f) {
      kfYaw += gz * dt;
    }
    if (kfYaw > 180.0f)  kfYaw -= 360.0f;
    if (kfYaw < -180.0f) kfYaw += 360.0f;

    // 3. Tilt-Compensated Vertical Linear Acceleration
    float rollRad  = rollEst * PI / 180.0f;
    float pitchRad = pitchEst * PI / 180.0f;

    float az_world = (-sin(pitchRad) * ax +
                       sin(rollRad) * cos(pitchRad) * ay +
                       cos(rollRad) * cos(pitchRad) * az - 1.0f) * 9.80665f;

    az_lpf = 0.85f * az_lpf + 0.15f * az_world;

    // Motion / Stillness Detector
    float gyroMag = fabs(gx) + fabs(gy) + fabs(gz);
    if (fabs(az_lpf) < 0.22f && gyroMag < 3.0f) {
      stillCounter++;
    } else {
      stillCounter = 0;
    }
    bool isStill = (stillCounter > 20); // True whenever held still (in air OR on table)

    // If held still (in air or on table), zero out acceleration & velocity jitter
    // without touching the altitude z!
    float az_input = az_lpf;
    if (isStill || fabs(az_input) < 0.22f) {
      az_input = 0.0f;
      kfAlt.v *= 0.70f;
      if (fabs(kfAlt.v) < 0.02f) kfAlt.v = 0.0f;
    }

    // 4. Kalman Prediction (250 Hz)
    kfAlt.predict(az_input, dt);

    // 5. Kalman Barometer Update (50 Hz)
    baroCounter++;
    if (baroCounter >= 5) {
      baroCounter = 0;
      float p_hPa  = bme.readPressure() / 100.0f;
      float rawAlt = readRelativeBaroMeters(p_hPa);

      // Push into 5-sample median window to strip sudden pressure spikes
      baroWindow[baroWinIdx] = rawAlt;
      baroWinIdx = (baroWinIdx + 1) % 5;
      float medAlt = median5(baroWindow);

      // Smooth EMA filter on barometer
      baro_lpf = 0.75f * baro_lpf + 0.25f * medAlt;

      // Table Zero-Hold: ONLY active when completely flat (< 1.2 deg), motionless,
      // and already right on the table surface (< 12 cm). Never triggers in mid-air!
      bool onTableSurface = (isStill && fabs(rollEst) < 1.2f && fabs(pitchEst) < 1.2f && fabs(baro_lpf) < 0.12f);
      if (onTableSurface) {
        groundPressure_hPa = 0.98f * groundPressure_hPa + 0.02f * p_hPa;
        baro_lpf = 0.0f;
      }

      kfAlt.update(baro_lpf);
    }

    // Output Telemetry ( with optional 5 cm display deadband right at 0.00m )
    float displayAlt = (fabs(kfAlt.z) < 0.05f && isStill) ? 0.00f : kfAlt.z;

    Serial.printf("Roll:%.2f,Pitch:%.2f,Yaw:%.2f,Alt_m:%.2f,Baro_m:%.2f,V_z:%.2f\n",
                  rollEst, pitchEst, kfYaw, displayAlt, baro_lpf, kfAlt.v);
  }

  delay(4);
}