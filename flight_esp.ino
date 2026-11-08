#include <Arduino.h>
#include <Wire.h>

// --- Hardware Definitions ---
#define IMU_ADDR       0x69
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT 0x3B

#define I2C_SDA        21
#define I2C_SCL        22

// --- Calibrated Sensor Parameters ---
float accOffsetX = 0.0f, accOffsetY = 0.0f;
float accScale   = 16384.0f; 
float gyrOffsetX = 0.0f, gyrOffsetY = 0.0f, gyrOffsetZ = 0.0f;

// ==========================================
// 1D Angle Kalman Filter (Roll & Pitch)
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

// --- FreeRTOS Threading & Synchronization ---
TaskHandle_t IMUTaskHandle = NULL;
SemaphoreHandle_t stateMutex = NULL;

// Cross-core shared state struct
struct DroneState {
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
} sharedState;

volatile bool zeroYawRequest = false;
unsigned long prevMicros = 0;

void calibrateIMU() {
  Serial.println("\n[CALIBRATION] Keep ESP32 flat and completely still...");
  delay(1000);

  const int samples = 800;
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
  accScale   = (float)(aZ / samples);
  if (accScale < 12000.0f || accScale > 20000.0f) accScale = 16384.0f;

  gyrOffsetX = (float)(gX / samples);
  gyrOffsetY = (float)(gY / samples);
  gyrOffsetZ = (float)(gZ / samples);

  kfYaw = 0.0f;
  Serial.println("[CALIBRATION] Ready! FreeRTOS IMU Task Active.");
}

// ==========================================
// Core 0: Deterministic Sensor Task (250 Hz)
// ==========================================
void IMU_Task(void *pvParameters) {
  // FreeRTOS precise timing variables
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(4); // 4ms = 250 Hz

  for (;;) {
    // Halts execution precisely until the next 4ms tick
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Handle Yaw Zeroing requested from Core 1
    if (zeroYawRequest) {
      kfYaw = 0.0f;
      zeroYawRequest = false;
    }

    unsigned long currentMicros = micros();
    float dt = (currentMicros - prevMicros) / 1000000.0f;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.004f;
    prevMicros = currentMicros;

    // Read IMU 
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

      float ax = (rawAX - accOffsetX) / accScale;
      float ay = (rawAY - accOffsetY) / accScale;
      float az = (float)rawAZ / accScale;

      float gx = (rawGX - gyrOffsetX) / 131.0f;
      float gy = (rawGY - gyrOffsetY) / 131.0f;
      float gz = (rawGZ - gyrOffsetZ) / 131.0f;

      float rollAcc  = atan2(ay, az) * 180.0f / PI;
      float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;

      float rollEst  = kfRoll.update(rollAcc, gx, dt);
      float pitchEst = kfPitch.update(pitchAcc, gy, dt);

      float rollRad  = rollEst * PI / 180.0f;
      float pitchRad = pitchEst * PI / 180.0f;
      float cosPitch = cos(pitchRad);

      if (fabs(cosPitch) > 0.15f) {
        float yawRateEarth = (gy * sin(rollRad) + gz * cos(rollRad)) / cosPitch;
        if (fabs(yawRateEarth) > 0.20f) {
          kfYaw += yawRateEarth * dt;
        }
      }

      if (kfYaw > 180.0f)  kfYaw -= 360.0f;
      if (kfYaw < -180.0f) kfYaw += 360.0f;

      // Lock Mutex and update the shared state for Core 1 to read
      if (xSemaphoreTake(stateMutex, 0) == pdTRUE) {
        sharedState.roll = rollEst;
        sharedState.pitch = pitchEst;
        sharedState.yaw = kfYaw;
        xSemaphoreGive(stateMutex);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  Wire.setTimeOut(50);

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();

  calibrateIMU();
  prevMicros = micros();

  // 1. Create a Mutex to prevent data corruption between cores
  stateMutex = xSemaphoreCreateMutex();
  
  // 2. Pin the IMU Task to Core 0 with Priority 2
  xTaskCreatePinnedToCore(
    IMU_Task,        // Function pointer
    "IMU_Task",      // Task name
    4096,            // Stack size 
    NULL,            // Parameters
    2,               // Priority (Higher = more critical)
    &IMUTaskHandle,  // Task handle
    0                // Pin to Core 0
  );
}

// ==========================================
// Core 1: Telemetry & Serial Reader
// ==========================================
void loop() {
  // Safely trigger a yaw reset flag for the IMU task
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'z' || c == 'Z') {
      zeroYawRequest = true;
    }
  }

  // Safely copy the latest attitude angles from the Mutex
  float localRoll = 0.0f, localPitch = 0.0f, localYaw = 0.0f;
  
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    localRoll  = sharedState.roll;
    localPitch = sharedState.pitch;
    localYaw   = sharedState.yaw;
    xSemaphoreGive(stateMutex);
  }

  // Stream data out
  Serial.printf("Roll:%.2f,Pitch:%.2f,Yaw:%.2f\n", localRoll, localPitch, localYaw);
  
  // Allow Core 1 background tasks to run (Yield)
  vTaskDelay(pdMS_TO_TICKS(4)); 
}