#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define NUM_THRUSTERS 8
#define PWM_FREQ 50

Servo thrusters[NUM_THRUSTERS];

// Check your wiring: usually on ESP32-S3, avoid pins 35-37 for output.
// Your current pins look safe.
int thrusterPins[NUM_THRUSTERS] = {
  4, 5, 6, 7, 15, 16, 17, 18
};

int thrusterValues[NUM_THRUSTERS];
int commandReceived = 0; // Status flag

// ================= IMU =================
Adafruit_MPU6050 mpu;
#define SDA_PIN 10
#define SCL_PIN 9

float calibX = -0.65; 
float calibY = 0.33; 
float calibZ = 1.17; 
float scaleX = 1.05;

// Variables
float ax, ay, az;
float gx, gy, gz;
float pitch = 0, roll = 0, yaw = 0;
float depthValue = 0.0; // Changed to float for precision

unsigned long lastTime = 0;
unsigned long lastCMD = 0;
float dt = 0;

void setup() {
  Serial.begin(115200);

  // 1. Initialize Thrusters
  // We allocate timers for ESP32PWM to ensure no conflicts
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < NUM_THRUSTERS; i++) {
    thrusters[i].setPeriodHertz(PWM_FREQ);
    thrusters[i].attach(thrusterPins[i], 1000, 2000);
    thrusters[i].writeMicroseconds(1500); // Initialize at STOP
    thrusterValues[i] = 1500;
  }
  
  // SAFETY DELAY: Give ESCs time to boot and hear the "Stop" signal
  delay(3000); 

  initIMU();
  Serial.println("AUV Ready");
}

void loop() {
  static unsigned long lastFB = 0;

  // 1. NON-BLOCKING COMMAND READER
  readSerialCommand();

  // 2. Failsafe (0.5s timeout)
  if (millis() - lastCMD > 500) { 
    for (int i = 0; i < NUM_THRUSTERS; i++) {
      thrusters[i].writeMicroseconds(1500);
    }
    commandReceived = 0; // Reset status
  }

  // 3. Update Sensors
  depthValue = updateDepthSensor();
  updateIMUsensor(); // No need to store return value really

  // 4. Telemetry (20 Hz)
  if (millis() - lastFB > 50) {
    sendFeedback();
    lastFB = millis();
  }
}

// ================= IMPROVED SERIAL READER =================
// This function never pauses the code. It builds the string char by char.
void readSerialCommand() {
  static String inputString = "";
  static boolean stringComplete = false;

  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
      break; 
    } else {
      inputString += inChar;
    }
  }

  if (stringComplete) {
    inputString.trim();
    if (inputString.startsWith("CMD")) {
      parseCommand(inputString);
      lastCMD = millis(); // Reset failsafe timer
      commandReceived = 1;
    }
    // Reset buffer
    inputString = "";
    stringComplete = false;
  }
}

void parseCommand(String data) {
  // Convert String to C-string for strtok
  char buffer[100];
  data.toCharArray(buffer, 100);

  char *token = strtok(buffer, ","); // Skip "CMD"
  token = strtok(NULL, ",");

  int index = 0;
  while (token != NULL && index < NUM_THRUSTERS) {
    int val = atoi(token);
    // SAFETY: Constrain immediately
    thrusterValues[index] = constrain(val, 1000, 2000);
    
    // Write immediately
    thrusters[index].writeMicroseconds(thrusterValues[index]);
    
    token = strtok(NULL, ",");
    index++;
  }
}

// ================= IMU LOGIC =================
void initIMU() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!mpu.begin()) {
    Serial.println("MPU FAIL");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

  // Pre-load logic
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  ax = (a.acceleration.x + calibX) * scaleX;
  ay = -(a.acceleration.y) + calibY;
  az = -(a.acceleration.z) + calibZ;

  roll  = atan2(ay, az) * 180 / PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

  lastTime = micros();
}

void updateIMUsensor() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  // Process Accel
  ax = a.acceleration.x + calibX;
  ay = -(a.acceleration.y) + calibY;
  az = -(a.acceleration.z) + calibZ;
  ax *= scaleX;

  float accelRoll  = atan2(ay, az) * 180 / PI;
  float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

  // Process Gyro
  gx = g.gyro.x * 180 / PI;
  gy = -g.gyro.y * 180 / PI;
  gz = -g.gyro.z * 180 / PI;

  // Filter
  pitch = 0.96 * (pitch + gy * dt) + 0.04 * accelPitch;
  roll  = 0.96 * (roll  + gx * dt) + 0.04 * accelRoll;
  yaw  += gz * dt;
}

// ================= SENSORS & FEEDBACK =================
float updateDepthSensor() {
  // TODO: Put real sensor logic here
  return 2.34; 
}

void sendFeedback() {
  Serial.print("FB,");
  Serial.print(commandReceived); Serial.print(",");
  Serial.print(depthValue, 2); Serial.print(","); // Now printing float!

  Serial.print(ax, 2); Serial.print(",");
  Serial.print(ay, 2); Serial.print(",");
  Serial.print(az, 2); Serial.print(",");

  Serial.print(gx, 2); Serial.print(",");
  Serial.print(gy, 2); Serial.print(",");
  Serial.print(gz, 2); Serial.print(",");

  Serial.print(pitch, 2); Serial.print(",");
  Serial.print(roll, 2); Serial.print(",");
  Serial.println(yaw, 2);
  
  // Optional: Reset command received status so you know if data is stale
  // commandReceived = 0; 
}