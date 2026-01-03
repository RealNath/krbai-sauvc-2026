#include <Arduino.h>
#include <ESP32Servo.h>

// --- USER CONFIGURATION ---
// Define your 8 GPIO pins connected to the ESCs here
// Ensure these pins are PWM capable (most GPIOs on S3 are)
const int thrusterPins[8] = {4, 5, 6, 7, 16, 8, 17, 18}; 

// Calibration Endpoints
const int MAX_THROTTLE = 2000;
const int MIN_THROTTLE = 1000;

// Create Servo objects for the 8 thrusters
Servo thrusters[8];

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open

  Serial.println("----------------------------------------------");
  Serial.println("   MULTI-ESC CALIBRATION TOOL (ESP32-S3)      ");
  Serial.println("----------------------------------------------");
  Serial.println("WARNING: PROPELLERS SHOULD BE REMOVED/CLEAR.");
  Serial.println("----------------------------------------------");

  // 1. Attach pins and immediately write MAX throttle
  // We do this BEFORE plugging in the battery
  Serial.println("STEP 1: Sending HIGH signal (2000us)...");
  
  for (int i = 0; i < 8; i++) {
    // Standard ESC range: 1000-2000us
    thrusters[i].setPeriodHertz(50); // Standard 50Hz servo/ESC signal
    thrusters[i].attach(thrusterPins[i], 1000, 2000);
    thrusters[i].writeMicroseconds(MAX_THROTTLE);
  }

  // 2. Prompt user to connect power
  Serial.println("STATUS: Max throttle sent to all 8 pins.");
  Serial.println("ACTION: Please PLUG IN your battery now.");
  Serial.println("LISTEN: Wait for the musical tone from the motors.");
  Serial.println("ACTION: Once the music stops (approx 3 seconds),");
  Serial.println("        type any character here and press ENTER to set Low point.");
  
  // 3. Wait for user input from Serial Monitor
  while (Serial.available() == 0) {
    delay(100);
  }
  
  // Clear the serial buffer
  while(Serial.available() > 0) Serial.read();

  // 4. Write MIN throttle
  Serial.println("\nSTEP 2: Sending LOW signal (1000us)...");
  for (int i = 0; i < 8; i++) {
    thrusters[i].writeMicroseconds(MIN_THROTTLE);
  }

  Serial.println("LISTEN: You should hear beep confirmation indicating calibration is done.");
  Serial.println("STATUS: Calibration Complete. Motors are now ARMED (0 throttle).");
  Serial.println("----------------------------------------------");
  Serial.println("Type 'test' to spin all motors at 10% speed for 2 seconds (optional).");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input == "test") {
      Serial.println("TESTING: Spinning at 1100us for 2 seconds...");
      for (int i = 0; i < 8; i++) {
        thrusters[i].writeMicroseconds(1100);
      }
      delay(2000);
      Serial.println("TESTING: Stopping...");
      for (int i = 0; i < 8; i++) {
        thrusters[i].writeMicroseconds(1000);
      }
    }
  }
}