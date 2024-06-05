#include "config.h"
#include <Wire.h>
#include <Metro.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ODriveArduino.h>

int motorsActive = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(); 
ODriveArduino odrive(Serial2); 

Metro controllerMetro = Metro(CONTROLLER_INTERVAL);
Metro activationMetro = Metro(ACTIVATION_INTERVAL);

void setup() {
  Serial2.begin(BAUDRATE_ODRIVE); // ODrive uses 115200 baud
  Serial.begin(BAUDRATE_PC); // Serial to PC

  Serial.println("start");

  // IMU
  if (!bno.begin())
  {
    Serial.print("No BNO055 IMU detected...");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop() {
  controlTask();
  activationTask();
}

void controlTask() {
  if (controllerMetro.check()) {
    motionController();
  }
}

void activationTask() {
  if (activationMetro.check()) {
    modeSwitch(1);
  }
}

void motionController() {
  // IMU sampling
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  float balanceControllerOutput = euler.z() * KP_BALANCE + gyro.x() * KD_BALANCE;

  odrive.SetCurrent(0, MOTORDIR_0 * balanceControllerOutput);
  odrive.SetCurrent(1, MOTORDIR_1 * balanceControllerOutput);
}

void modeSwitch(int mode_desired) {
  if (mode_desired == motorsActive) {
  }
  else {
    if (mode_desired == 1) {
      int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial.println("Engaging motors");
      odrive.run_state(0, requested_state, false);
      odrive.run_state(1, requested_state, false);
    }
    else if (mode_desired == 0) {
      int requested_state = AXIS_STATE_IDLE;
      Serial.println("Disengaging motors");
      odrive.run_state(0, requested_state, false);
      odrive.run_state(1, requested_state, false);
    }
    else {
      Serial.println("Invalid mode selection");
    }
    motorsActive = mode_desired;
  }
  return;
}
