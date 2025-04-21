/*
_____________________________________________________________
Code for BNO IMU connected via I2C.
Code originally by: Ashton Posey
Code modified for Pi Pico W by: Radhakrishna Vojjala
Date of last modification: 6 Jan 2025
_____________________________________________________________
This file contains the Setup function for the BNO055 IMU.
The Setup function is simply bno.begin()
*/

// Update function for BNO IMU. The setup function is simply bno.begin();

void BNOupdate(){
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER, VECTOR_GRAVITY...
  sensors_event_t orientationData, angVelocityData, magnetometerData, accelerometerData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  accelerometer[0] = (float)accelerometerData.acceleration.x;
  accelerometer[1] = (float)accelerometerData.acceleration.y;
  accelerometer[2] = (float)accelerometerData.acceleration.z;
  orientation[0] = (float)orientationData.orientation.x;
  orientation[1] = (float)orientationData.orientation.y;
  orientation[2] = (float)orientationData.orientation.z;
  magnetometer[0] = (float)magnetometerData.magnetic.x;
  magnetometer[1] = (float)magnetometerData.magnetic.y;
  magnetometer[2] = (float)magnetometerData.magnetic.z;
  gyroscope[0] = (float)angVelocityData.gyro.x;
  gyroscope[1] = (float)angVelocityData.gyro.y;
  gyroscope[2] = (float)angVelocityData.gyro.z;

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
}