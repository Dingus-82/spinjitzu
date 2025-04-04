#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

// Constants
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define SERVO_PIN (9)  // GPIO pin for servo on Pico
#define TARGET_ORIENTATION (0.0)  // Target heading in degrees
#define TOLERANCE (5.0)  // Tolerance in degrees
#define MAX_SERVO_SPEED (100)  // Maximum servo speed (0-180)
#define MIN_SERVO_SPEED (-100)    // Minimum servo speed (0-180)
#define NEUTRAL_SERVO (0)     // Neutral position (no rotation)

// PID Constants
#define KP (1.0)  // Proportional gain
#define KI (0.05) // Integral gain  
#define KD (0.1)  // Derivative gain

// Global variables
Adafruit_BNO055 bno = Adafruit_BNO055(55);  // IMU
Servo reactionWheel;  // Servo object for reaction wheel

// PID variables
float previousError = 0;
float integral = 0;
unsigned long previousTime = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);  // Wait for serial monitor
  
  Serial.println("Reaction Control Wheel System Initializing...");

  // Initialize IMU
  if (!bno.begin()) {
    Serial.println("Could not find BNO055 IMU. Check wiring!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  
  // Initialize reaction wheel servo
  pinMode(SERVO_PIN, OUTPUT);
  analogWriteFreq(50);  // Set PWM frequency to 50 Hz (20ms period)
  setServoSpeed(NEUTRAL_SERVO);  // Start with wheel stopped
  delay(1000);  // Allow servo to initialize
  
  Serial.println("System Ready");
}

void setServoSpeed(int speed) {
    // Map speed (-100 to 100) to pulse width (1000 to 2000 µs)
    int pulseWidth = map(speed, -100, 100, 1000, 2000);
    
    // Convert pulse width to duty cycle (0-255 for Pico's analogWrite)
    int dutyCycle = map(pulseWidth, 1000, 2000, 26, 51);  // ~5% to ~10% duty cycle

    analogWrite(9, dutyCycle);  // Set PWM output
}

void loop() {
  // Get current orientation from IMU
  sensors_event_t event;
  bno.getEvent(&event);
  float currentHeading = event.orientation.x;  // Using x-axis for heading
  
  // Calculate error (difference from target orientation)
  float error = calculateHeadingError(currentHeading, TARGET_ORIENTATION);
  
  // Calculate PID output
  float controlOutput = calculatePID(error);
  
  // Convert PID output to servo command
  int servoCommand = mapPIDToServo(controlOutput);
  
  // Apply command to reaction wheel
  setServoSpeed(servoCommand);
  
  // Debug output
  Serial.print("Heading: ");
  Serial.print(currentHeading);
  Serial.print("° | Error: ");
  Serial.print(error);
  Serial.print("° | PID Out: ");
  Serial.print(controlOutput);
  Serial.print(" | Servo Cmd: ");
  Serial.println(servoCommand);
  
  // Delay for next reading
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

float calculateHeadingError(float current, float target) {
  // Normalize the error to -180 to 180 range
  float error = target - current;
  while (error > 180) error -= 360;
  while (error < -180) error += 360;
  return error;
}

float calculatePID(float error) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;  // Convert to seconds
  
  // Proportional term
  float proportional = KP * error;
  
  // Integral term (with anti-windup)
  integral += KI * error * deltaTime;
  integral = constrain(integral, -MAX_SERVO_SPEED, MAX_SERVO_SPEED);
  
  // Derivative term
  float derivative = 0;
  if (deltaTime > 0) {
    derivative = KD * (error - previousError) / deltaTime;
  }
  
  // Calculate total output
  float output = proportional + integral + derivative;
  
  // Update variables for next iteration
  previousError = error;
  previousTime = currentTime;
  
  return output;
}

int mapPIDToServo(float pidOutput) {
  // Constrain PID output to servo range
  pidOutput = constrain(pidOutput, -MAX_SERVO_SPEED, MAX_SERVO_SPEED);
  
  // Map PID output to servo command
  int servoCommand = NEUTRAL_SERVO + pidOutput;
  servoCommand = constrain(servoCommand, MIN_SERVO_SPEED, MAX_SERVO_SPEED);
  
  return servoCommand;
}