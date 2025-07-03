#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#include <ESP32Servo.h>

// Motor and control pins
int rightMotorPin1 = 14, rightMotorPin2 = 27;
int leftMotorPin1  = 26, leftMotorPin2  = 25;
int sleepPin       = 20;

// Ultrasonic sensor pins
const int trigPin = 5;
const int echoPin = 18;

// Servo pin
const int servoPin = 17;
Servo servo;

// Obstacle avoidance mode flag
bool obstacleMode = false;
const int OBSTACLE_THRESHOLD = 25; // cm

// Tuning: how long to rotate when choosing a direction (in milliseconds)
const unsigned long TURN_DURATION = 600; // adjust as needed

//#define Forward_speed 150
#define MAX_MOTOR_SPEED 255
const int PWMFreq = 1000;      // PWM frequency
const int PWMResolution = 8;   // PWM resolution
const int rightMotorPin1PWMChannel = 4;
const int rightMotorPin2PWMChannel = 5;
const int leftMotorPin1PWMChannel  = 6;
const int leftMotorPin2PWMChannel  = 7;

// Dabble connection flag and joystick deadzone
bool dabbleConnected = false;
const float ANALOG_THRESHOLD = 2.0;

// Rotate motors with DRV8833
void rotateMotor(int rightSpeed, int leftSpeed) {
  // Right motor
  if (rightSpeed < 0) {
    ledcWrite(rightMotorPin1PWMChannel, 0);
    ledcWrite(rightMotorPin2PWMChannel, abs(rightSpeed));
  } else if (rightSpeed > 0) {
    ledcWrite(rightMotorPin1PWMChannel, abs(rightSpeed));
    ledcWrite(rightMotorPin2PWMChannel, 0);
  } else {
    ledcWrite(rightMotorPin1PWMChannel, 0);
    ledcWrite(rightMotorPin2PWMChannel, 0);
  }
  // Left motor
  if (leftSpeed < 0) {
    ledcWrite(leftMotorPin1PWMChannel, 0);
    ledcWrite(leftMotorPin2PWMChannel, abs(leftSpeed));
  } else if (leftSpeed > 0) {
    ledcWrite(leftMotorPin1PWMChannel, abs(leftSpeed));
    ledcWrite(leftMotorPin2PWMChannel, 0);
  } else {
    ledcWrite(leftMotorPin1PWMChannel, 0);
    ledcWrite(leftMotorPin2PWMChannel, 0);
  }
}

// Measure distance in cm using HC-SR04
long measureDistanceCM() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // timeout to avoid blocking too long
  if (duration == 0) {
    // No echo within timeout: treat as “far” or clear
    return 0;
  }
  long distance = duration * 0.034 / 2; // speed of sound ~0.034 cm/us
  return distance;
}

void setUpPinModes() {
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftMotorPin1,  OUTPUT);
  pinMode(leftMotorPin2,  OUTPUT);
  pinMode(sleepPin, OUTPUT);
  digitalWrite(sleepPin, HIGH);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  ledcSetup(rightMotorPin1PWMChannel, PWMFreq, PWMResolution);
  ledcSetup(rightMotorPin2PWMChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPin1PWMChannel,  PWMFreq, PWMResolution);
  ledcSetup(leftMotorPin2PWMChannel,  PWMFreq, PWMResolution);

  ledcAttachPin(rightMotorPin1, rightMotorPin1PWMChannel);
  ledcAttachPin(rightMotorPin2, rightMotorPin2PWMChannel);
  ledcAttachPin(leftMotorPin1,  leftMotorPin1PWMChannel);
  ledcAttachPin(leftMotorPin2,  leftMotorPin2PWMChannel);

  rotateMotor(0, 0);
}

void setup() {
  setUpPinModes();
  // Set up servo (SG90) on pin 17 at 50Hz
  servo.attach(servoPin, 500, 2500);
  servo.setPeriodHertz(50);
  servo.write(90); // center (forward)

  Dabble.begin("MyBluetoothCar"); 
}

void loop() {
  Dabble.processInput();

  // Initial handshake: wait for any gamepad input to confirm connection
  if (!dabbleConnected) {
    if (GamePad.isUpPressed() || GamePad.isDownPressed() ||
        GamePad.isLeftPressed() || GamePad.isRightPressed() ||
        abs(GamePad.getXaxisData()) > ANALOG_THRESHOLD ||
        abs(GamePad.getYaxisData()) > ANALOG_THRESHOLD ||
        GamePad.isStartPressed() || GamePad.isSelectPressed()) {
      dabbleConnected = true;
    } else {
      rotateMotor(0, 0);
      return;
    }
  }

  // Toggle obstacle-avoidance mode
  if (GamePad.isStartPressed())  obstacleMode = true;
  if (GamePad.isSelectPressed()) obstacleMode = false;

  if (obstacleMode) {
    // --- Autonomous Obstacle Avoidance Mode ---
    long distance = measureDistanceCM();
    if (distance > OBSTACLE_THRESHOLD || distance == 0) {
      // Path clear: move forward
      servo.write(90); // ensure servo centered
      rotateMotor(MAX_MOTOR_SPEED/2.5, MAX_MOTOR_SPEED/2.5);
    } else {
      // Obstacle detected
      rotateMotor(0, 0);
      delay(0);

      // Back up slowly
      rotateMotor(-MAX_MOTOR_SPEED/2, -MAX_MOTOR_SPEED/2);
      delay(500);
      rotateMotor(0, 0);
      delay(100);

      // Scan left
      servo.write(30); // turn ultrasonic to left
      delay(300);
      long distLeft = measureDistanceCM();
      // Scan right
      servo.write(150); // turn ultrasonic to right
      delay(300);
      long distRight = measureDistanceCM();
      // Re-center servo
      servo.write(90);
      delay(300);

      bool leftClear  = (distLeft  > OBSTACLE_THRESHOLD) || (distLeft == 0);
      bool rightClear = (distRight > OBSTACLE_THRESHOLD) || (distRight == 0);

      // Decide turn direction
      if (leftClear && (!rightClear || distLeft >= distRight)) {
        // Turn left
        rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
        delay(TURN_DURATION);
        rotateMotor(0, 0);
      }
      else if (rightClear && (!leftClear || distRight > distLeft)) {
        // Turn right
        rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
        delay(TURN_DURATION);
        rotateMotor(0, 0);
      }
      else {
        // Both sides blocked: rotate ~180°
        rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
        // Rotate longer for ~180°; adjust if needed
        delay(1000);
        rotateMotor(0, 0);
      }
      // Small pause before next measurement
      delay(200);

      // After rotation, next loop iteration will measure distance again and move forward if clear.
    }
  }
  else {
    // --- Manual Control Mode (original Dabble joystick logic) ---
    int rightSpeed = 0, leftSpeed = 0;
    if (GamePad.isUpPressed()) {
      rightSpeed =  MAX_MOTOR_SPEED; leftSpeed =  MAX_MOTOR_SPEED;
    }
    else if (GamePad.isDownPressed()) {
      rightSpeed = -MAX_MOTOR_SPEED; leftSpeed = -MAX_MOTOR_SPEED;
    }
    else if (GamePad.isLeftPressed()) {
      rightSpeed =  MAX_MOTOR_SPEED; leftSpeed = -MAX_MOTOR_SPEED;
    }
    else if (GamePad.isRightPressed()) {
      rightSpeed = -MAX_MOTOR_SPEED; leftSpeed =  MAX_MOTOR_SPEED;
    }
    else {
      // Analog joystick control
      float xVal = GamePad.getXaxisData();
      float yVal = GamePad.getYaxisData();
      if (abs(xVal) < ANALOG_THRESHOLD && abs(yVal) < ANALOG_THRESHOLD) {
        rightSpeed = 0; leftSpeed = 0;
      }
      else if (abs(xVal) > abs(yVal)) {
        float factor = min(abs(xVal)/7.0, 1.0);
        if (xVal > 0) {
          rightSpeed = -MAX_MOTOR_SPEED * factor;
          leftSpeed  =  MAX_MOTOR_SPEED * factor;
        } else {
          rightSpeed =  MAX_MOTOR_SPEED * factor;
          leftSpeed  = -MAX_MOTOR_SPEED * factor;
        }
      }
      else {
        float factor = min(abs(yVal)/7.0, 1.0);
        if (yVal > 0) {
          rightSpeed =  MAX_MOTOR_SPEED * factor;
          leftSpeed  =  MAX_MOTOR_SPEED * factor;
        } else {
          rightSpeed = -MAX_MOTOR_SPEED * factor;
          leftSpeed  = -MAX_MOTOR_SPEED * factor;
        }
      }
    }
    rotateMotor(rightSpeed, leftSpeed);
  }
}
