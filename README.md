# 🚘 ESP32 Bluetooth Car with Dabble (Joystick, Accelerometer & Obstacle Avoidance)

This is a **4-motor Bluetooth-controlled robot car** powered by an **ESP32 (38-pin)** and controlled through the **Dabble mobile app**. It supports **Joystick Mode**, **Accelerometer Mode**, and **Obstacle Avoidance** using an **HC-SR04 ultrasonic sensor**. The motors are driven using a **DRV8833 motor driver**, and the sensor scans the area using a **servo motor**.

---

## 🔧 Hardware Used

| Component            | Description                                  |
| -------------------- | -------------------------------------------- |
| ESP32 Dev Board      | 38-pin (Board version: 2.x.x required)       |
| DRV8833 Motor Driver | 4 BO Motors (controlled via PWM on IN pins)  |
| HC-SR04              | Ultrasonic distance sensor                   |
| Servo Motor          | For ultrasonic scanning (uses `ESP32Servo`)  |
| Li-ion/LiPo Battery  | 7.4V recommended                             |
| Chassis & wheels     | for mounting hardware                        |

---

## 📱 Mobile App: Dabble

Control your car using **Dabble** –:

* Available on Android: [Dabble on Play Store](https://play.google.com/store/apps/details?id=io.dabbleapp)
* Uses Bluetooth communication with the ESP32

### ✅ Supported Dabble Modes:

| Mode          | Feature                                       |
| ------------- | --------------------------------------------- |
| Joystick      | Move in all directions using virtual joystick |
| Accelerometer | Tilt phone to move the car                    |
| Gamepad       | Switch to obstacle avoidance / manual mode    |

---

## 🫠 Features

* ✅ Bluetooth control via Dabble App
* ✅ Dual control modes: **Joystick** & **Accelerometer**
* ✅ **Obstacle Avoidance Mode** using HC-SR04 + Servo
* ✅ **PWM motor control** (DRV8833-Motor Driver)
* ✅ Built-in Deadzone to avoid joystick drift
* ✅ No library used for HC-SR04 – custom echo & trig logic

---

## ⚙️ Wiring Details

### 🚗 Motor & Control Pins (4 motors via 2 DRV8833s):

| Motor           | ESP32 Pin |
| --------------- | --------- |
| Right Motor IN1 | 14        |
| Right Motor IN2 | 27        |
| Left Motor IN1  | 26        |
| Left Motor IN2  | 25        |

### 💥 Motor Sleep Pin:

| Sleep Pin (optional) | ESP32 Pin |
| -------------------- | --------- |
| DRV8833 SLP          | 20        |

### 📱 Ultrasonic Sensor:

| Function | ESP32 Pin |
| -------- | --------- |
| Trigger  | 5         |
| Echo     | 18        |

### 🔁 Servo Motor:

| Servo Pin | ESP32 Pin |
| --------- | --------- |
| Signal    | 17        |

---

## 🔌 PWM Configuration

| Motor Channel | PWM Channel |
| ------------- | ----------- |
| Right IN1     | 4           |
| Right IN2     | 5           |
| Left IN1      | 6           |
| Left IN2      | 7           |

* Frequency: `1000 Hz`
* Resolution: `8-bit`
* Max speed: `255`

---

## 🧮 Logic Overview

* **Manual Control**:

  * **Joystick Mode**: Move car using Dabble joystick
  * **Accelerometer Mode**: Tilt phone to control movement
* **Obstacle Avoidance Mode**:

  * Triggered via **Start button**
  * Uses **HC-SR04** to detect obstacles in front
  * Scans left/right with servo
  * Decides the safer direction and moves accordingly
  * Can be disabled using **Select button**

---

## 📜 Sample Constants

```cpp
#define MAX_MOTOR_SPEED 255
const int PWMFreq = 1000;
const int PWMResolution = 8;
const int rightMotorPin1PWMChannel = 4;
const int rightMotorPin2PWMChannel = 5;
const int leftMotorPin1PWMChannel  = 6;
const int leftMotorPin2PWMChannel  = 7;

const float ANALOG_THRESHOLD = 2.0; // Deadzone for joystick
const int OBSTACLE_THRESHOLD = 25; // in cm
const unsigned long TURN_DURATION = 600; // ms
```

---

## 🛠️ Setup Instructions

1. **Install Arduino IDE**
2. Install ESP32 board support via **Board Manager**

   * Board version: **2.x.x**
3. Install required libraries:

   * `ESP32Servo` (for controlling servo motor)
   * `DabbleESP32` *(optional if using Serial parsing manually)*
4. Upload code via USB
5. Open **Dabble App**, connect via Bluetooth, and start controlling!

---

## 📦 Files Included

* `main.ino` – Arduino sketch
* `README.md` – This documentation

---

## 🧑‍💻 Developed By

**Vishnu Kumar Dhoni**
STEM Educator | Robotics & Embedded Systems Developer
*IIT Patna*

---

## 📝 License

This project is licensed under the [MIT License](https://opensource.org/licenses/MIT).
Free to use, modify, and distribute with attribution.

---

## 🔗 Useful Links

* [Dabble App (Play Store)](https://play.google.com/store/apps/details?id=io.dabbleapp)
* [DRV8833 Datasheet](https://www.ti.com/lit/ds/symlink/drv8833.pdf)
* [ESP32Servo GitHub](https://github.com/madhephaestus/ESP32Servo)
* [ESP32 Arduino Core Docs](https://docs.espressif.com/projects/arduino-esp32/en/latest/)

---

## 📷 Demo Preview


