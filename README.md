# 🤖 ESP32 PID Line Following Robot

#

An autonomous line following robot built using an **ESP32 DevKit V1**, L298N motor driver, IR sensor array, and DC geared motors. The robot detects and follows a predefined track using sensor feedback and PID (Proportional, Integral, Derivative) control.

The main goal of this project is to achieve accurate line tracking, smooth movement, and reliable performance, especially on sharp turns and challenging track sections.

## 🚀 Project Features

- **ESP32 Based Control:** Uses ESP32 DevKit V1 as the main controller.
- **PID Control Algorithm:** Helps maintain accurate line tracking and reduces unnecessary movement.
- **IR Sensor Detection:** Detects the line and calculates the robot's position relative to it.
- **Motor Control:** Controls two DC geared motors through an L298N motor driver.
- **Sharp Turn Handling:** Designed to handle curves and sharp turns.
- **Adjustable Parameters:** Allows tuning of PID constants and motor speed.
- **Autonomous Operation:** Follows the track without continuous human control.

## 🛠️ Hardware Components

| Component                  |    Quantity |
| -------------------------- | ----------: |
| ESP32 DevKit V1 (30 Pin)   |           1 |
| L298N Motor Driver         |           1 |
| IR Sensor Array            |           1 |
| N20 DC Gear Motors         |           2 |
| Robot Chassis (12 × 15 cm) |           1 |
| Robot Wheels               |           2 |
| Caster Wheel               |           1 |
| Battery Pack               |           1 |
| Jumper Wires               | As required |
| Power Switch               |           1 |

## ⚙️ Working Principle

1. The IR sensor array detects the line on the track.
2. The sensors send readings to the ESP32.
3. The ESP32 calculates the robot's position relative to the line.
4. The PID algorithm calculates the correction required to stay on track.
5. The ESP32 sends control signals to the L298N motor driver.
6. The motors adjust their speeds to keep the robot following the line.

## 🧠 PID Control

PID stands for:

- **Proportional (P):** Corrects the current tracking error.
- **Integral (I):** Helps reduce accumulated error over time.
- **Derivative (D):** Helps reduce sudden changes and overshooting.

The PID controller calculates the correction using:

```cpp
error = setPoint - position;

integral += error;

derivative = error - previousError;

correction = (Kp * error) +
             (Ki * integral) +
             (Kd * derivative);

previousError = error;
```

The values of `Kp`, `Ki`, and `Kd` need to be tuned according to the robot, motors, sensor array, and track.

## 🔌 System Architecture

```text
       IR Sensor Array
              |
              v
        ESP32 DevKit V1
              |
        PID Algorithm
              |
              v
       L298N Motor Driver
           /       \
          v         v
      Left Motor  Right Motor
          |         |
          +----+----+
               |
        Robot Movement
```

## 💻 Software and Technologies

- Arduino IDE
- Embedded C++
- ESP32
- PID Control Algorithm
- IR Sensor Processing
- Motor Driver Control

## 🔧 Building and Setup

1. Assemble the robot chassis and attach the motors and wheels.
2. Mount the IR sensor array at a suitable height above the track.
3. Connect the motors to the L298N motor driver.
4. Connect the motor driver control pins to the ESP32.
5. Connect the IR sensor outputs to the ESP32 according to the pin configuration in the source code.
6. Connect the power supply with correct voltage and polarity.
7. Install the ESP32 board package in Arduino IDE.
8. Open the project source code.
9. Select the correct ESP32 board and COM port.
10. Upload the firmware.
11. Test the sensor readings and calibrate the sensors.
12. Tune the PID parameters and motor speeds until the robot follows the track reliably.

**Important:** Check the actual pin configuration and power requirements of your components before connecting them. Do not connect 5V signals directly to ESP32 GPIO pins.

## 📈 Future Improvements

- Improved PID tuning for better accuracy.
- Automatic sensor calibration.
- Better intersection detection.
- Recovery logic when the robot loses the line.
- Battery voltage monitoring.
- Speed optimization.
- Testing on more complex tracks.

## 🎯 Learning Outcomes

Through this project, I explored:

- ESP32 microcontroller programming.
- Embedded C++ development.
- Sensor interfacing and calibration.
- DC motor control using L298N.
- PID control and feedback systems.
- Hardware and software integration.
- Autonomous robotics.

## 👨‍💻 Developer

**Wizard Developer**

Electronics and Computing Student
COMSATS University Islamabad

GitHub: [WIzard-developerr](https://github.com/WIzard-developerr)

LinkedIn: [Connect with me](https://www.linkedin.com/in/muhammad-anas-7aa957241/)

---

⭐ If you find this project interesting, consider giving the repository a star!

**Built with curiosity, electronics, and code.**
