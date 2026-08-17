# Self-Balancing Robot (Full-Stack Embedded Project)

An independent full-stack robotics project featuring custom PCB hardware, C/C++ embedded firmware, and PID control algorithms on STM32 Microcontroller.

## 📌 Overview
This project focuses on designing and building an autonomous self-balancing two-wheeled robot from scratch. It integrates hardware design, sensor fusion, discrete control theory, and embedded systems programming.

## 🛠️ System Architecture & Features
- **Microcontroller:** STM32F103 Series.
- **Hardware & PCB:** Custom 2-layer PCB designed with onboard MPU6050 IMU and Motor Driver.
- **Firmware:** Embedded C/C++ using STM32 HAL/Register-level programming, Timer Interrupts, and I2C/PWM peripherals.
- **Control Strategy:** 
  - Sensor fusion via **Complementary/Kalman Filter** for stable pitch/roll estimation.
  - **Cascade PID Algorithm** for position, velocity, and tilt balancing.
- **Mechanical:** Custom 3D-printed chassis designed in SolidWorks.

## 📂 Project Structure
```text
self-balancing-robot/
├── firmware/     # STM32 Core, Drivers, and IOC config
├── hardware/     # Schematics, Wiring, PCB, and BOM
├── mechanical/   # SolidWorks CAD, STL, and Dimension files
├── control/      # Block diagrams, Calculations, and PID tuning
├── data/         # Raw, processed, and telemetry logs
├── scripts/      # Python scripts for telemetry and plotting
├── docs/         # System documentation, calibration & PID tuning guides
└── images/       # Project photos, diagrams, and plots
