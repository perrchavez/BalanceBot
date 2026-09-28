# BalanceBot

ESP32 Bluetooth-controlled self-balancing robot using a BNO055 IMU, quadrature encoders, and cascaded feedback control.

## Features

- Self-balancing using IMU feedback
- Complementary-filter attitude estimation
- Inner PID balance controller
- Outer velocity controller using encoder feedback
- Differential turning control
- Bluetooth speed and steering commands
- Motor dead-zone compensation
- Fall protection

## Hardware

- ESP32
- BNO055 IMU
- 12 V DC gear motors with quadrature encoders
- DRV8871 motor drivers
- 3S Li-ion battery

## Software

Firmware is written in C using ESP-IDF.

The `main/` directory contains the embedded firmware, including IMU communication, orientation estimation, motor control, encoder feedback, Bluetooth control, and feedback controllers.

## Results

The robot has demonstrated more than four hours of continuous balancing and can transition between carpet and hard flooring while being controlled over Bluetooth.
