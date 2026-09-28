# BalanceBot

ESP32 Bluetooth-controlled self-balancing robot using a BNO055 IMU, quadrature encoders, and cascaded feedback control.

![BalanceBot](media/balancebot.jpg)

## Demo

[Watch the BalanceBot demo video](https://youtu.be/JyW7P2Z0T2w)

The demo shows the robot balancing, driving, turning, and transitioning between carpet and hard flooring while responding to Bluetooth control commands.

## Features

- Self-balancing using IMU feedback
- Complementary-filter attitude estimation
- Inner PID balance controller
- Outer PI velocity controller using encoder feedback
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
- Custom Onshape CAD and 3D-printed mechanical components

## Software and Control

Firmware is written in C using ESP-IDF.

The `main/` directory contains the embedded firmware for:

- IMU communication
- Orientation estimation
- Motor control
- Encoder feedback
- Bluetooth communication
- Balance, velocity, and turning controllers

The robot uses a cascaded control architecture with an inner PID balance loop, an outer PI velocity loop, and differential turning control using encoder feedback.

## Results

The robot demonstrated more than four hours of continuous balancing and can transition between carpet and hard flooring while being controlled over Bluetooth.
