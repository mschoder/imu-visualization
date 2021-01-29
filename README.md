# IMU Visualization

A simple project to visualize absolute orientation (AHRS) using an MPU-9250 IMU + Magnetometer sensor and an on-device implementation of the Madgwick filter on an Arduino Uno board. Visualization is done in the browser using Three.js after reading serial data from the Arduino and passing via websocket.

![](viz/app_ui.png)

References: 
- [Madgwick Filter Paper](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwj0jfjVxcDuAhUyoFsKHX_LBC8QFjAAegQIARAC&url=https%3A%2F%2Fwww.x-io.co.uk%2Fres%2Fdoc%2Fmadgwick_internal_report.pdf&usg=AOvVaw3AS1JXKz1SJfskDlAi_Vji)
- [SparkFun MPU-9250 Madgwick implementation for Arduino](https://learn.sparkfun.com/tutorials/mpu-9250-hookup-guide/all)
- [Kris Winer's comprehensive MPU 9250 reference and Arduino code](https://github.com/kriswiner/MPU9250)
- [Example Three.js code for use with IMU](https://github.com/drquinn/imu-visualize)


