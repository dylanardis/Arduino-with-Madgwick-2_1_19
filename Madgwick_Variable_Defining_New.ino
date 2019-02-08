//-------------------------------------------------------------------------------
//  TinyCircuits LSM9DS1 9 Axis TinyShield Example Sketch
//  Last Updated 11 July 2016
//
//  This demo is intended for the ASD2511 Sensor Board TinyShield with a LSM9DS1
//  9 axis sensor populated. It shows basic use of a modified RTIMULib with the
//  sensor.
//
//  This program now includes an EEPROM compatibility file for TinyScreen+.
//  Using it will lock the last 16KB of flash for EEPROM emulation and prevent
//  the bootloader from erasing or writing that section. This should not affect
//  other programs unless they are trying to use the last 16KB of flash.
//
//  Modified by Ben Rose for TinyCircuits, https://tinycircuits.com
//
//-------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014-2015, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
////////////////////////////////////////////////////////////////////////////


#include <Wire.h>
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"
#ifndef ARDUINO_ARCH_SAMD
#include <EEPROM.h>
#endif

RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

//  DISPLAY_INTERVAL sets the rate at which results are displayed

#define DISPLAY_INTERVAL  300                         // interval between pose displays

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

#ifdef SERIAL_PORT_MONITOR
#define SerialMonitor SERIAL_PORT_MONITOR
#else
#define SerialMonitor Serial
#endif
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value


unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;
float deltat = 0.0f;
uint32_t lastUpdate = 0;    // used to calculate integration interval
uint32_t Now = 0;           // used to calculate integration interval

float q[4] = {1, 0, 0, 0};

float ax;
float ay;
float az;

float gx;
float gy;
float gz;

float mx;
float my;
float mz;


void setup()
{
  int errcode;

  SerialMonitor.begin(SERIAL_PORT_SPEED);
  while (!SerialMonitor);

  Wire.begin();
  imu = RTIMU::createIMU(&settings);                        // create the imu object

  //SerialMonitor.print("ArduinoIMU starting using device "); SerialMonitor.println(imu->IMUName());
  if ((errcode = imu->IMUInit()) < 0) {
    SerialMonitor.print("Failed to init IMU: "); SerialMonitor.println(errcode);
  }

  if (imu->getCalibrationValid());
  //SerialMonitor.println("Using compass calibration");
  else
    //SerialMonitor.println("No valid compass calibration data");

    lastDisplay = lastRate = millis();
  sampleCount = 0;

  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.

  fusion.setSlerpPower(0.02);

  // use of sensors in the fusion algorithm can be controlled here
  // change any of these to false to disable that sensor

  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);
}

void loop()
{
  unsigned long now = millis();
  unsigned long delta;

  if (imu->IMURead()) {                                // get the latest data if ready yet
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    sampleCount++;
    RTVector3 accelData = imu->getAccel();
    RTVector3 gyroData = imu->getGyro();
    RTVector3 compassData = imu->getCompass();
    ax = accelData.x();
    ay = accelData.y();
    az = accelData.z();
    gx = gyroData.x();
    gy = gyroData.y();
    gz = gyroData.z();
//    mx = compassData.x();
//    my = compassData.y();
//    mz = compassData.z();
    mx = 0.0233*compassData.x()+ 0.22065;
    my = 0.0233*compassData.y()- 0.554832;
    mz = 0.0233*compassData.z()+ 0.37301;

     Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
    MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f, mx, my, mz);

    if ((delta = now - lastRate) >= 1000) {
      //SerialMonitor.print("Sample rate: "); SerialMonitor.print(sampleCount);
      if (imu->IMUGyroBiasValid());
      //SerialMonitor.println(", gyro bias valid");
      else
        //SerialMonitor.println(", calculating gyro bias - don't move IMU!!");

        sampleCount = 0;
      lastRate = now;
    }
    if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
      lastDisplay = now;
      // RTVector3 accelData = imu->getAccel();
      // RTVector3 gyroData = imu->getGyro();
      // RTVector3 compassData = imu->getCompass();
      //RTVector3 fusionData=fusion.getFusionPose();
      
      //displayAxis("Accel:", accelData.x(), accelData.y(), accelData.z());        // accel data
      //displayAxis("correctedAccel:", ax, ay, az);
      //Serial.print(ax);
      //Serial.print(ay);
      //Serial.print(az);
      
      //displayAxis("Gyro:", gyroData.x(), gyroData.y(), gyroData.z()); // gyro data
      //displayAxis("correctedGyro:", gx, gy, gz);
      
      //displayAxis("Mag:", compassData.x(), compassData.y(), compassData.z());    // compass data     
      //displayAxis("correctedMag:", mx, my, mz);
      //Serial.print(mx);
      //Serial.print(my);
      //Serial.print(mz);
      
      //displayDegrees(fusionData.x(), fusionData.y(), fusionData.z());   // fused output
      
      Serial.print(q[0]);
      Serial.print(" , ");
      Serial.print(q[1]);
      Serial.print(" , ");
      Serial.print(q[2]);
      Serial.print(" , ");
      Serial.print(q[3]);

      SerialMonitor.println();
    }
  }
}





void displayDegrees(const char *label, float x, float y, float z)
{

  SerialMonitor.print(x * RTMATH_RAD_TO_DEGREE); SerialMonitor.print(",");
  SerialMonitor.print(y * RTMATH_RAD_TO_DEGREE); SerialMonitor.print(",");
  SerialMonitor.print(z * RTMATH_RAD_TO_DEGREE); SerialMonitor.print("/n");
}



void displayAxis(const char *label, float x, float y, float z)
{
  SerialMonitor.print(label);
  SerialMonitor.print(" x:"); SerialMonitor.print(x);
  SerialMonitor.print(" y:"); SerialMonitor.print(y);
  SerialMonitor.print(" z:"); SerialMonitor.print(z);
}



void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{

  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;



  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  mx *= norm;
  my *= norm;
  mz *= norm;



  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f / norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;



  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

 

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

//  Serial.print(" deltat = ");
//  Serial.print(deltat);
//  Serial.print(" norm = ");
//  Serial.print(norm);
//
//  Serial.print(" q1 = ");
//  Serial.print(q[0]);
//  Serial.print(" q2 = ");
//  Serial.print(q[1]);
//  Serial.print(" q3 = ");
//  Serial.print(q[2]);
//  Serial.print(" q4 = ");
//  Serial.print(q[3]);
//
//  Serial.print(" s1 = ");
//  Serial.print(s1);
//  Serial.print(" s2 = ");
//  Serial.print(s2);
//  Serial.print(" s3 = ");
//  Serial.print(s3);
//  Serial.print(" s4 = ");
//  Serial.print(s4);
//
//  Serial.print(" gx = ");
//  Serial.print(gx);
//  Serial.print(" gy = ");
//  Serial.print(gy);
//  Serial.print(" gz = ");
//  Serial.print(gz);
//
//  Serial.print(" qDot1 = ");
//  Serial.print(qDot1);
//  Serial.print(" qDot2 = ");
//  Serial.print(qDot2);
//  Serial.print(" qDot3 = ");
//  Serial.print(qDot3);
//  Serial.print(" qDot4 = ");
//  Serial.println(qDot4);

}
























/*void sendFloat(float f) {
  byte * b = (byte *) &f;
  Serial.print("f:");
  Serial.write(b[0]);
  Serial.write(b[1]);
  Serial.write(b[2]);
  Serial.write(b[3]);
  Serial.print(68); //Send nonsense.. Else serial drops offline??
  Serial.flush();
  return;
  }

  void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  Serial.print("f:");
  for (int i = 0; i < 4; i++) {

    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);

    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

    Serial.print(c1);
    Serial.print(c2);
  }
  }

  void displayDegrees(float x, float y, float z)
  {
  //sendFloat(qs);
  float Vals[] = {x, y, z};
  //float Vals[] = {qs, qx, qy, qz};
  int i;
  for (i = 0; i < 4; i = i + 1) {
    serialFloatPrint(Vals[i]);
    if (i <= 2)
      Serial.write(',');
    if (i == 3)
      Serial.write('\n');



  }  }

*/
