#ifndef BMX055_lib
#define BMX055_lib

#include "Arduino.h"
#include "BMX055_Minimal_Lib.h"
#include <Wire.h>

// BMX055 Accl I2C address is 0x18(24)
#define Addr_Accl 0x18
// BMX055 Gyro I2C address is 0x68(104)
#define Addr_Gyro 0x68
// BMX055 Mag I2C address is 0x10(16)
#define Addr_Mag 0x10

BMX055::BMX055(){
  
}

void BMX055::init(){
  
  Wire.begin();
    // Start I2C Transmission
  Wire.beginTransmission(Addr_Accl);
  // Select PMU_Range register
  Wire.write(0x0F);
  // Range = +/- 8g
  Wire.write(0x08);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Accl);
  // Select PMU_BW register
  Wire.write(0x10);
  // Bandwidth = 500 Hz  ****
  Wire.write(0x0E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Accl);
  // Select PMU_LPW register
  Wire.write(0x11);
  // Normal mode, Sleep duration = 0.5ms
  Wire.write(0x00);
  // Stop I2C Transmission on the device
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Gyro);
  // Select Range register
  Wire.write(0x0F);
  // Full scale = +/- 1000 degree/s
  Wire.write(0x01);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Gyro);
  // Select Bandwidth register
  Wire.write(0x10);
  // ODR = 100 Hz ****
  Wire.write(0x02);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Gyro);
  // Select LPM1 register
  Wire.write(0x11);
  // Normal mode, Sleep duration = 2ms
  Wire.write(0x00);
  // Stop I2C Transmission
  Wire.endTransmission();


    // Start I2C Transmission
  Wire.beginTransmission(Addr_Mag);
  // Select Mag register
  Wire.write(0x4B);
  // Soft reset
  Wire.write(0x00);
  // Stop I2C Transmission
  Wire.endTransmission();
  
  delay(100);

    // Start I2C Transmission
  Wire.beginTransmission(Addr_Mag);
  // Select Mag register
  Wire.write(0x4B);
  // Soft reset
  Wire.write(0x82);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(100);

  Wire.beginTransmission(Addr_Mag);
  // Select Mag register
  Wire.write(0x4B);
  // Soft reset
  Wire.write(0x01);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(300);

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Mag);
  // Select Mag register
  Wire.write(0x4C);
  // Normal Mode, ODR = 10 Hz
  Wire.write(0x00);
  // Stop I2C Transmission
  Wire.endTransmission();

//  // Start I2C Transmission
//  Wire.beginTransmission(Addr_Mag);
//  // Select Mag register
//  Wire.write(0x4E);
//  // X, Y, Z-Axis enabled
//  Wire.write(0x84);
//  // Stop I2C Transmission
//  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Mag);
  // Select Mag register
  Wire.write(0x51);
  // No. of Repetitions for X-Y Axis = 9
  Wire.write(0x04);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Mag);
  // Select Mag register
  Wire.write(0x52);
  // No. of Repetitions for Z-Axis = 15
  Wire.write(0x0F);
  // Stop I2C Transmission
  Wire.endTransmission();
  
  delay(300);
}

void BMX055::getSensorData(byte* data)
{
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x02);
  Wire.endTransmission(0);

 
  Wire.requestFrom(Addr_Accl, 6);
  
  for (int i = 0; i < 6; i++)
  {
      *(data+i) = Wire.read();   
  }
  
  // Start I2C Transmission
  Wire.beginTransmission(Addr_Gyro);
  // Select data register
  Wire.write(0x02);
    // Stop I2C Transmission
  Wire.endTransmission(0);
  
  // Request 1 byte of data
  Wire.requestFrom(Addr_Gyro, 6);
  for (int i = 6; i < 12; i++)
  {
      *(data+i) = Wire.read();
  }

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Mag);
  // Select data register
  Wire.write(66);
    // Stop I2C Transmission
  Wire.endTransmission(0);
  
  // Request 1 byte of data
  Wire.requestFrom(Addr_Mag, 6);
  for (int i = 12; i < 18; i++)
  {
      *(data+i) = Wire.read();
  }
}

#endif
