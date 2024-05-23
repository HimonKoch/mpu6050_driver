/*
MPU 6050 I2C Device Driver
Author - Himon Koch
*/

#include "MPU6050.h"
#include <Wire.h>
#include <Arduino.h>

// CONSTRUCTOR
mpu6050_base::mpu6050_base() {
  Wire.begin();
};


// function to read a single byte from a register
uint8_t mpu6050_base::reg_read(reg_addr_t reg_addr) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg_addr);
  Wire.endTransmission();

  Wire.requestFrom(MPU6050_ADDR, 1);

  for (int tries = 0; tries < 5; tries++) {
    if (Wire.available()) {
      return Wire.read();
    };  
  };
  Serial.println("[-] FAILED TO READ REGISTER");
  return 0x00;
};

// function to write a single byte into a register
uint8_t mpu6050_base::reg_wite(reg_addr_t reg_addr, uint8_t data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg_addr);
  Wire.write(data);
  Wire.endTransmission();
  Serial.print("[+] DATA WRITEN INTO REGISTER NUM ");
  Serial.println(reg_addr, HEX);
};

// function to verify i2c communication between host and slave
returntype_t mpu6050_base::verify_comm() {
  return(reg_read(WHO_AM_I == MPU6050_ADDR)) ? SUCCESS : COMM_FAILURE;
}