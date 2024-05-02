#include <Arduino.h>
#include "MPU6050.h"

// put function declarations here:
mpu6050_base device_obj;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  // uint8_t ret = device_obj.read_register(WHO_AM_I);
  // Serial.println(ret);
  // delay(100);
  // CONFIG THE SAMPLE RATE
  // device_obj.write_register(CONFIG, 0b0000000);
  // device_obj.write_register(SMPLRT_DIV, 159);
  // device_obj.write_register(PWR_MGMT_1, DEVICE_RESET_NOSLEEP_NOCYCLE);
  // uint8_t ret = device_obj.read_register(PWR_MGMT_1);
  // Serial.println(ret);

  // uint16_t x, y, z;

  // while(1)  {
  //   device_obj.read_gyro(&x, &y, &z);

  //   Serial.print(x);
  //   Serial.print("\t");
  //   Serial.print(y);
  //   Serial.print("\t");
  //   Serial.print(z);
  //   Serial.print("\t");

  //   Serial.println("\n");

  //   delay(100);
  // }

  returntype ret = device_obj.selftest_gyro();
  delay(500);
}
