#include <Arduino.h>
#include "MPU6050.h"

mpu6050_base device_obj;

void setup() {
  Serial.begin(115200);
}

void loop() {
  returntype_t data = device_obj.verify_comm();
  if (data == SUCCESS)
    Serial.println("[+] COMM SUCCESSFUL");
  else
    Serial.println("[-] COMM FAILED");
  delay(500);
}
