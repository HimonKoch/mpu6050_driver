#include <Arduino.h>
#include "MPU6050.h"
#include <Adafruit_MPU6050.h>

// put function declarations here:
mpu6050_base device_obj(CLK_SEL_INTERNAL, DLPF_CFG_0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("CODE START");
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

  uint16_t x, y, z;
  if(device_obj.gyro_setup(500000, GYRO_FULLRANGE_250) == SUCCESS) {
    Serial.println("setup successful");
  }
  else {
    Serial.println("setup Failed");
  }

  while(1)  {
    device_obj.read_gyro(&x, &y, &z);

    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.print(z);
    Serial.print("\t");

    Serial.println("\n");

    delay(100);
  }

}


// Basic demo for accelerometer readings from Adafruit MPU6050

// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Wire.h>

// Adafruit_MPU6050 mpu;

// void setup(void) {
//   Serial.begin(9600);
//   while (!Serial)
//     delay(10); // will pause Zero, Leonardo, etc until serial console opens

//   Serial.println("Adafruit MPU6050 test!");

//   // Try to initialize!
//   if (!mpu.begin()) {
//     Serial.println("Failed to find MPU6050 chip");
//     while (1) {
//       delay(10);
//     }
//   }
//   Serial.println("MPU6050 Found!");

//   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//   Serial.print("Accelerometer range set to: ");
//   switch (mpu.getAccelerometerRange()) {
//   case MPU6050_RANGE_2_G:
//     Serial.println("+-2G");
//     break;
//   case MPU6050_RANGE_4_G:
//     Serial.println("+-4G");
//     break;
//   case MPU6050_RANGE_8_G:
//     Serial.println("+-8G");
//     break;
//   case MPU6050_RANGE_16_G:
//     Serial.println("+-16G");
//     break;
//   }
//   mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//   Serial.print("Gyro range set to: ");
//   switch (mpu.getGyroRange()) {
//   case MPU6050_RANGE_250_DEG:
//     Serial.println("+- 250 deg/s");
//     break;
//   case MPU6050_RANGE_500_DEG:
//     Serial.println("+- 500 deg/s");
//     break;
//   case MPU6050_RANGE_1000_DEG:
//     Serial.println("+- 1000 deg/s");
//     break;
//   case MPU6050_RANGE_2000_DEG:
//     Serial.println("+- 2000 deg/s");
//     break;
//   }

//   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
//   Serial.print("Filter bandwidth set to: ");
//   switch (mpu.getFilterBandwidth()) {
//   case MPU6050_BAND_260_HZ:
//     Serial.println("260 Hz");
//     break;
//   case MPU6050_BAND_184_HZ:
//     Serial.println("184 Hz");
//     break;
//   case MPU6050_BAND_94_HZ:
//     Serial.println("94 Hz");
//     break;
//   case MPU6050_BAND_44_HZ:
//     Serial.println("44 Hz");
//     break;
//   case MPU6050_BAND_21_HZ:
//     Serial.println("21 Hz");
//     break;
//   case MPU6050_BAND_10_HZ:
//     Serial.println("10 Hz");
//     break;
//   case MPU6050_BAND_5_HZ:
//     Serial.println("5 Hz");
//     break;
//   }

//   Serial.println("");
//   delay(100);
// }

// void loop() {

//   /* Get new sensor events with the readings */
//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);

//   // /* Print out the values */
//   // Serial.print("Acceleration X: ");
//   // Serial.print(a.acceleration.x);
//   // Serial.print(", Y: ");
//   // Serial.print(a.acceleration.y);
//   // Serial.print(", Z: ");
//   // Serial.print(a.acceleration.z);
//   // Serial.println(" m/s^2");

//   Serial.print("Rotation X: ");
//   Serial.print(g.gyro.x);
//   Serial.print(", Y: ");
//   Serial.print(g.gyro.y);
//   Serial.print(", Z: ");
//   Serial.print(g.gyro.z);
//   Serial.println(" rad/s");

//   // Serial.print("Temperature: ");
//   // Serial.print(temp.temperature);
//   // Serial.println(" degC");

//   //Serial.println("");
//   delay(100);
// }
