#include"MPU6050.h"
#include <Arduino.h>

mpu6050_base::mpu6050_base(){
            initialized = false;
}

// private function to read a register
int mpu6050_base::read_register(uint8_t register_addr)  {
    if(initialized == false)    {
                Wire.begin();
                initialized = true;
    }

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(register_addr);
    Wire.endTransmission();

    Wire.requestFrom(MPU6050_ADDR, 1);

    int val = 0;

    if(Wire.available())  {
        Serial.println("wire available");
        val = Wire.read();
    }
    else    {
        Serial.println("wire unavailable");
    }

    return val;
}

// private function to write into a register
void mpu6050_base::write_register(uint8_t register_addr, uint8_t data)   {
    if(initialized == false)    {
        Wire.begin();
        initialized = true;
    }

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(register_addr);
    Wire.write(data);
    Wire.endTransmission();
}


// public function to read WHO_AM_I register to verify I2C communication
returntype mpu6050_base::verify_slave_communication()  {
    if(initialized == false)    {
        Wire.begin();
        initialized = true;
    }

    returntype ret = FAILURE;
    if(read_register(WHO_AM_I) == 0x68) {
        ret = SUCCESS;
    }

    return ret;
}


// public function to read gyro registers
void mpu6050_base::read_gyro(uint16_t * x_val, uint16_t * y_val, uint16_t * z_val)    {
    // checking of wire library is initiated
    if(initialized == false)    {
        Wire.begin();
        initialized = true;
    }

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission();

    Wire.requestFrom(MPU6050_ADDR, 6, true);

    if(Wire.available())    {
    *x_val = (Wire.read() << 8) | Wire.read();
    *y_val = (Wire.read() << 8) | Wire.read();
    *z_val = (Wire.read() << 8) | Wire.read();
    }

    else    {
        Serial.println("Wire unavailable");
    }
}


// public function to self test gyro
returntype mpu6050_base::selftest_gyro(){
    // checking of wire library is initiated
    if(initialized == false)    {
        Wire.begin();
        initialized = true;
    }

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(SELF_TEST_X);
    Wire.write(INIT_GYRO_SELF_TEST);
    Wire.endTransmission();

    delay(100);

    uint16_t x, y, z;
    read_gyro(&x, &y, &z);

    float x_response = x / 131.0;
    float y_response = y / 131.0;
    float z_response = z / 131.0;

    Serial.print("GYRO self test x value = ");
    Serial.println(x_response);
    Serial.print("GYRO self test y value = ");
    Serial.println(y_response);
    Serial.print("GYRO self test z value = ");
    Serial.println(z_response);

    return SUCCESS;
}