#include"MPU6050.h"
#include <Arduino.h>

mpu6050_base::mpu6050_base(clk_sel_t clk = CLK_SEL_INTERNAL, dlpf_cfg_t cfg = DLPF_CFG_0) {
            initialized = false;
            clock_config(clk);
            dlpf_config(cfg);
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
    returntype ret = FAILURE;
    if(read_register(WHO_AM_I) == 0x68) {
        ret = SUCCESS;
    }

    return ret;
}


// private function to set the sample rate
returntype mpu6050_base::set_sample_rate(int rate)    {

    if ((rate < 31250) || (rate > 8000000)) {
        return FAILURE;
    }
    uint8_t rate_input = (8000000 / rate) - 1;

    write_register(SMPLRT_DIV, rate_input);

    return SUCCESS;
}

// private function to set the gyro sensitivity
returntype mpu6050_base::set_gyro_sensitivity(gyro_range_t range)  {
    write_register(GYRO_CONFIG, range);
    return SUCCESS;
}

// private function to set the gyro sensitivity
returntype mpu6050_base::set_accl_sensitivity(accel_range_t range)  {
    write_register(GYRO_CONFIG, range);
    return SUCCESS;
}


// pyblic function to setup the gyro for measurement
returntype mpu6050_base::gyro_setup(int rate, gyro_range_t range)    {
    if ((set_sample_rate(rate) == SUCCESS) && (set_gyro_sensitivity(range) == SUCCESS)) {
        gyro_range = range;
        return SUCCESS;
    }
    else {
        return FAILURE;
    }
}

// pyblic function to setup the gyro for measurement
returntype mpu6050_base::accl_setup(int rate, accel_range_t range)    {
    if ((set_sample_rate(rate) == SUCCESS) && (set_accl_sensitivity(range) == SUCCESS)) {
        accl_range = range;
        return SUCCESS;
    }
    else {
        return FAILURE;
    }
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

    float lsb_sensitivity;

    switch (gyro_range) {
        case GYRO_FULLRANGE_250: {
            lsb_sensitivity = 131.0;
            break;
        }
        case GYRO_FULLRANGE_500: {
            lsb_sensitivity = 65.5;
            break;
        }

        case GYRO_FULLRANGE_1K: {
            lsb_sensitivity = 32.8;
            break;
        }

        case GYRO_FULLRANGE_2K: {
            lsb_sensitivity = 16.4;
            break;
        }
    }

    Serial.print("$$$$$$$$");
    Serial.println(lsb_sensitivity);

    if(Wire.available())    {
    *x_val = (Wire.read() << 8) | Wire.read();
    *y_val = (Wire.read() << 8) | Wire.read();
    *z_val = (Wire.read() << 8) | Wire.read();
    }

    else    {
        Serial.println("Wire unavailable");
    }
}

// public function to read accelerometer registers
void mpu6050_base::read_accel(uint16_t * x_val, uint16_t * y_val, uint16_t * z_val)    {
    // checking of wire library is initiated
    if(initialized == false)    {
        Wire.begin();
        initialized = true;
    }

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission();

    Wire.requestFrom(MPU6050_ADDR, 6, true);

    float lsb_sensitivity;

    switch (accl_range) {
        case ACCL_FULLRANGE_2G: {
            lsb_sensitivity = 16384;
            break;
        }
        case ACCL_FULLRANGE_4G: {
            lsb_sensitivity = 8192;
            break;
        }

        case ACCL_FULLRANGE_8G: {
            lsb_sensitivity = 4096;
            break;
        }

        case ACCL_FULLRANGE_16G: {
            lsb_sensitivity = 2048;
            break;
        }
    }

    if(Wire.available())    {
    *x_val = ((Wire.read() << 8) | Wire.read()) / lsb_sensitivity;
    *y_val = ((Wire.read() << 8) | Wire.read()) / lsb_sensitivity;
    *z_val = ((Wire.read() << 8) | Wire.read()) / lsb_sensitivity;
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

// private function to specify clock source
void mpu6050_base::clock_config(clk_sel_t clk) {
    write_register(PWR_MGMT_1, clk);
    clk_config = clk;
}

// private function to configure the DLPF
void mpu6050_base::dlpf_config(dlpf_cfg_t cfg) {
    write_register(CONFIG, cfg);
    dlpf_cfg = cfg;
}