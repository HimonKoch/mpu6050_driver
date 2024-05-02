/*
MPU 6050 I2C Device Driver
Author - Himon Koch
*/



#include <stdint.h>
#include <Wire.h>

/*---- SELF TEST REGISTERS --------------------*/
#define SELF_TEST_X         0x0D
#define SELF_TEST_Y         0x0E
#define SELF_TEST_Z         0x0F
#define SELF_TEST_A         0x10

/*---- CONFIG REGISTERS -----------------------*/
#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define FIFO_EN             0x23

/*---- I2C REGISTERS --------------------------*/
#define I2C_MST_CTRL        0x24
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV0_DO         0x63
#define I2C_SLV1_ADDR       0x28
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A
#define I2C_SLV2_ADDR       0x2B
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_ADDR       0x2C
#define I2C_SLV2_REG        0x2D
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_ADDR       0x2F
#define I2C_SLV3_REG        0x30
#define I2C_SLV3_DO         0x66
#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36
#define I2C_MST_DELAY_CT_RL 0x67

/*---- INTERRUPT REGISTERS --------------------*/
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define INT_STATUS          0x3A

/*---- ACCELEROMETER READINGS REGISTERS -------*/
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40

/*---- TEMPERATURE READINGS REGISTERS ---------*/
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42

/*---- GYROSCOPE READINGS REGISTERS -----------*/
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48

/*---- EXTERNAL SENSOR REGISTERS --------------*/
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54
#define EXT_SENS_DATA_12    0x55
#define EXT_SENS_DATA_13    0x56
#define EXT_SENS_DATA_14    0x57
#define EXT_SENS_DATA_15    0x58
#define EXT_SENS_DATA_16    0x59
#define EXT_SENS_DATA_17    0x5A
#define EXT_SENS_DATA_18    0x5B
#define EXT_SENS_DATA_19    0x5C
#define EXT_SENS_DATA_20    0x5D
#define EXT_SENS_DATA_21    0x5E
#define EXT_SENS_DATA_22    0x5F
#define EXT_SENS_DATA_23    0x60

#define SIGNAL_PATH_RESET   0x68
#define USER_CTRL           0x6A
#define PWR_MGMT_1          0x6B
#define PWR_MGMT_2          0x6C

#define FIFO_COUNTH         0x72
#define FIFO_COUNTL         0x73
#define FIFO_R_W            0x74
#define WHO_AM_I            0x75

#define MPU6050_ADDR        0x68


#define DEVICE_RESET_NOSLEEP_NOCYCLE 0x80
#define INIT_GYRO_SELF_TEST      0x1F

enum returntype
{
    SUCCESS,
    FAILURE
};

class mpu6050_base{
    public:
        // initialise function - start I2C comm, confirm WHO_AM_I address value
        mpu6050_base();

        // public function to self test Gyro
        returntype selftest_gyro();

        // public function to read WHO_AM_I register to verify I2C communication
        returntype verify_slave_communication();

        // functions to configure interrupt behaviour -> INT_PIN_CFG
        //      function to enable FSYNC pin to send interrupt to host
        //      various macros defining INT_LEVEL, INT_OPEN, LATCH_INT_EN, INT_RD_CLEAR, FSYNC_INT_LEVEL values
    
        // public functions to read accelerometer readings from ACCEL_XOUT_H, etc registers

        // public functions to read temperature sensors readings from TEMP_OUT_H and TEMP_OUT_L registers

        // public functios to reading gyro readings from GYRO_XOUT_H, etc registers
        void read_gyro(uint16_t *x_val, uint16_t *y_val, uint16_t *z_val);

        // public (tentative) function to read external sensor data

        // public function to enable and disable FIFO buffer -> USER_CTRL

        // public function to enable and disable primary I2C bus -> USER_CTRL

        // public function to reset FIFO, I2C master and SIG_COND_RESET -> USER_CTRL

        // public function to disable temp sensor -> PWR_MGMT_1

        // public function to configure sleep mode -> PWR_MGMT_1 and LP_WAKE_CTRL(PWR_MGMT_2)

        // public function to reset the entire MPU -> PWR_MGMT_1

        // public function to specify clock source -> PWR_MGMT_1

        // public function to put individual axes of gyro and accelerometer on stand by

        // public function to read FIFO count

        // public function to read and write FIFO buffer

    private:
        bool initialized;

        // private fubnction to read from registers
        int read_register(uint8_t register_addr);

        // private function to write into a register
        void write_register(uint8_t register_addr, uint8_t data);

        // [tentative] function to change  SMPLRT_DIV value

        // function to change CONFIG value based on individual situations (maybe multiple functions) (private functions for sure)

        // private function to enable FIFO

        // private function to configure auxiliary I2C bus -> I2C_MST_CTRL

        // private functions to configure individual slave i2c nodes if aux i2c bus is used -> I2C_SLV0_ADDR, I2C_SLV0_REG, and I2C_SLV0_CTRL

        // function to configure interrupt enables in case of fifo overflow, I2C_MST_INT_EN and data ready state

        // private funciton to read WHO_AM_I register to be used during configuration

};