//FXA_Gyro_Testbench.ino
#include "Arduino.h"
// #include "icarus_pinout.h"
#include <i2c_t3.h>
#include <math.h>



/* =====================================================================================
                    GYROSCOPE SENSOR (FXAS21002C)
====================================================================================== */

// This Functions lets you interface with the gyroscope to extract angular rotation information.
// Only the Z axis is implemented

/*=========================================================================
    I2C ADDRESS
    -----------------------------------------------------------------------*/
    #define GYRO_ADDR  0x20
/*=========================================================================*/



/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define GYRO_REGISTER_CTRL_REG0     0x0D
    #define GYRO_REGISTER_CTRL_REG1     0x13
    #define GYRO_REGISTER_CTRL_REG2     0x14
    #define GYRO_REGISTER_CTRL_REG3     0x15
    #define GYRO_REGISTER_OUT_Z_MSB     0x05
    #define GYRO_REGISTER_STATUS        0x00
/*=========================================================================*/



/*=========================================================================
    Variables
    -----------------------------------------------------------------------*/
    #define LSBtoDEG     0.125  //15.625 (LSB per mili degree) * 0.008 (conversion to degrees and compensation for the 3LSB we erased)

    int16_t gyro_bias_z = 0;    // This is the current bias of the gyroscope
    float gyro_angle = 0;            // Current integrated angle.

    elapsedMicros gyro_timer;
/*=========================================================================*/



/*===================================================================
                Public available functions
====================================================================*/


// This function configures and starts the Gyroscope
void gyroSetup(void){

    int32_t bias_accumulator = 0;

    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_600);
    i2c_write_byte(GYRO_ADDR, GYRO_REGISTER_CTRL_REG0, 0x02);         // LPF = 256Hz(7,6), HPF deactivated(2), Scale +-500deg/s(1,0)
    i2c_write_byte(GYRO_ADDR, GYRO_REGISTER_CTRL_REG1, 0x03);         // No Reset(6), No SelfTest(5), DataRate = 800Hz(4,3,2), Active and Ready(1,0)

    //Perform inital calibration
    gyroCalibration();

    // Reset timer
    gyro_timer = 0;
}



// This function reads the angular velocity in the Z axis and returns it as a int16_t
int16_t gyroReadCorrected(void){

    int16_t Z_AngSpeed;

    // Read the data from gyroscope
    Z_AngSpeed = i2c_read_word(GYRO_ADDR, GYRO_REGISTER_OUT_Z_MSB);
    //Remove the 3 LSB because they are mostly noise
    Z_AngSpeed = Z_AngSpeed>>3;
    // Substract the current error from the measurement
    Z_AngSpeed -= gyro_bias_z;


    // Integrate the angle over the last period of time (escalated to seconds)
    gyro_angle += Z_AngSpeed*LSBtoDEG*(gyro_timer/1000000.0);
    //Reset timer;
    gyro_timer = 0;

    return Z_AngSpeed;
}

// This function recalculates the bias for the Gyroscope
void gyroCalibration(void){

    int32_t bias_accumulator = 0;

    for (size_t i = 0; i < 16; i++) {
        bias_accumulator += gyroRead(0);
    }
    gyro_bias_z = (int16_t)(bias_accumulator >> 4);
}

// This function is here to help reset the angle if it strays to far.
void setAngle(float z_angle){

    gyro_angle = z_angle;
}

// This function returns the current integrated angle.
float getAngle(void){

    return gyro_angle;
}

// This function returns the current angular velocity in degrees per seconds.
float getAngularVelocity(void){

    int16_t angvel;

    angvel =  gyroReadCorrected();
    return angvel*LSBtoDEG;
}

/*===================================================================
                Private functions
====================================================================*/

// This function reads the angular velocity in the Z axis and returns it as a int16_t
int16_t gyroRead(int16_t gyro_bias_error){

    int16_t Z_AngSpeed;

    // Read the data from gyroscope
    Z_AngSpeed = i2c_read_word(GYRO_ADDR, GYRO_REGISTER_OUT_Z_MSB);
    // Substract the current error from the measurement
    Z_AngSpeed = Z_AngSpeed>>3;
    Z_AngSpeed -= gyro_bias_error;

    return Z_AngSpeed;
}

/*===================================================================
    Down from here are the I2C support functions
====================================================================*/

//Reads a byte from the I2C bus
uint8_t i2c_read_byte(uint8_t address, uint8_t reg){

    //Receives de 'reg' register of the device 'address' through the I2C bus
    uint8_t value;

    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission(I2C_NOSTOP);
    Wire.requestFrom(address, 1, I2C_STOP);
    value = Wire.readByte();
    Wire.endTransmission();

    return value;
}

//Writes a byte from the I2C bus
void i2c_write_byte(uint8_t address, uint8_t reg, uint8_t value){

    //Transmit one byte (value) through the I2C bus to the 'reg' register in the 'address' device
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

//Reads 2 bytes from the I2C bus
int16_t i2c_read_word(uint8_t address, uint8_t reg){

    //Receives de 'reg' register of the device 'address' through the I2C bus
    uint8_t value1, value2;
    int16_t value;

    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission(I2C_NOSTOP);
    Wire.requestFrom(address, 2, I2C_STOP);
    while (Wire.available() < 2);
        value1 = Wire.readByte();
        value2 = Wire.readByte();
    Wire.endTransmission();

    // Joining the data of the word
    value = ((value1 << 8) & 0xFF00) | (value2 & 0x00FF);
    return value;
}
