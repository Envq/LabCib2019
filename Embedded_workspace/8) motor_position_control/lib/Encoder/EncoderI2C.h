#ifndef ENCODER_I2C_H
#define ENCODER_I2C_H

#include "mbed.h"
#include "../I2CRW/I2CRW.h"

/**
 *  Abstract class used to implement different types of I2C encoder
 */
class EncoderI2C {
    public:
        EncoderI2C(int device_address, PinName sda, PinName scl): 
            device(device_address,sda,scl) 
        {
            // ntd
            device.frequency(I2C_FREQ);
        }
        virtual void resetOffset() = 0;
        virtual float getAngleDeg() = 0;
        virtual float getAngleRad() = 0;
        virtual int getAddress() = 0;        
    protected:
        I2CRW device;
        virtual unsigned int getAngle() = 0;
        virtual int getResolutionBits() = 0;
};

#endif  // ENCODER_I2C_H