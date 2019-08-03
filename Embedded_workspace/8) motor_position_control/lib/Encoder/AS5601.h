#ifndef AS5601_H
#define AS5601_H

#include "mbed.h"
#include "math.h"
#include "EncoderI2C.h"


class AS5601 : public EncoderI2C {
    public:
        AS5601(PinName sda, PinName scl);
        AS5601(int device_address, PinName sda, PinName scl);
        
        void update();

        void writeABN(unsigned int rate);
        int readABN();
        int readAGC();
        virtual void resetOffset();
        virtual float getAngleDeg();
        virtual float getAngleRad();
        virtual int getAddress();
        float getAngleDegCompensated();
        float getAngleRadCompensated();
        
        virtual unsigned int getAngle();

    private: 
        virtual int getResolutionBits();
        
        // Variables used for angle compensation
        double past_degrees, past_radiants;
        int cycle_count;

        // Variable for test the coherence of the I2C register read
        unsigned int bits, prev_bits;
        int msb, lsb;
        int prev_msb, prev_lsb;
        int msb_success, lsb_success;
        
        // Address of the device
        static const int ADDRESS = 0x36;
        
        // Number of bit read for the angle
        static const int BIT_ANGLE_RESOLUTION = 12;
        
        // Address of ABN register
        static const int ABN_REGISTER = 0x09;
        
        // Address of raw angle register (MSB)
        static const int RAW_ANGLE_REG_MSB = 0x0C;
        
        // Address of raw angle register (LSB)
        static const int RAW_ANGLE_REG_LSB = 0x0D;
        
        // Address of angle register (MSB)
        static const int ANGLE_REG_MSB = 0x0E;
        
        // Address of angle register (LSB)
        static const int ANGLE_REG_LSB = 0x0F;
        
        // Address of AGC register
        static const int AGC_REGISTER = 0x1A;
};

#endif  // AS5601_H