#ifndef I2CRW_H
#define I2CRW_H

#include "mbed.h"

#define I2C_FREQ 1000000

class I2CRW : public I2C {
    private:
        int device_address;
    public:
        I2CRW(int device_address, PinName sda, PinName scl);
        int readReg(int reg_addr, int *reg_data, int cnt);
        int readReg8(int reg_addr, int *reg_data);
        int writeReg8(int reg_addr, int reg_data);
};

#endif  // I2CRW_H