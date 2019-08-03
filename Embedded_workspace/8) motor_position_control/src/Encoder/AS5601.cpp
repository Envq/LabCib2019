#include "AS5601.h"

/**
 *  Constructor of AS5601 class (device address set to ADDRESS)
 *  @param sda  pin name of the I2C data bus
 *  @param scl  pin name of the I2C clock
 */
AS5601::AS5601(PinName sda, PinName scl) : EncoderI2C(ADDRESS, sda, scl)
{
    prev_lsb = 0.0;
    prev_msb = 0.0;
    prev_bits = 0;
    cycle_count = 0;
    past_degrees = 0.0;
}

/**
 *  Constructor of AS5601 class
 *  @param device_address   address of the device
 *  @param sda  pin name of the I2C data bus
 *  @param scl  pin name of the I2C clock
 */
AS5601::AS5601(int device_address, PinName sda, PinName scl) : EncoderI2C(device_address, sda, scl)
{
    prev_lsb = 0.0;
    prev_msb = 0.0;
    prev_bits = 0;
    cycle_count = 0;
    past_degrees = 0.0;
}

#define __sign(x) (std::signbit(x)?-1.0:1.0)
/**
 *  @return the angle
 */
unsigned int AS5601::getAngle()
{
    msb = 0;
    lsb = 0;
    bits = 0;
    msb_success = device.readReg8(ANGLE_REG_MSB, &msb);
    lsb_success = device.readReg8(ANGLE_REG_LSB, &lsb);
    
    if (msb_success!=0 || lsb_success != 0)
        return 10000;

    // create the complete number by shifting 8 bit left the msb part
    bits = (msb << 8 | lsb);

    // Correction in case the Most Significant Byte is not yet updated
    if (lsb - prev_lsb > 127 && msb == prev_msb) 
        // Alert: If (prev==0) and the byte is not updated, (prev_msb-1) creates a spike since all bits flip to 1   
        msb = (prev_msb != 0) ? prev_msb - 1 : prev_msb; 
    else if (lsb - prev_lsb < -127 && msb == prev_msb) 
        msb = prev_msb + 1;
    
    bits = (msb << 8 | lsb);

    prev_lsb = lsb;
    prev_msb = msb;
    prev_bits = bits;
    return bits;
}

// TODO
/**
 *  Set the angle's zero position
 */
void AS5601::resetOffset()
{
    //I2C_bus_write8(indirizzo,indice_azzera,comando_per_azzerare);
    //device.write_reg8(indice
}

/**
 *  @return the number of bits of the angle
 */
int AS5601::getResolutionBits()
{
    return BIT_ANGLE_RESOLUTION;
}

/**
 *  @return the angle in degrees considering the compensation
 */
float AS5601::getAngleDegCompensated()
{
    float angle_bits = getAngle();
    float bits_resolution = pow(2.0, (float)getResolutionBits());
    float actual_degrees = (360.0 / bits_resolution) * angle_bits;

    if (past_degrees > 300 && actual_degrees < 60)
    {
        cycle_count += 1;
    }
    else if (past_degrees < 60 && actual_degrees > 300)
    {
        cycle_count -= 1;
    }

    past_degrees = actual_degrees;

    return 360 * cycle_count + actual_degrees;
}

/**
 *  @return the angle in radiants considering the compensation
 */
float AS5601::getAngleRadCompensated()
{
    unsigned int angle_bits = getAngle();
    float bits_resolution = pow(2.0, BIT_ANGLE_RESOLUTION);
    float actual_radiants = (2.0f * M_PI / bits_resolution) * angle_bits;

    if (past_radiants > 5.8 && actual_radiants < 0.4)
    {
        cycle_count += 1;
    }
    else if (past_radiants < 0.4 && actual_radiants > 5.8)
    {
        cycle_count -= 1;
    }

    past_radiants = actual_radiants;

    return 2.0f * M_PI * cycle_count + actual_radiants;
}

/**
 *  @return the angle in degrees
 */
float AS5601::getAngleDeg()
{
    float angle_bits = getAngle();
    float bits_resolution = pow(2.0, (float)getResolutionBits());
    return (360.0 / bits_resolution) * angle_bits;
}

/**
 *  @return the angle in radiants
 */
float AS5601::getAngleRad()
{
    float angle_bits = getAngle();
    float bits_resolution = pow(2.0, (float)getResolutionBits());
    return (2 * M_PI / bits_resolution) * angle_bits;
}

/* Set the ABN register
 * @param   rate 
 * 0000 : 8 (61 Hz)
 * 0001 : 16 (122 Hz)
 * 0010 : 32 (244 Hz)
 * 0011 : 64 (488 Hz)
 * 0100 : 128 (976 Hz) 
 * 0101 : 256 (1.9 kHz) 
 * 0110 : 512 (3.9 kHz) 
 * 0111 : 1024 (7.8 kHz) 
 * others : 2048 (15.6 kHz)
 */
void AS5601::writeABN(unsigned int rate)
{
    device.writeReg8(ABN_REGISTER, rate);
}

/**
 *  @return the ABN register
 */
int AS5601::readABN()
{
    int data = 0;
    device.readReg8(ABN_REGISTER, &data);
    return data;
}

/**
 *  @return the AGC register
 */
int AS5601::readAGC()
{
    int data = 0;
    device.readReg8(AGC_REGISTER, &data);
    return data;
}

/**
 *  @return the address of the device
 */
int AS5601::getAddress()
{
    return ADDRESS;
}
