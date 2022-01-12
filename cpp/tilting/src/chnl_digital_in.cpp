#include "SRTxMCU.h"

DigitalInChannel::DigitalInChannel(int addr, int pin_d_in) : MCUChannel(), PIN_D_IN(pin_d_in)
{
    pin_state = false; //optional
    core.attach(this, addr);
}


void DigitalInChannel::begin()
{
    pinMode(PIN_D_IN, INPUT_PULLUP);
}


void DigitalInChannel::spin()
{
    pin_state = digitalRead(PIN_D_IN);
}


bool DigitalInChannel::get_data(uint8_t trgt, uint8_t *data)
{
    switch (trgt)
    {
    case TGT_DIN:
        encode_bool(data, pin_state);
        return true;

    default:
        return false;
    }
}


int DigitalInChannel::data_size(uint8_t trgt)
{
    switch (trgt)
    {
    case TGT_DIN:
        return SIZE_BOOL;
    
    default:
        return 0;
    }
}
