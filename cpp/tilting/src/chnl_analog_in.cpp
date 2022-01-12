#include "SRTxMCU.h"

AnalogInChannel::AnalogInChannel(int addr, int pin_a_in) : MCUChannel(), PIN_A_IN(pin_a_in)
{
    pin_state = false; //optional
    core.attach(this, addr);
}


void AnalogInChannel::begin()
{
    // pinMode(PIN_A_IN, INPUT_PULLUP); #FUTURE: set analog read resolution?
}


void AnalogInChannel::spin()
{
    pin_state = analogRead(PIN_A_IN);
}


bool AnalogInChannel::get_data(uint8_t trgt, uint8_t *data)
{
    switch (trgt)
    {
    case TGT_AIN:
        encode_uint16(data, pin_state);
        return true;

    default:
        return false;
    }
}


int AnalogInChannel::data_size(uint8_t trgt)
{
    switch (trgt)
    {
    case TGT_AIN:
        return SIZE_UINT16;
    
    default:
        return 0;
    }
}
