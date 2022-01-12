#include "SRTxMCU.h"

AnalogOutChannel::AnalogOutChannel(int addr, int pin_a_out) : MCUChannel(), PIN_A_OUT(pin_a_out)
{
    pin_state = false; //optional
    need_update = true;
    core.attach(this, addr);
}


void AnalogOutChannel::begin()
{
    pinMode(PIN_A_OUT, OUTPUT);
}

void AnalogOutChannel::spin()
{
    if (need_update)
    {
        analogWrite(PIN_A_OUT, pin_state);
        need_update = false;
    }   
}

bool AnalogOutChannel::set_data(uint8_t trgt, uint8_t *data)
{
    switch (trgt)
    {
    case TGT_AOUT:
        pin_state = decode_uint16(data);
        need_update = true;
        return true;

    default:
        return false;
    }
}

bool AnalogOutChannel::get_data(uint8_t trgt, uint8_t *data)
{
    switch (trgt)
    {
    case TGT_AOUT:
        encode_uint16(data, pin_state);
        return true;

    default:
        return false;
    }
}


int AnalogOutChannel::data_size(uint8_t trgt)
{
    switch (trgt)
    {
    case TGT_AOUT:
        return SIZE_UINT16;
    
    default:
        return 0;
    }
}
