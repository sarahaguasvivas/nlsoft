#include "SRTxMCU.h"

DigitalOutChannel::DigitalOutChannel(int addr, int pin_d_out) : MCUChannel(), PIN_D_OUT(pin_d_out)
{
    pin_state = false; //optional
    need_update = true;
    core.attach(this, addr);
}


void DigitalOutChannel::begin()
{
    pinMode(PIN_D_OUT, OUTPUT);
}

void DigitalOutChannel::spin()
{
    if (need_update)
    {
        digitalWrite(PIN_D_OUT, pin_state);
        need_update = false;
    }   
}

bool DigitalOutChannel::set_data(uint8_t trgt, uint8_t *data)
{
    switch (trgt)
    {
    case TGT_DOUT:
        pin_state = decode_bool(data);
        need_update = true;
        return true;

    default:
        return false;
    }
}

bool DigitalOutChannel::get_data(uint8_t trgt, uint8_t *data)
{
    switch (trgt)
    {
    case TGT_DOUT:
        encode_bool(data, pin_state);
        return true;

    default:
        return false;
    }
}


int DigitalOutChannel::data_size(uint8_t trgt)
{
    switch (trgt)
    {
    case TGT_DOUT:
        return SIZE_BOOL;
    
    default:
        return 0;
    }
}
