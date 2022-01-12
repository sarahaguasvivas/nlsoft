#include "SRTxMCU.h"


UltravoltChannel::UltravoltChannel(int addr, int pin_hv_enable) : MCUChannel(), PIN_HV_ENABLE(pin_hv_enable)
{
    hv_active = false;
    core.attach(this, addr);
}


void UltravoltChannel::disable_hv()
{
    digitalWrite(PIN_HV_ENABLE, LOW);
    hv_active = false;
    hv_locked = true;
}


void UltravoltChannel::begin()
{
    pinMode(PIN_HV_ENABLE, OUTPUT);
    disable_hv();
}


void UltravoltChannel::at_disconnect()
{
    disable_hv();
}

void UltravoltChannel::at_connect()
{
   disable_hv();
}


void UltravoltChannel::spin()
{
    if (!hv_active && !hv_locked)
    {
        if (ticks_remaining > 0) { ticks_remaining--; }
        else { hv_locked = true; }
    }
}

bool UltravoltChannel::run_cmd(uint8_t cmd)
{
    switch (cmd)
    {
    case CMD_UNLOCK_HV:
        hv_locked = false;
        ticks_remaining = (int)((3*USEC_1HZ)/core.cb_delay(core.CB_SPIN)); //ensure a 3sec delay regardless of spin freq
        return true;

    case CMD_ENABLE_HV:
        if (hv_locked) { return false; }
        else
        {
            digitalWrite(PIN_HV_ENABLE, HIGH);
            hv_active = true;
            return true;
        }
        
    case CMD_DISABLE_HV:
        disable_hv();
        return true;

    default:
        return false;
    }
}




bool UltravoltChannel::get_data(uint8_t trgt, uint8_t *data)
{
    switch (trgt)
    {
    case TGT_HV_LOCKED:
        encode_bool(data, hv_locked);
        return true;
    
    case TGT_HV_ACTIVE:
        encode_bool(data, hv_active);
        return true;

    default:
        return false;
    }
}


int UltravoltChannel::data_size(uint8_t trgt)
{
    
    switch (trgt)
    {
    case TGT_HV_LOCKED:
    case TGT_HV_ACTIVE:
        return SIZE_BOOL;

    default:
        return 0;
    }
}






