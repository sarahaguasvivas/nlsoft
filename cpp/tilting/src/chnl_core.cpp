#include "SRTxMCU.h"

CoreChannel::CoreChannel(int addr) : MCUChannel()
{
    core.attach(this, addr);
}

bool CoreChannel::run_cmd(uint8_t cmd)
{
    switch (cmd)
    {
    case CMD_CONNECT:
        core.disable_cb(core.CB_STDBY);
        core.enable_cb(core.CB_SYNC);
        sys_led.enable();
        for (int ix = 0; ix < ADDR_MAX; ix++)
        {
            if (core.is_defined(ix)) { core.channels[ix]->at_connect(); }
        }
        return true;

    case CMD_DISCONNECT:
        core.enable_cb(core.CB_STDBY);
        core.disable_cb(core.CB_SYNC);
        sys_led.enable();
        for (int ix = 0; ix < ADDR_MAX; ix++)
        {
            if (core.is_defined(ix)) { core.channels[ix]->at_disconnect(); }
        }
        return true;

    case CMD_ASK_CONFIG:
        for (int ix = 0; ix < ADDR_MAX; ix++)
        {
            if (core.is_defined(ix)) { core.channels[ix]->send_config(); }
        }
        return true;

    case CMD_PING:
        send_status(STS_OK);
        return true;

    case CMD_RESET_CLOCK:
        sys_clock.reset();
        return true;

    case CMD_BLINK:
        if (core.cb_enabled(core.CB_BLINK)) { core.disable_cb(core.CB_BLINK); }
        else { core.enable_cb(core.CB_BLINK); }
        return true;

    default:
        return false;
    }
}


bool CoreChannel::set_data(uint8_t trgt, uint8_t *data)
{
    switch (trgt)
    {
    case TGT_LED:
        if (decode_bool(data)) {sys_led.enable();}
        else {sys_led.disable();}
        return true;

    case TGT_BLINK_USEC:
        core.update_cb(core.CB_BLINK, decode_uint32(data));
        return true;

    case TGT_SPIN_USEC:
        core.update_cb(core.CB_SPIN, decode_uint32(data));
        return true;

    case TGT_STDBY_USEC:
        core.update_cb(core.CB_STDBY, decode_uint32(data));
        return true;

    case TGT_SYNC_USEC:
        core.update_cb(core.CB_SYNC, decode_uint32(data));
        return true;
    
    default:
        return false;
    }
}


bool CoreChannel::get_data(uint8_t trgt, uint8_t *data)
{
    switch (trgt)
    {
    case TGT_LED:
        encode_bool(data, sys_led.get());
        return true;

    case TGT_N_CHNL:
        encode_uint8(data, core.count_channels());
        return true;

    case TGT_CLOCK:
        encode_uint32(data, sys_clock.get());
        return true;

    case TGT_BLINK_USEC:
        encode_uint32(data, core.cb_delay(core.CB_BLINK));
        return true;

    case TGT_SPIN_USEC:
        encode_uint32(data, core.cb_delay(core.CB_SPIN));
        return true;

    case TGT_STDBY_USEC:
        encode_uint32(data, core.cb_delay(core.CB_STDBY));
        return true;

    case TGT_SYNC_USEC:
        encode_uint32(data, core.cb_delay(core.CB_SYNC));
        return true;
    
    default:
        return false;
    }
}


int CoreChannel::data_size(uint8_t trgt)
{
    switch (trgt)
    {
    case TGT_LED:
        return SIZE_BOOL;

    case TGT_N_CHNL:
        return SIZE_UINT8;
    
    case TGT_CLOCK:
    case TGT_BLINK_USEC:
    case TGT_SPIN_USEC:
    case TGT_STDBY_USEC:
    case TGT_SYNC_USEC:
        return SIZE_UINT32;
    
    default:
        return 0;
    }
}
