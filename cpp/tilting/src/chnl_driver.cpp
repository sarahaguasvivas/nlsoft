#include "SRTxMCU.h"

DriverChannel::DriverChannel(int addr, int pin_chg, int pin_drn, int pin_v_mon) : MCUChannel(), 
                        PIN_CHG(pin_chg), PIN_DRN(pin_drn), PIN_V_MON(pin_v_mon)
{
    is_ctrl = false; //start with raw pwm mode
    update_pwm = true;
    charge_pwm = 0;
    drain_pwm = 9000;
    core.attach(this, addr);
}


void DriverChannel::begin()
{
    pinMode(PIN_CHG, OUTPUT);
    pinMode(PIN_DRN, OUTPUT);
    pinMode(PIN_V_MON, INPUT);
}


void DriverChannel::at_disconnect()
{
    is_ctrl = false;
    update_pwm = true;
    charge_pwm = 0;
    drain_pwm = 9000;
}


void DriverChannel::spin()
{
    v_mon = analogRead(PIN_V_MON);

    if (is_ctrl) { run_ctrl(); }

    if (update_pwm) 
    {
        analogWrite(PIN_CHG, charge_pwm);
        analogWrite(PIN_DRN, 9000); //drain_pwm);
        update_pwm = false;
    }
}



void DriverChannel::run_ctrl()
{
    // NOTE(sarahaguasvivas): Check this code for reference

    if (v_ref > 900){ v_ref = 900; } // limit v_ref to 9kV

    //Note: both error[] and input[] can be positive or negative (not UInt)
    error[0] = error[1];
    error[1] = error[2]; //time update
    error[2] = int32_t(v_ref - v_mon); // read error
    //TODO: This assumes perfect v_mon (no conversion from v_mon to volts)

    //// CONTROLLER
    input[0] = input[1]; //time update (ch_idx is k-2, ch_idx+10 is k-1, ch_idx+20 is k)
    input[1] = input[2]; //time update
    input[2] = input[1] + 200*error[2] -180*error[1]; //control
    ////
    //FUTURE: user-settable controller gains

    // Saturate to prevent windup:
    if (input[2] > 65535){ input[2] = 65535; }
    if (input[2] < -65535){ input[2] = -65535; }

    //Set to charge or drain
    if (input[2] > 0)
    { //CHARGE
        charge_pwm = input[2];
        drain_pwm =  9000; // TODO(sarahaguasvivas): change back to 9000
    }
    else
    { //DRAIN
        drain_pwm = 9000; // TODO(sarahaguasvivas): This was previously 9000
        charge_pwm =  0;
    }

    update_pwm = true; //values will be written later in the spin loop
}

bool DriverChannel::set_data(uint8_t trgt, uint8_t *data)
{
    switch (trgt)
    {
    case TGT_IS_CTRL:
        is_ctrl = decode_bool(data);
        return true;

    case TGT_CHG_PWM: // NOTE(sarahaguasvivas) only target I care about
        charge_pwm = decode_uint16(data);
        is_ctrl = false;
        update_pwm = true;
        return true;

    case TGT_DRN_PWM:
        drain_pwm = decode_uint16(data);
        is_ctrl = false;
        update_pwm = true;
        return true;

    case TGT_V_REF:
        v_ref = decode_uint16(data);
        is_ctrl = true;
        return true;

    case TGT_V_MON:
        return false; // the user can't set this

    default:
        return false;
    }
}






bool DriverChannel::get_data(uint8_t trgt, uint8_t *data)
{
    switch (trgt)
    {
    case TGT_IS_CTRL:
        encode_bool(data, is_ctrl);
        return true;

    case TGT_CHG_PWM:
        encode_uint16(data, charge_pwm);
        return true;

    case TGT_DRN_PWM:
        encode_uint16(data, drain_pwm);
        return true;

    case TGT_V_REF:
        encode_uint16(data, v_ref);
        return true;

    case TGT_V_MON:
        encode_uint16(data, v_mon);
        return true;

    default:
        return false;
    }
}


int DriverChannel::data_size(uint8_t trgt)
{
    
    switch (trgt)
    {
    case TGT_IS_CTRL:
        return SIZE_BOOL;

    case TGT_CHG_PWM:
    case TGT_DRN_PWM:
    case TGT_V_REF:
    case TGT_V_MON:
        return SIZE_UINT16;

    default:
        return 0;
    }
}






