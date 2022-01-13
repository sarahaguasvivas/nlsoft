#ifndef CHNLS_H
#define CHNLS_H

#include "SRTxMCU.h"

class CoreChannel : public MCUChannel {
public:
//------------------------- main fxns -------------------------//

    //constructor initializes initial values, pins
    CoreChannel(int addr);

    // return type of channel configuration
    uint8_t get_config() { return CFG_CORE; }

    // the "main" loop of the channel, essentially
    void spin() {};

    // runs a command (CTX_RUN_CMD)
    bool run_cmd(uint8_t cmd);

    // set value of a target from decoded packet bytes
    bool set_data(uint8_t trgt, uint8_t *data);

    // get value of a target, encode into packet as bytes
    bool get_data(uint8_t trgt, uint8_t *data);

    // get size of the target's encoded data
    int data_size(uint8_t trgt);

};


extern CoreChannel core_channel; //defined here because it's special


class TestingChannel : public MCUChannel {
public:

    //constructor initializes initial values, pins
    TestingChannel(int addr, int pin_sck, int pin_sda);

    // return type of channel configuration
    uint8_t get_config() { return CFG_TESTING; }

    // the "main" loop of the channel, essentially
    void spin();

    //initialize pins
    void begin();
    
    //runs whenever a host connects
    void at_connect();

    //runs whenever a host disconnects
    void at_disconnect();

    // runs a command (CTX_RUN_CMD)
    bool run_cmd(uint8_t cmd);

    // set value of a target from decoded packet bytes
    bool set_data(uint8_t trgt, uint8_t *data);

    // get value of a target, encode into packet as bytes
    bool get_data(uint8_t trgt, uint8_t *data);

    // get size of the target's encoded data
    int data_size(uint8_t trgt);



private:
    const int PIN_SCK;
    const int PIN_SDA;

    Adafruit_SH110X display = Adafruit_SH110X(64, 128, &Wire);
    // U8X8_SH1106_128X64_NONAME_HW_I2C display;
    
    bool test_bool;
    
    uint8_t test_uint8;
    uint16_t test_uint16;
    uint32_t test_uint32;

    int8_t test_int8;
    int16_t test_int16;
    int32_t test_int32;

    bool need_update;
    void update_display();
};












class DigitalInChannel : public MCUChannel {
public:

    //constructor initializes initial values, pins
    DigitalInChannel(int addr, int pin_d_in);

    // return type of channel configuration
    uint8_t get_config() { return CFG_DIGITAL_IN; }

    //initialize pins
    void begin();

    // the "main" loop of the channel, essentially
    void spin();

    // get value of a target, encode into packet as bytes
    bool get_data(uint8_t trgt, uint8_t *data);

    // get size of the target's encoded data
    int data_size(uint8_t trgt);

private:
    const int PIN_D_IN;

    bool pin_state;

};









class DigitalOutChannel : public MCUChannel {
public:

    //constructor initializes initial values, pins
    DigitalOutChannel(int addr, int pin_d_out);

    // return type of channel configuration
    uint8_t get_config() { return CFG_DIGITAL_OUT; }

    //initialize pins
    void begin();

    // the "main" loop of the channel, essentially
    void spin();

    // set value of a target from decoded packet bytes
    bool set_data(uint8_t trgt, uint8_t *data);

    // get value of a target, encode into packet as bytes
    bool get_data(uint8_t trgt, uint8_t *data);

    // get size of the target's encoded data
    int data_size(uint8_t trgt);

private:
    const int PIN_D_OUT;

    bool pin_state;
    bool need_update;
};







class AnalogInChannel : public MCUChannel {
public:

    //constructor initializes initial values, pins
    AnalogInChannel(int addr, int pin_d_in);

    // return type of channel configuration
    uint8_t get_config() { return CFG_ANALOG_IN; }

    //initialize pins
    void begin();

    // the "main" loop of the channel, essentially
    void spin();

    // get value of a target, encode into packet as bytes
    bool get_data(uint8_t trgt, uint8_t *data);

    // get size of the target's encoded data
    int data_size(uint8_t trgt);

private:
    const int PIN_A_IN;

    uint16_t pin_state;

};









class AnalogOutChannel : public MCUChannel {
public:

    //constructor initializes initial values, pins
    AnalogOutChannel(int addr, int pin_d_out);

    // return type of channel configuration
    uint8_t get_config() { return CFG_ANALOG_OUT; }

    //initialize pins
    void begin();

    // the "main" loop of the channel, essentially
    void spin();

    // set value of a target from decoded packet bytes
    bool set_data(uint8_t trgt, uint8_t *data);

    // get value of a target, encode into packet as bytes
    bool get_data(uint8_t trgt, uint8_t *data);

    // get size of the target's encoded data
    int data_size(uint8_t trgt);

private:
    const int PIN_A_OUT;

    uint16_t pin_state;
    bool need_update;

};








class DriverChannel : public MCUChannel {
public:

    //constructor initializes initial values, pins
    DriverChannel(int addr, int pin_chg, int pin_drn, int pin_v_mon);

    // return type of channel configuration
    uint8_t get_config() { return CFG_DRIVER; }

    // the "main" loop of the channel, essentially
    void spin();

    //initialize pins
    void begin();

    //runs whenever a host disconnects
    void at_disconnect();

    // set value of a target from decoded packet bytes
    bool set_data(uint8_t trgt, uint8_t *data);

    // get value of a target, encode into packet as bytes
    bool get_data(uint8_t trgt, uint8_t *data);

    // get size of the target's encoded data
    int data_size(uint8_t trgt);

    uint16_t charge_pwm;
    bool update_pwm; //toggled whenenver a new PWM value is written: prevent constantly writing the same thing


private:
    const int PIN_CHG;
    const int PIN_DRN;
    const int PIN_V_MON;

    uint16_t drain_pwm;
    uint16_t v_ref;
    uint16_t v_mon;


    bool is_ctrl; //true -> run CL voltage controller, false -> write raw PWM values

    void run_ctrl(); //solves for pwm values given vref/vmon and stores them

    // for controller
    int32_t error[3]; //contains error[k-2,k-1,k] for each channel
    int32_t input[3]; //contains input[k-2,k-1,k] for each channel
    // example: input[0-9] is for k-2, input[10-19] is for k-1, input[20-29] is for k, for 10 channels
    // will be provided in dcu_targets.cpp or scu_targets.cpp
   
};










class UltravoltChannel : public MCUChannel {
public:

    //constructor initializes initial values, pins
    UltravoltChannel(int addr, int pin_hv_enable);

    // return type of channel configuration
    uint8_t get_config() { return CFG_ULTRAVOLT; }

    //initialize pins
    void begin();

    // the "main" loop of the channel, essentially
    void spin();

    //runs whenever a host connects
    void at_connect();

    //runs whenever a host disconnects
    void at_disconnect();

    // runs a command (CTX_RUN_CMD)
    bool run_cmd(uint8_t cmd);

    // get value of a target, encode into packet as bytes
    bool get_data(uint8_t trgt, uint8_t *data);

    // get size of the target's encoded data
    int data_size(uint8_t trgt);

private:
    const int PIN_HV_ENABLE;

    bool hv_locked;
    bool hv_active;
    int ticks_remaining; //counts down how long to keep the lock open

    void disable_hv(); //disable HV and set lock
};



// (addr, SPIClass*, pin, pin) 



class SensorChannel : public MCUChannel {
public:

    //constructor initializes initial values, pins
    SensorChannel(int addr, int pin_cs, int pin_rdy);

    // return type of channel configuration
    uint8_t get_config() { return CFG_SENSOR; }

    //initialize pins
    void begin();

    // the "main" loop of the channel, essentially
    void spin();

    //runs whenever a host connects
    void at_connect();

    //runs whenever a host disconnects
    void at_disconnect();

    // runs a command (CTX_RUN_CMD)
    bool run_cmd(uint8_t cmd);

    // set value of a target from decoded packet bytes
    bool set_data(uint8_t trgt, uint8_t *data);

    // get value of a target, encode into packet as bytes
    bool get_data(uint8_t trgt, uint8_t *data);

    // get size of the target's encoded data
    int data_size(uint8_t trgt);

private:
    const int PIN_CS;
    const int PIN_RDY;

    bool chip_ok;

    int16_t x_mG;
    int16_t y_mG;
    int16_t z_mG;
    int16_t x_mG_base;
    int16_t y_mG_base;
    int16_t z_mG_base;
    int32_t z_filt[3];

    // pull the CS pin low to activate I/O to the device
    void enable_cs();

    // pull the CS pin high to disable I/O to the device
    void disable_cs();

    // TODO: document what this does and why it does it
    void config_chip();

    // determine if the chip is connected & responsive, save the result to chip_ok
    void check_chip();

    // read data from the chip over spi, save data to x/y/z_mG
    void read_chip();

    //get the baseline data from all the sensors
    void get_baseline();
};










#endif
