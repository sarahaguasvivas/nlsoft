#ifndef MCUCHNL_H
#define MCUCHNL_H

#include "SRTxMCU.h"



class MCUChannel {
public:

//------------------------- main fxns -------------------------//
//NOTE: override these for proper functionality

    //constructor takes pin values, and attaches channel to core
    MCUChannel();

    // return type of channel configuration
    virtual uint8_t get_config() {return CFG_NULL;}
    
    // initialize pins, etc. called by core.begin()
    virtual void begin() {};

    // the "main" loop of the channel, essentially
    virtual void spin() {};

    // reset the channel to a default state
    virtual void reset() {};

    // do this if something bad happens
    virtual void panic() {};

//------------------------- targets -------------------------//
// these return true if sucessful, false if not -> for error handling
//NOTE: override these for proper functionality

    // runs a command (CTX_RUN)
    virtual bool run_cmd(uint8_t cmd){return false;}

    // set value of a target from decoded packet bytes
    virtual bool set_data(uint8_t trgt, uint8_t *data) {return false;}

    // get value of a target, encode into packet as bytes
    virtual bool get_data(uint8_t trgt, uint8_t *data) {return false;}

    // get size of the target's encoded data
    virtual int data_size(uint8_t trgt) {return 0;}



//------------------------- extras -------------------------//
//NOTE: override these for extra functionality

    //runs whenever a host connects
    virtual void at_connect() {};

    //runs whenever a host disconnects
    virtual void at_disconnect() {};



//------------------------- inherited methods -------------------------//

    // accessor function for the address
    uint8_t get_addr() { return addr; }
    void set_addr(uint8_t new_addr) { addr = new_addr; }
   
    // accessor functions for streaming status
    bool is_sync(uint8_t trgt) { return sync_active[trgt]; }
    void toggle_sync(uint8_t trgt, bool state) { sync_active[trgt] = state; }


    // send channel configuration info to the PC
    void send_config();

    // send a status packet to the PC
    void send_status(uint8_t sts);

    // send a data packet to the PC, return false if encoding failed
    bool send_data(uint8_t trgt);



//------------------------- inherited fields -------------------------//
protected:

    //the address of the channel
    uint8_t addr;

    //indexed by target
    bool sync_active[TRGT_MAX];

};





#endif
