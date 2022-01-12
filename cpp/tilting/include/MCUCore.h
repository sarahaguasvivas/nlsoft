#ifndef MCUCORE_H
#define MCUCORE_H

#include "SRTxMCU.h"


class MCUCore {
public:
    
//------------------------- general -------------------------//
    
    //constructor
    MCUCore();
    
    //call at the end of setup
    static void begin();

    //call at the start of loop
    static void spin();


//------------------------- channels -------------------------//

    //connects a channel to be managed by the core
    static void attach(MCUChannel *chptr, int addr);

//------------------------- callbacks -------------------------//

    static const int CB_SPIN = 0; //MCUCore.spin_channels()
    static const int CB_BLINK = 1; //MCUCore.toggle_led()
    static const int CB_STDBY = 2; //special blink
    static const int CB_SYNC = 3; //MCUCore.stream_channels()

    // returns true if the specified callback is enabled
    static bool cb_enabled(int id);

    // enables the specified callback
    static void enable_cb(int id);

    // disables the specified callback
    static void disable_cb(int id);

    // changes the frequency at which the callback runs
    static void update_cb(int id, uint32_t d);

    // get the delay interval of the current callback (usec)
    static uint32_t cb_delay(int id);





//------------------------- channels -------------------------//

    //array of pointers to all attatched channels
    static MCUChannel *channels[ADDR_MAX];

    //check whether a channel is defined at addr
    static bool is_defined(int addr);

    static uint8_t count_channels();
   

private:

    //holds whether or not the channel is defined
    static bool channel_defined[ADDR_MAX];


    //run the spin() method of each channel
    static void spin_channels();

    //cycle thru all defined channels, send targets if they're streaming
    static void sync_channels();

    //send a timestamp to julia
    static bool send_time();



//------------------------- callbacks -------------------------//
    //container to hold parameters/data for callback functions
    struct cb
    {
        uint32_t prev_micros; //timestamp of the previous run
        uint32_t delay_micros; //delay interval
        bool enabled;
        void (* fxn) (); //pointer to the callback function
    };

    static cb callbacks[N_CALLBACKS];
    
    // runs scheduled callback functions if enough time has passed
    static void spin_callbacks();

    // schedule a call to function f every d microseconds
    static void schedule_cb(int id, uint32_t d, void (* fxn) ());


//------------------------- incoming -------------------------//

    static PacketBuffer buffer;

    //fill buffer from serial, handle any available packets
    static void spin_buffer();

    // handle an incoming packet
    static void route_packet(SRTxPacket pkt);

};






#endif
