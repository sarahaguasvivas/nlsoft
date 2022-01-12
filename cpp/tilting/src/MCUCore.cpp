#include "SRTxMCU.h"

// initialize static members -> values will be set later
MCUCore::cb MCUCore::callbacks[N_CALLBACKS];
PacketBuffer MCUCore::buffer;
MCUChannel * MCUCore::channels[ADDR_MAX];
bool MCUCore::channel_defined[ADDR_MAX];


uint32_t default_blink_usec = USEC_10HZ; // toggle LED every 0.1 sec (5 Hz blink)
uint32_t default_spin_usec = 1000; // 1000 Hz
uint32_t default_stdby_usec = 300000; // step LED thru custom pattern every 0.5 sec
uint32_t default_sync_usec = 4000;//3333 for 300 Hz //4000 for 250 Hz - the sample rate (with respect to sampling MCU -> julia)

//------------------------- constructors & setup -------------------------//

MCUCore::MCUCore()
{
    sys_led.disable();
    schedule_cb(CB_BLINK, USEC_10HZ, &sys_led.toggle); // a 10Hz blink - primarily useful for testing
    schedule_cb(CB_SPIN, uint32_t(1000), &MCUCore::spin_channels); // 1 kHz - the control loop frequency (and the hardware->MCU sample rate)
    schedule_cb(CB_STDBY, 3*USEC_10HZ, &sys_led.standby); // step thru custom standby blink at 3.3Hz
    schedule_cb(CB_SYNC, uint32_t(4000), &MCUCore::sync_channels); // 300Hz or 250 Hz - sample rate (MCU->julia)
    //NOTE: setting this below 1000 (ie. faster than 1kHz) should work as of v0.7.0
    
    for (int addr = 0; addr < ADDR_MAX; addr++)
    {
        channel_defined[addr] = false;
    }
}


void MCUCore::begin()
{
    analogWriteResolution(16);
    analogReadResolution(12); //currently
    // analogReadResolution(12);//max val: 4095 but ~1.5x slower
    enable_cb(CB_SPIN); //'technically' unecessary, cb enabled by default
    enable_cb(CB_STDBY); //'technically' unecessary, cb enabled by default
    disable_cb(CB_SYNC);
    disable_cb(CB_BLINK);
    sys_clock.reset();
    spi_begin(); //will only begin the spi channel defined, or none

    for (int addr = 0; addr < ADDR_MAX; addr++)
    {
        if (is_defined(addr)) {channels[addr]->begin();} 
    }
    Serial.begin(115200); //teensy ignores the baud rate, and uses maximum USB 12MBit/s
}


void MCUCore::spin()
{
    spin_callbacks();
    spin_buffer();
    Serial.send_now();
}


//------------------------- channels -------------------------//

void MCUCore::attach(MCUChannel *chptr, int addr)
{
    chptr->set_addr(addr);
    channels[addr] = chptr;
    channel_defined[addr] = true;
}


uint8_t MCUCore::count_channels()
{
    uint8_t n_channels = 0;

    for (int addr = 0; addr < ADDR_MAX; addr++)
    {
        if (is_defined(addr)) {n_channels++;}
    }
    return n_channels;
}



bool MCUCore::is_defined(int addr)
{
    if (addr < ADDR_MAX)
    {
        return channel_defined[addr];
    }
    return false;
}

void MCUCore::spin_buffer()
{
    while (true == buffer.packet_available())
    {
        SRTxPacket pkt = buffer.read_packet();
        
        //packets are handled by their destination channel
        if (is_defined(pkt.addr)) { route_packet(pkt); }
        //TODO: //FUTURE: send addr not defined error
    }
    //at this point, the input buffer does not contain any more packets, and we should refill it
    buffer.spin();
}


void MCUCore::route_packet(SRTxPacket pkt)
{
    switch (pkt.ctxt)
    {
        case CTX_RUN:
            //run the designated command, send error if cmd not defined
            if(!channels[pkt.addr]->run_cmd(pkt.trgt)) {channels[pkt.addr]->send_status(STS_CMD_ERR);}
            break;

        case CTX_SET:
            // decode and store the new data, send error if decoding fxn not defined
            if(!channels[pkt.addr]->set_data(pkt.trgt, pkt.data)) {channels[pkt.addr]->send_status(STS_DEC_ERR);}
            break;

        case CTX_ASK:
            // encode and send the requested data, send error if encoding fxn not defined
            if(!channels[pkt.addr]->send_data(pkt.trgt)) {channels[pkt.addr]->send_status(STS_ENC_ERR);}
            break;

        case CTX_SYNC:
            channels[pkt.addr]->toggle_sync(pkt.trgt, true);
            break;

        case CTX_UNSYNC:
            channels[pkt.addr]->toggle_sync(pkt.trgt, false);
            break;
        
        default:
            channels[pkt.addr]->send_status(STS_CTX_ERR); //context not recognized
            break;
    }
}

void MCUCore::spin_channels()
{
    for (int addr = 0; addr < ADDR_MAX; addr++)
    {
        if (is_defined(addr)) { channels[addr]->spin(); }
    }
}

void MCUCore::sync_channels()
{
    //always send timestamp & send it first
    send_time();

    int n_streams = 0; //counter to determine if we're streaming anything
    for (int addr = 0; addr < ADDR_MAX; addr++){
        if (is_defined(addr)){
            for (int trgt = 0; trgt < TRGT_MAX; trgt++){
                if (true == channels[addr]->is_sync(trgt)){
                    // attempt to encode and send the requested data
                    if(channels[addr]->send_data(trgt)){ n_streams++; }
                    else
                    {    
                        //send error once, and stop streaming
                        channels[addr]->send_status(STS_SYNC_ERR);
                        channels[addr]->toggle_sync(trgt, false);
                    }
                }
            }
        }
    }
}




//------------------------- callbacks -------------------------//



void MCUCore::spin_callbacks()
{
    // get current time & check if enough has passed to run each callback
    uint32_t current_micros = micros();

    for (int ix = 0; ix < N_CALLBACKS; ix++)
    {
        // see http://arduino.cc/forum/index.php/topic,124048.msg932592.html#msg932592

        if (current_micros - callbacks[ix].prev_micros >= callbacks[ix].delay_micros)
        {
            // update time
            callbacks[ix].prev_micros = current_micros;
            // callbacks[ix].prev_micros += callbacks[ix].delay_micros;

            // call the pointer to the callback fxn if it's enabled
            if (callbacks[ix].enabled) {(*(callbacks[ix].fxn))();}
        }
    }
}

void MCUCore::schedule_cb(int id, uint32_t d, void (*fxn)())
{
    callbacks[id].prev_micros = micros();
    callbacks[id].delay_micros = d;
    callbacks[id].enabled = true;
    callbacks[id].fxn = fxn;
}

bool MCUCore::cb_enabled(int id)
{
    return callbacks[id].enabled;
}

void MCUCore::enable_cb(int id)
{
    callbacks[id].enabled = true;
}

void MCUCore::disable_cb(int id)
{
    callbacks[id].enabled = false;
}

void MCUCore::update_cb(int id, uint32_t d)
{
    callbacks[id].delay_micros = d;
}

uint32_t MCUCore::cb_delay(int id)
{
    return callbacks[id].delay_micros;
}





bool MCUCore::send_time()
{
    uint8_t pkt_size = 8;
        
    uint8_t pkt[pkt_size] = 
    {
        PKT_START,
        pkt_size,
        CTX_TIME,
    };
    pkt[pkt_size - 1] = PKT_END;

    //encode & send
    uint8_t *dataptr = pkt + 3;
    encode_uint32(dataptr, sys_clock.get());
    Serial.write(pkt, pkt_size);
    return true;
}

