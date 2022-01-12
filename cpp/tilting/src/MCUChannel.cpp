#include "SRTxMCU.h"

//------------------------- constructors & setup -------------------------//

MCUChannel::MCUChannel()
{
    for (int trgt = 0; trgt < TRGT_MAX; trgt++)
    {
        sync_active[trgt] = false;
    }
}


//------------------------- under the hood -------------------------//


void MCUChannel::send_config()
{
    uint8_t pkt[PKT_MINSIZE] = 
    {
        PKT_START,
        PKT_MINSIZE,
        CTX_CONFIG,
        this->addr,
        get_config(),
        PKT_END
    };

    Serial.write(pkt, PKT_MINSIZE);
}


void MCUChannel::send_status(uint8_t sts)
{
    uint8_t pkt[PKT_MINSIZE] = 
    {
        PKT_START,
        PKT_MINSIZE,
        CTX_STATUS,
        this->addr,
        sts,
        PKT_END
    };

    Serial.write(pkt, PKT_MINSIZE);
}





bool MCUChannel::send_data(uint8_t trgt)
{
    uint8_t pkt_size = PKT_MINSIZE + data_size(trgt);
        
    uint8_t pkt[pkt_size] = 
    {
        PKT_START,
        pkt_size,
        CTX_DATA,
        this->addr,
        trgt
    };
    pkt[pkt_size - 1] = PKT_END;

    //attempt data encoding, send packet if successful
    uint8_t *dataptr = pkt + PKT_MINSIZE - 1;
    if (get_data(trgt, dataptr))
    {
        Serial.write(pkt, pkt_size);
        return true;
    } else { return false; }
}





