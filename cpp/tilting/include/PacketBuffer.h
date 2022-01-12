#ifndef PKTBUF_H
#define PKTBUF_H

#include "SRTxMCU.h"



// buffers incoming bytes and reconstructs packets
class PacketBuffer {
public:

    // constructor
    PacketBuffer() : input_buffer(1024) {};

    // copy up to MAX_S_BYTES from hardware serial buffer -> input buffer, if available
    void spin();

    // iterates thru the input buffer until either the next data is a complete, valid packet, or the buffer is too empty to continue
    bool packet_available();

    // returns the next packet in the buffer
    SRTxPacket fetch_packet();

    // returns (and deletes) the next packet in the buffer
    SRTxPacket read_packet();

private:
    
    // buffers received bytes until they can be processed
    CircularBuffer<uint8_t> input_buffer; // should be much larger than MAX_S_BYTES
    
    // maximum number of bytes to copy from serial on each loop iteration
    const int MAX_S_BYTES = 64;

    // iterates thru bytes in the input_buffer and removes them until either the next one is PKT_START (returns true), or the buffer is too empty (returns false)
    bool find_packet_start();

    // delete bytes from the input buffer
    void delete_packet(uint8_t packet_size);
};


#endif
