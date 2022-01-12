#include "SRTxMCU.h"


SRTxPacket PacketBuffer::read_packet()
{
    SRTxPacket packet;
    input_buffer.pop(); // delete start byte
    packet.size = input_buffer.pop();
    packet.ctxt = input_buffer.pop();
    packet.addr = input_buffer.pop();
    packet.trgt = input_buffer.pop();

    //copy the data bytes (if any exist)
    if (packet.size > PKT_MINSIZE)
    {
        int data_size = packet.size - PKT_MINSIZE;

        for (int ix = 0; ix < data_size; ix++)
        {
            packet.data[ix] = input_buffer.pop();
        }
    }

    input_buffer.pop(); // delete end byte

    return packet;
}





SRTxPacket PacketBuffer::fetch_packet()
{
    SRTxPacket packet;
    packet.size = input_buffer.peek(1);
    packet.ctxt = input_buffer.peek(2);
    packet.addr = input_buffer.peek(3);
    packet.trgt = input_buffer.peek(4);

    //copy the data bytes (if any exist)
    if (packet.size > PKT_MINSIZE)
    {
        int data_size = packet.size - PKT_MINSIZE;
        int start_idx = PKT_MINSIZE - 1;

        for (int ix = 0; ix < data_size; ix++)
        {
            packet.data[ix] = input_buffer.peek(start_idx + ix);
        }
    }

    return packet;
}




void PacketBuffer::spin()
{
    for (int ix = 0; ix < MAX_S_BYTES; ix++)
    {
        if (Serial.available())
        {
            input_buffer.push(Serial.read());
        }
        ix++; // this is intentionally outside of the if statement
        // consequently, ix really tracks copy *attempts*, not necessarily copies -> which functions like a quasi-timeout
    }
}




bool PacketBuffer::find_packet_start()
{
    while (input_buffer.size() >= PKT_MINSIZE)
    {
        uint8_t next_byte = input_buffer.peek();
        if (PKT_START == next_byte)
        {
            return true;
        }
        else
        {
            input_buffer.del(); //deletes byte if it is not PKT_START
        }
    }

    return false;
}




bool PacketBuffer::packet_available()
{
    while (input_buffer.size() >= PKT_MINSIZE)
    {
        if (find_packet_start())
        {
            // at this point, we can assume we are at the start of a (not necessarily complete or valid) packet
            // so -> check to see if buffer contains enough data to form a full packet
            // since size is variable, we pull this from the packet itself 
            uint8_t packet_size = input_buffer.peek(1); //second byte, hence offset = 1
            //TODO: check that PSIZE_MIN <= packet_size <= PSIZE_MAX

            if (input_buffer.size() >= packet_size)
            {
                //is the packet valid?
                uint8_t last_byte = input_buffer.peek(packet_size - 1);
                if (PKT_END == last_byte)
                {
                    return true; // we found a valid packet! nice.
                }
                else
                {
                    //we found an invalid packet, delete it from the buffer, but keep going from the top if possible
                    delete_packet(packet_size);
                }
            }
            else
            {
                return false; //the buffer contains a partial packet, but not enough data to read it
            }
        }
    }

    return false; //the buffer does not contain enough data to hold a full packet
}




void PacketBuffer::delete_packet(uint8_t packet_size)
{
    for (int ix = 0; ix < packet_size; ix++)
    {
        input_buffer.del();
    }
}
