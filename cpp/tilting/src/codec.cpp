#include "SRTxMCU.h"


bool decode_bool(uint8_t *data)
{
    return (bool)data[0];
}

void encode_bool(uint8_t *data, bool value)
{
    data[0] = (uint8_t)value;
}





uint8_t decode_uint8(uint8_t *data)
{
    return data[0];
}

void encode_uint8(uint8_t *data, uint8_t value)
{
    data[0] = value;
}





uint16_t decode_uint16(uint8_t *data)
{
    uint16_t value = data[0] | (data[1] << 8);
    return value;
}

void encode_uint16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xff);
    data[1] = (uint8_t)(value >> 8);
}





uint32_t decode_uint32(uint8_t *data)
{
    uint32_t value = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
    return value;
}

void encode_uint32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xff);
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}







int8_t decode_int8(uint8_t *data)
{
    return data[0];
}

void encode_int8(uint8_t *data, int8_t value)
{
    data[0] = value;
}





int16_t decode_int16(uint8_t *data)
{
    int16_t value = data[0] | (data[1] << 8);
    return value;
}

void encode_int16(uint8_t *data, int16_t value)
{
    data[0] = (uint8_t)(value & 0xff);
    data[1] = (uint8_t)(value >> 8);
}






int32_t decode_int32(uint8_t *data)
{
    int32_t value = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
    return value;
}

void encode_int32(uint8_t *data, int32_t value)
{
    data[0] = (uint8_t)(value & 0xff);
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}
