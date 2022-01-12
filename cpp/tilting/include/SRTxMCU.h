#ifndef SRTX_H
#define SRTX_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>


#include "CircularBuffer.h"
#include "protocol.h"




#define N_CALLBACKS 4


const uint32_t USEC_1HZ = 1000000;
const uint32_t USEC_10HZ = 100000;
const uint32_t USEC_100HZ = 10000;
const uint32_t USEC_2000Hz = 500;
const uint32_t USEC_1000HZ = 1000;


struct SRTxPacket
{
    uint8_t size;
    uint8_t ctxt;
    uint8_t addr;
    uint8_t trgt;
    uint8_t data[PKT_MAXSIZE-PKT_MINSIZE];
};

// functions to encode/decode data for packets //(see codec.cpp)
bool decode_bool(uint8_t *data);
void encode_bool(uint8_t *data, bool value);
const uint8_t SIZE_BOOL = 1;

uint8_t decode_uint8(uint8_t *data);
void encode_uint8(uint8_t *data, uint8_t value);
const uint8_t SIZE_UINT8 = 1;

uint16_t decode_uint16(uint8_t *data);
void encode_uint16(uint8_t *data, uint16_t value);
const uint8_t SIZE_UINT16 = 2;

uint32_t decode_uint32(uint8_t *data);
void encode_uint32(uint8_t *data, uint32_t value);
const uint8_t SIZE_UINT32 = 4;

int8_t decode_int8(uint8_t *data);
void encode_int8(uint8_t *data, int8_t value);
const uint8_t SIZE_INT8 = 1;

int16_t decode_int16(uint8_t *data);
void encode_int16(uint8_t *data, int16_t value);
const uint8_t SIZE_INT16 = 2;

int32_t decode_int32(uint8_t *data);
void encode_int32(uint8_t *data, int32_t value);
const uint8_t SIZE_INT32 = 4;








#include "System.h" //led and clock
#include "PacketBuffer.h"
#include "MCUChannel.h"
#include "MCUCore.h"

// THE core - call it's public member fxns globally
extern MCUCore core;


#include "Channels.h"










#endif
