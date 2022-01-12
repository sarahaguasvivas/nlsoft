#include "SRTxMCU.h"

TestingChannel::TestingChannel(int addr, int pin_sck, int pin_sda) : MCUChannel(), PIN_SCK(pin_sck), PIN_SDA(pin_sda)
{
    // Adafruit_SH110X u8x8 = Adafruit_SH110X(64, 128, &Wire);
    // Adafruit_SH110X u8x8(64, 128, &Wire);

    // U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE, pin_sck, pin_sda);
    // this->display = Adafruit_SH110X(64, 128, &Wire);
    core.attach(this, addr);
}


void TestingChannel::begin()
{
    display.begin(0x3C, true); // Address 0x3C default
    display.clearDisplay();

    display.setRotation(1);
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0,0);


    // display.print("Connecting to SSID\n'adafruit':");
    // display.print("connected!");
    // display.println("IP: 10.0.1.23");
    // display.println("Sending val #0");
    // display.display(); // actually display all of the above
    display.printf("SRTx v%u.%u.%u\n", PROTOCOL_VERSION[0], PROTOCOL_VERSION[1], PROTOCOL_VERSION[2]);
    display.printf("n=%u\n", core.count_channels());
    need_update = false;
    display.display();
}


void TestingChannel::spin()
{
    if (need_update) {update_display();}
}


void TestingChannel::at_connect()
{
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("connected");
    display.display();
}

void TestingChannel::at_disconnect()
{
    need_update = false;
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("disconnected");
    display.display();
    test_bool = 0;
    test_uint8 = 0;
    test_uint16 = 0;
    test_uint32 = 0;
    test_int8 = 0;
    test_int16 = 0;
    test_int32 = 0;
}


bool TestingChannel::run_cmd(uint8_t cmd)
{
    switch (cmd)
    {
    case CMD_RUNTEST:
        test_bool = !test_bool;
        test_uint8++;
        test_uint16++;
        test_uint32++;
        test_int8++;
        test_int16++;
        test_int32++;
        need_update = true;
        return true;

    default:
        return false;
    }
}


bool TestingChannel::set_data(uint8_t trgt, uint8_t *data)
{
    switch (trgt)
    {
    case TGT_T_BOOL:
        test_bool = decode_bool(data);
        need_update = true;
        return true;

    case TGT_T_UINT8:
        test_uint8 = decode_uint8(data);
        need_update = true;
        return true;

    case TGT_T_UINT16:
        test_uint16 = decode_uint16(data);
        need_update = true;
        return true;

    case TGT_T_UINT32:
        test_uint32 = decode_uint32(data);
        need_update = true;
        return true;

    case TGT_T_INT8:
        test_int8 = decode_int8(data);
        need_update = true;
        return true;

    case TGT_T_INT16:
        test_int16 = decode_int16(data);
        need_update = true;
        return true;

    case TGT_T_INT32:
        test_int32 = decode_int32(data);
        need_update = true;
        return true;

    
    default:
        return false;
    }
}


bool TestingChannel::get_data(uint8_t trgt, uint8_t *data)
{
    switch (trgt)
    {
    case TGT_T_BOOL:
        encode_bool(data, test_bool);
        return true;

    case TGT_T_UINT8:
        encode_uint8(data, test_uint8);
        return true;

    case TGT_T_UINT16:
        encode_uint16(data, test_uint16);
        return true;

    case TGT_T_UINT32:
        encode_uint32(data, test_uint32);
        return true;

    case TGT_T_INT8:
        encode_int8(data, test_int8);
        return true;

    case TGT_T_INT16:
        encode_int16(data, test_int16);
        return true;

    case TGT_T_INT32:
        encode_int32(data, test_int32);
        return true;
    
    default:
        return false;
    }
}


int TestingChannel::data_size(uint8_t trgt)
{
    switch (trgt)
    {
    case TGT_T_BOOL:
        return SIZE_BOOL;

    case TGT_T_UINT8:
        return SIZE_UINT8;

    case TGT_T_UINT16:
        return SIZE_UINT16;

    case TGT_T_UINT32:
        return SIZE_UINT32;

    case TGT_T_INT8:
        return SIZE_INT8;

    case TGT_T_INT16:
        return SIZE_INT16;

    case TGT_T_INT32:
        return SIZE_INT32;
    
    default:
        return 0;
    }
}


void TestingChannel::update_display()
{
    need_update = false;
    display.clearDisplay();
    display.setCursor(0,0);
    display.printf("BOOL: %u\n", test_bool);
    display.printf("UINT8: %u\n", test_uint8);
    display.printf("UINT16: %u\n", test_uint16);
    display.printf("UINT32: %u\n", test_uint32);
    display.printf("INT8: %d\n", test_int8);
    display.printf("INT16: %d\n", test_int16);
    display.printf("INT32: %d\n", test_int32);
    display.display();
}