#include "SRTxMCU.h"

SensorChannel::SensorChannel(int addr, int pin_cs, int pin_rdy) : MCUChannel(), PIN_CS(pin_cs), PIN_RDY(pin_rdy)
{
    x_mG = 0;
    y_mG = 0;
    z_mG = 0;
    chip_ok = false;
    core.attach(this, addr);
}



void SensorChannel::begin()
{
    pinMode(PIN_CS, OUTPUT);
    pinMode(PIN_RDY, INPUT);
    delayMicroseconds(100);
    check_chip();
    if (chip_ok) { config_chip(); }
    if (chip_ok && digitalRead(PIN_RDY) == HIGH) { get_baseline(); }
}

void SensorChannel::at_connect()
{
    disable_cs();
    delayMicroseconds(100);
    check_chip();
}

void SensorChannel::at_disconnect()
{
    disable_cs();
}



void SensorChannel::spin()
{
    if (chip_ok && digitalRead(PIN_RDY) == HIGH){ read_chip(); }
}

void SensorChannel::enable_cs() { digitalWrite(PIN_CS, LOW); }

void SensorChannel::disable_cs() { digitalWrite(PIN_CS, HIGH); }

void SensorChannel::get_baseline()
{
    spi_beginTransaction();
    uint8_t data_reg[6] = {0};
    int16_t data16bit[3] = {0};

    enable_cs();
    //YO: this doesn't match, (0xC0 | 0x28) represents "11101000"
    spi_transfer(0xC0 | 0x28);     //10101000: 0x80 represents read mode, 0x28 represents OUT_X_L register
    data_reg[0] = spi_transfer(0); //OUT_X_L register
    data_reg[1] = spi_transfer(0); //OUT_X_H register
    data_reg[2] = spi_transfer(0); //OUT_Y_L register
    data_reg[3] = spi_transfer(0); //OUT_Y_H register
    data_reg[4] = spi_transfer(0); //OUT_Z_L register
    data_reg[5] = spi_transfer(0); //OUT_Z_H register
    disable_cs();

    //TODO: clean up the following (esp. typecasting)
    /* Get 16-bit data, raw data unit is "count or LSB" */ //FIX: then why is SPI set to MSBFIRST?
    data16bit[0] = (int16_t)(data_reg[1] << 8 | data_reg[0]);
    data16bit[1] = (int16_t)(data_reg[3] << 8 | data_reg[2]);
    data16bit[2] = (int16_t)(data_reg[5] << 8 | data_reg[4]);

    x_mG_base = (int16_t)((1000 * data16bit[0]) / LIS3MDL_FROM_FS_16G_TO_G); //- avg_chip_mG[0][ix];
    y_mG_base = (int16_t)((1000 * data16bit[1]) / LIS3MDL_FROM_FS_16G_TO_G); //- avg_chip_mG[1][ix];
    z_mG_base = (int16_t)((1000 * data16bit[2]) / LIS3MDL_FROM_FS_16G_TO_G); //- avg_chip_mG[2][ix];

    spi_endTransaction();
    z_filt[0] = z_mG_base;
    z_filt[1] = z_mG_base; 
    z_filt[2] = z_mG_base;
}

void SensorChannel::config_chip()
{
    spi_beginTransaction(); // start SPI
    delayMicroseconds(1000);

    /******************************************************************************************************************
    Register: Control 2
    Address: 0x21
    BIT 7  |  BIT 6  |  BIT 5  |   BIT 4  |  BIT 3   |   BIT 2   |  BIT 1 |   BIT 0
      0    |  FS1    |   FS0   |     0    |  REBOOT  |  SOFT_RST |   0    |     0
    + FS0 & FS0: (Control the full scale)
        FS1   FS0     Full- Scale [G]    LSB/G
         0     0        +-4              6842
         0     1        +-8              3421
         1     0        +-12             2281
         1     1        +-16             1711
    **********************************************************************************************************************/
    enable_cs();       // RESET
    spi_transfer(0x21 | 0x40); // 0b01100001: 0x00 represents write mode, 0x21 represents Register: Control 2
    spi_transfer(0x08);        // b00001000
    disable_cs();
    delay(10); //NOTE: this needs to be this long - tested

    enable_cs();       // Set FS = 16
    spi_transfer(0x21 | 0x40); // 0b01100001: 0x00 represents write mode, 0x21 represents Register: Control 2
    spi_transfer(0x60);        //
    disable_cs();
    delayMicroseconds(100);

    /******************************************************************************************************************
    Register: Control 1
    Address: 0x20
    BIT 7    |  BIT 6  |  BIT 5  |   BIT 4  |  BIT 3   |   BIT 2   |     BIT 1    |   BIT 0
    TEMP_EN   |  OM1    |   OM0   |    DO2   |   DO1    |    DO0    |   FAST_ODR   |    ST
    **********************************************************************************************************************/
    enable_cs();
    spi_transfer(0x20 | 0x40); // 0b00100000: 0x00 represents write mode, 0x20 represents Register: Control 1
    spi_transfer(0x02);        // b00000010
    disable_cs();
    delayMicroseconds(100);

    /******************************************************************************************************************
    Register: Control 4
    Address: 0x23
    BIT 7    |  BIT 6  |  BIT 5  |   BIT 4  |  BIT 3   |   BIT 2   |     BIT 1    |   BIT 0
      0      |    0    |    0    |     0    |   OMZ1   |    OMZ0   |      BLE     |    0
    **********************************************************************************************************************/
    enable_cs();
    spi_transfer(0x23 | 0x40); // 0b00100011 : 0x00 represents write mode, 0x23 represents Register: Control 4
    spi_transfer(0x00);        // b00000000 - low power, 5.3 mgauss RMS noise, start up time 1.2ms, TON duration 0.9ms
    disable_cs();
    delayMicroseconds(100);

    /******************************************************************************************************************
    Register: Control 3
    Address: 0x22
    BIT 7    |  BIT 6  |  BIT 5  |   BIT 4  |  BIT 3   |   BIT 2   |     BIT 1    |   BIT 0
      0      |    0    |    LP   |     0    |    0     |    SIM    |      MD1     |    MD0
    **********************************************************************************************************************/
    enable_cs();
    spi_transfer(0x22 | 0x40); // 0b00100010: 0x00 represents write mode, 0x22 represents Register: Control 3
    spi_transfer(0x00);        // b00000000 - continuous measurement mode
    disable_cs();
    delayMicroseconds(100);

    /******************************************************************************************************************
    Register: Control 5
    Address: 0x24
      BIT 7    |  BIT 6  |  BIT 5  |   BIT 4  |  BIT 3   |   BIT 2   |     BIT 1    |   BIT 0
    FAST_READ   |   BDU   |    0    |     0    |   0      |     0     |      0       |    0
    **********************************************************************************************************************/
    enable_cs();
    spi_transfer(0x24 | 0x40); // 0b00100100: 0x00 represents write mode, 0x24 represents Register: Control 5
    spi_transfer(0x00);        // b00000000
    disable_cs();
    delayMicroseconds(400);

    //    Ending the SPI transaction
    spi_endTransaction();
}


void SensorChannel::check_chip()
{
    spi_beginTransaction();
    enable_cs();
    spi_transfer(0x0F | 0xC0);    // 0b10001111: 0x80 represents write mode, 0x21 represents Register: Control 2
    uint8_t chip_id = spi_transfer(0x00); // b00000000
    delayMicroseconds(100);
    disable_cs();
    if (chip_id == 61) { chip_ok = true; }
    else { chip_ok = false; }
    spi_endTransaction();
}


void SensorChannel::read_chip()
{
    spi_beginTransaction();
    uint8_t data_reg[6] = {0};
    int16_t data16bit[3] = {0};

    enable_cs();
    //YO: this doesn't match, (0xC0 | 0x28) represents "11101000"
    spi_transfer(0xC0 | 0x28);     //10101000: 0x80 represents read mode, 0x28 represents OUT_X_L register
    data_reg[0] = spi_transfer(0); //OUT_X_L register
    data_reg[1] = spi_transfer(0); //OUT_X_H register
    data_reg[2] = spi_transfer(0); //OUT_Y_L register
    data_reg[3] = spi_transfer(0); //OUT_Y_H register
    data_reg[4] = spi_transfer(0); //OUT_Z_L register
    data_reg[5] = spi_transfer(0); //OUT_Z_H register
    disable_cs();

    //TODO: clean up the following (esp. typecasting)
    /* Get 16-bit data, raw data unit is "count or LSB" */ //FIX: then why is SPI set to MSBFIRST?
    data16bit[0] = (int16_t)(data_reg[1] << 8 | data_reg[0]);
    data16bit[1] = (int16_t)(data_reg[3] << 8 | data_reg[2]);
    data16bit[2] = (int16_t)(data_reg[5] << 8 | data_reg[4]);

    x_mG = (int16_t)((1000 * data16bit[0]) / LIS3MDL_FROM_FS_16G_TO_G); //- avg_chip_mG[0][ix];
    y_mG = (int16_t)((1000 * data16bit[1]) / LIS3MDL_FROM_FS_16G_TO_G); //- avg_chip_mG[1][ix];
    
    z_filt[0] = z_filt[1];
    z_filt[1] = z_filt[2];
    z_filt[2] = (int16_t)((1000 * data16bit[2]) / LIS3MDL_FROM_FS_16G_TO_G); //- avg_chip_mG[2][ix];

    unsigned medtry1 = (z_filt[1] <= z_filt[0]) + (z_filt[2] <= z_filt[0]);
    unsigned medtry2 = (z_filt[0] <= z_filt[1]) + (z_filt[2] <= z_filt[1]);
    // unsigned less_than_c = (z_filt[0] <= z_filt[2]) + (z_filt[1] <= z_filt[2]);
    double median;

    if (medtry1 == 1) {median = z_filt[0];}
    else if (medtry2 == 1) {median = z_filt[1];}
    else {median = z_filt[2];}
    
    if (abs(z_filt[2]-median) > 300) {z_mG = z_filt[1];}
    else if (abs(z_filt[2]) < 200) {z_mG = z_filt[1]; z_filt[2] = median;}
    else {z_mG = z_filt[2];}

    spi_endTransaction();           
}






















bool SensorChannel::run_cmd(uint8_t cmd)
{
    switch (cmd)
    {
    case CMD_CHECK_CHIP:
        check_chip();
        return true;

    case CMD_CONFIG_CHIP:
        config_chip();
        return true;

    case CMD_FORCE_READ:
        read_chip();
        return true;

    default:
        return false;
    }
}






bool SensorChannel::set_data(uint8_t trgt, uint8_t *data)
{
    switch (trgt)
    {
    case TGT_CHIP_OK:
        chip_ok = decode_bool(data);
        return true;

    // don't have the others, manually setting a sensor value makes no sense

    default:
        return false;
    }
}

bool SensorChannel::get_data(uint8_t trgt, uint8_t *data)
{
    switch (trgt)
    {
    case TGT_CHIP_OK:
        encode_bool(data, chip_ok);
        return true;

    case TGT_X_MG:
        encode_int16(data, x_mG);
        return true;

    case TGT_Y_MG:
        encode_int16(data, y_mG);
        return true;

    case TGT_Z_MG:
        encode_int16(data, z_mG);
        return true;

    default:
        return false;
    }
}


int SensorChannel::data_size(uint8_t trgt)
{
    switch (trgt)
    {
    case TGT_CHIP_OK:
        return SIZE_BOOL;

    case TGT_X_MG:
    case TGT_Y_MG:
    case TGT_Z_MG:
        return SIZE_INT16;
    
    default:
        return 0;
    }
}
