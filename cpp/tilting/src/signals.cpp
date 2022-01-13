#include "signals.hpp"

SensorChannel chnl_7(7, 2, 30);
SensorChannel chnl_8(8, 3, 29);
SensorChannel chnl_9(9, 4, 28);
SensorChannel chnl_10(10, 5, 27);
SensorChannel chnl_11(11, 6, 26);
SensorChannel chnl_12(12, 7, 25);

int signals[NUM_CHANNELS];

static const uint8_t CS_PINS[MAG_SENSOR_NUM] = {2, 3, 4, 5, 6, 7};  
static const uint8_t RDY_PINS[MAG_SENSOR_NUM] = {30, 29, 28, 27, 26, 25}; 
static const uint8_t DISP_PIN = 39;                   

byte Chip_ID;

uint8_t data_reg[6] = {0};
int data16bit[3] = {0};
float magnetic_mG[3][MAG_SENSOR_NUM] = {};
float magnetic_mG_old[MAG_SENSOR_NUM] = {7000, 7000};
unsigned long loop_timing = 0;
unsigned long loop_control_period;

int upper_range = 100;
int lower_range = 0;

int mytime = 0;

float avg_chip_mG[3][MAG_SENSOR_NUM] = {};

void setup_signal_collector(){
    chnl_7.begin();
    chnl_8.begin();
    chnl_9.begin();
    chnl_10.begin();
    chnl_11.begin();
    chnl_12.begin();
    delay(3000);
    loop_control_period = round(1000000 * (1.0 / LOOP_FREQUENCY)); //This is for loop control If you use micros() for timing, change 1000 to 1000000
    pinMode(DISP_PIN, INPUT);
    for (int i = 0; i < MAG_SENSOR_NUM; i++)
    {
        pinMode(CS_PINS[i], OUTPUT);
        digitalWriteFast(CS_PINS[i], HIGH);
        pinMode(RDY_PINS[i], INPUT);
    }

    pinMode(PROBEPIN, OUTPUT);
    digitalWriteFast(PROBEPIN, LOW);
    // pinMode(SPI1_SCK, OUTPUT);
    // pinMode(SPI1_MOSI, OUTPUT);
    // pinMode(SPI1_MISO, INPUT);

    // SPI1.setSCK(SPI1_SCK);
    // SPI1.setMOSI(SPI1_MOSI);
    // SPI1.setMISO(SPI1_MISO);
//    SPI1.begin(); //start the spi-bus
    SPI.begin(); //start the spi-bus

    delay(100);

    SPI.beginTransaction(SPISettings(SPISPEED, MSBFIRST, SPI_MODE0)); // start SPI
    delay(1000);

    for (int i = 0; i < MAG_SENSOR_NUM; i++)
    {
        digitalWriteFast(CS_PINS[i], LOW);
        SPI.transfer(0x0F | 0xC0);    // 0b10001111: 0x80 represents write mode, 0x21 represents Register: Control 2
        Chip_ID = SPI.transfer(0x00); // b00000000
        digitalWriteFast(CS_PINS[i], HIGH);
        delayMicroseconds(100);
        if (Chip_ID == 61)
        {
            Serial.print("Identity of LIS3MDL# ");
            Serial.print(i + 1);
            Serial.println(" checked!");
        }
        Chip_ID = 0;
    }
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
    toggle_CS_PINS(LOW);       // RESET
    SPI.transfer(0x21 | 0x40); // 0b01100001: 0x00 represents write mode, 0x21 represents Register: Control 2
    SPI.transfer(0x08);        // b00001000
    toggle_CS_PINS(HIGH);
    delay(10);

    toggle_CS_PINS(LOW);       // Set FS = 16
    SPI.transfer(0x21 | 0x40); // 0b01100001: 0x00 represents write mode, 0x21 represents Register: Control 2
    SPI.transfer(0x60);        //
    toggle_CS_PINS(HIGH);
    delayMicroseconds(100);

    /******************************************************************************************************************
    Register: Control 1
    Address: 0x20
    BIT 7    |  BIT 6  |  BIT 5  |   BIT 4  |  BIT 3   |   BIT 2   |     BIT 1    |   BIT 0
    TEMP_EN   |  OM1    |   OM0   |    DO2   |   DO1    |    DO0    |   FAST_ODR   |    ST
  **********************************************************************************************************************/
    toggle_CS_PINS(LOW);
    SPI.transfer(0x20 | 0x40); // 0b00100000: 0x00 represents write mode, 0x20 represents Register: Control 1
    SPI.transfer(0x02);        // b00000010
    toggle_CS_PINS(HIGH);
    delayMicroseconds(100);

    /******************************************************************************************************************
    Register: Control 4
    Address: 0x23
    BIT 7    |  BIT 6  |  BIT 5  |   BIT 4  |  BIT 3   |   BIT 2   |     BIT 1    |   BIT 0
      0      |    0    |    0    |     0    |   OMZ1   |    OMZ0   |      BLE     |    0
  **********************************************************************************************************************/
    toggle_CS_PINS(LOW);
    SPI.transfer(0x23 | 0x40); // 0b00100011 : 0x00 represents write mode, 0x23 represents Register: Control 4
    SPI.transfer(0x00);        // b00000000 - low power, 5.3 mgauss RMS noise, start up time 1.2ms, TON duration 0.9ms
    toggle_CS_PINS(HIGH);
    delayMicroseconds(100);

    /******************************************************************************************************************
    Register: Control 3
    Address: 0x22
    BIT 7    |  BIT 6  |  BIT 5  |   BIT 4  |  BIT 3   |   BIT 2   |     BIT 1    |   BIT 0
      0      |    0    |    LP   |     0    |    0     |    SIM    |      MD1     |    MD0
  **********************************************************************************************************************/
    toggle_CS_PINS(LOW);
    SPI.transfer(0x22 | 0x40); // 0b00100010: 0x00 represents write mode, 0x22 represents Register: Control 3
    SPI.transfer(0x00);        // b00000000 - continuous measurement mode
    toggle_CS_PINS(HIGH);
    delayMicroseconds(100);

    /******************************************************************************************************************
    Register: Control 5
    Address: 0x24
      BIT 7    |  BIT 6  |  BIT 5  |   BIT 4  |  BIT 3   |   BIT 2   |     BIT 1    |   BIT 0
    FAST_READ   |   BDU   |    0    |     0    |   0      |     0     |      0       |    0
  **********************************************************************************************************************/
    toggle_CS_PINS(LOW);
    SPI.transfer(0x24 | 0x40); // 0b00100100: 0x00 represents write mode, 0x24 represents Register: Control 5
    SPI.transfer(0x00);        // b00000000
    toggle_CS_PINS(HIGH);
    delay(100);

    Serial.println("Initialization complete");
    delay(100);

    //    Ending the SPI transaction
    SPI.endTransaction();

    while (!Serial && (millis() <= 5000))
        ; // WAIT UP TO 5000 MILLISECONDS FOR SERIAL OUTPUT CONSOLE
    Serial.println("configured, starting");
    Serial.println("");
    delay(100);
    // Serial.println("Time(millis) Disp(mm) X-axis     Y-axis      Z-axis");
    Serial.println("Time(millis)     X-axis     Y-axis      Z-axis");

    delay(2000);
    SPI.beginTransaction(SPISettings(SPISPEED, MSBFIRST, SPI_MODE0)); // start SPI

    //  //find the baseline readings before starting the code
    //  for (int chip_iterator = 0; chip_iterator < MAG_SENSOR_NUM; chip_iterator++) {
    //    calibration(chip_iterator);
    //    for (int j = 0; j <= 2; j++) {
    //      Serial.print(avg_chip_mG[j][chip_iterator]);
    //      Serial.print(" ");
    //    }
    //  }

    // Serial.println("Calibration Over!");
}

void collect_signal(float * signal_to_read, float* calibration_vector, 
                                            int signal_size)
{
    // channel driver
    // float displacement_laser = analogRead(DISP_PIN);
    // displacement_laser = map(displacement_laser,10,1023,0,3.3); //in mm

    if (digitalRead(RDY_PINS[0]) == HIGH){ // && digitalRead(RDY_PINS[1]) == HIGH && digitalRead(RDY_PINS[2]) == HIGH && digitalRead(RDY_PINS[3]) == HIGH && digitalRead(RDY_PINS[4]) == HIGH && digitalRead(RDY_PINS[5]) == HIGH && digitalRead(RDY_PINS[6]) == HIGH && digitalRead(RDY_PINS[7]) == HIGH && digitalRead(RDY_PINS[8]) == HIGH && digitalRead(RDY_PINS[9]) == HIGH) {
        //       Serial.print("Min:500  ");
        for (int chip_iterator = 0; chip_iterator < MAG_SENSOR_NUM; chip_iterator++)
        {
            digitalWriteFast(CS_PINS[chip_iterator], LOW);
            SPI.transfer(0xC0 | 0x28);     //10101000: 0x80 represents read mode, 0x28 represents OUT_X_L register
            data_reg[0] = SPI.transfer(0); //OUT_X_L register
            data_reg[1] = SPI.transfer(0); //OUT_X_H register
            data_reg[2] = SPI.transfer(0); //OUT_Y_L register
            data_reg[3] = SPI.transfer(0); //OUT_Y_H register
            data_reg[4] = SPI.transfer(0); //OUT_Z_L register
            data_reg[5] = SPI.transfer(0); //OUT_Z_H register
            digitalWrite(CS_PINS[chip_iterator], HIGH);

            /* Get 18bits data, raw data unit is "count or LSB" */
            data16bit[0] = (int)(data_reg[1] << 8 | data_reg[0]);
            data16bit[1] = (int)(data_reg[3] << 8 | data_reg[2]);
            data16bit[2] = (int)(data_reg[5] << 8 | data_reg[4]);

            magnetic_mG[0][chip_iterator] = (1000 / LIS3MDL_FROM_FS_16G_TO_G) * (data16bit[0]); //- avg_chip_mG[0][chip_iterator];
            magnetic_mG[1][chip_iterator] = (1000 / LIS3MDL_FROM_FS_16G_TO_G) * (data16bit[1]); //- avg_chip_mG[1][chip_iterator];
            magnetic_mG[2][chip_iterator] = (1000 / LIS3MDL_FROM_FS_16G_TO_G) * (data16bit[2]); //- avg_chip_mG[2][chip_iterator];

            for (int j = 0; j < 3; j++)
            { //unwrapping of mag data >> check if the baseline changes when this is commented out
                if (magnetic_mG[j][chip_iterator] > 19150)
                {
                    magnetic_mG[j][chip_iterator] -= 38300;
                }
            }
         *(signal_to_read + chip_iterator) = (float)magnetic_mG[0][chip_iterator] / calibration_vector[chip_iterator];
         *(signal_to_read + 6 + chip_iterator) = (float)magnetic_mG[1][chip_iterator] / calibration_vector[6 + chip_iterator];
         *(signal_to_read + 12 + chip_iterator) = (float)magnetic_mG[2][chip_iterator]/ calibration_vector[12 + chip_iterator];
        }
        digitalWriteFast(PROBEPIN, !digitalRead(PROBEPIN));
        while (micros() < (loop_timing + loop_control_period))
        {
            //wait...
        }
        loop_timing = micros();
    }
}

void toggle_CS_PINS(bool low_high)
{
    // 10 sensors max per sensor MCU
    digitalWriteFast(CS_PINS[0], low_high);
    digitalWriteFast(CS_PINS[1], low_high);
    digitalWriteFast(CS_PINS[2], low_high);
    digitalWriteFast(CS_PINS[3], low_high);
    digitalWriteFast(CS_PINS[4], low_high);
    digitalWriteFast(CS_PINS[5], low_high);
}
