#ifndef SYS_H
#define SYS_H

#include "SRTxMCU.h"

// for sensors to work, uncomment either:
// #define USE_SPI_BUS 1
// or both of:
// #define USE_SPI_BUS 0
// #define SYSTEM_LED 3 //can be whatever makes sense


#define SPI_SPEED 1000000 // Arduino Mega

#ifndef SYSTEM_LED
#define SYSTEM_LED 13 //only need to change this to use SPI 0
#endif

#ifndef USE_SPI_BUS
#define USE_SPI_BUS -1
#endif


//TODO: comment what these are
#define LIS3MDL_FROM_FS_4G_TO_G 6842
#define LIS3MDL_FROM_FS_8G_TO_G 3421
#define LIS3MDL_FROM_FS_12G_TO_G 2281
#define LIS3MDL_FROM_FS_16G_TO_G 1711


//these functions wrap SPI.fxn, SPI1.fxn, or nothing depending on USE_SPI_BUS
void spi_begin();
void spi_endTransaction();
void spi_beginTransaction();
uint8_t spi_transfer(uint8_t data);




// global system led
class SystemLED {
public:

    SystemLED();

    //get the current state of the built-in LED
    static bool get();
    
    //enable the built-in LED
    static void enable();

    //disable the built-in LED
    static void disable();

    //turn on the LED if off, and vice versa
    static void toggle();

    // iterate thru states in a custom pattern
    static void standby();

    static const int  PIN_LED = SYSTEM_LED;

private:
    static bool led_state;
};


//global system clock
class SystemClock {
public:

    //reset the system clock by saving the current micros()
    static void reset();

    //returns the microseconds elapsed since last clock reset
    static uint32_t get();

private:

    // in micros
    static uint32_t clock_offset;

};


// these both get their own static/global instance
// this is so they can be used by both:
// the MCUCore -> callbacks
// the CoreChannel -> communication


extern SystemLED sys_led;
extern SystemClock sys_clock;




#endif
