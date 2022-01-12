#include "System.h"

SystemLED sys_led;
bool SystemLED::led_state;

SystemClock sys_clock;
uint32_t SystemClock::clock_offset;


#if USE_SPI_BUS == -1
#warning SPI disabled, which is OK if you wanted that
void spi_begin() {}
void spi_endTransaction() {}
void spi_beginTransaction() {}
uint8_t spi_transfer(uint8_t data) { return 0; }
#endif

#if USE_SPI_BUS == 0
void spi_begin() { SPI.begin(); }
void spi_endTransaction() { SPI.endTransaction(); }
void spi_beginTransaction() { SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0)); }
uint8_t spi_transfer(uint8_t data) { return SPI.transfer(data); }
#endif

#if USE_SPI_BUS == 1
void spi_begin() { SPI1.begin(); }
void spi_endTransaction() { SPI1.endTransaction(); }
void spi_beginTransaction() { SPI1.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0)); }
uint8_t spi_transfer(uint8_t data) { return SPI1.transfer(data); }
#endif



//------------------------- led -------------------------//

SystemLED::SystemLED()
{
    pinMode(PIN_LED, OUTPUT);
}

bool SystemLED::get()
{
    return led_state;
}

void SystemLED::toggle()
{
    led_state = !led_state;
    digitalWrite(PIN_LED, led_state);
}

void SystemLED::enable()
{
    led_state = HIGH;
    digitalWrite(PIN_LED, led_state);
}

void SystemLED::disable()
{
    led_state = LOW;
    digitalWrite(PIN_LED, led_state);
}

void SystemLED::standby()
{
    static const bool blink[6] = {HIGH, LOW, HIGH, LOW, LOW, LOW};
    static int ix = 0;

    led_state = blink[ix];
    digitalWrite(PIN_LED, led_state);
    ix++;
    if (ix >= 6) {ix = 0;}
}



//------------------------- clock -------------------------//

void SystemClock::reset() 
{
    clock_offset = micros();
}

uint32_t SystemClock::get()
{
    return micros() - clock_offset;
}

