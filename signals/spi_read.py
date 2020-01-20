import spidev

spi_ch = 0

spi = spidev.SpiDev()
spi.open(0, 0)

spi.max_speed_hz = 5000

