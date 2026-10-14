"""Waveshare Pico-LCD-1.14 V2 (ST7789), 240 x 135 landscape.

Pin assignments, orientation, offsets and tuning registers follow the Waveshare
example. Reset/wake timing follows the ST7789VW controller specification.
Colours are byte-swapped for the little-endian MicroPython RGB565 framebuffer
sent directly over SPI (the controller expects the most-significant byte first).
"""

from machine import Pin, SPI
import framebuf
import utime

BL = 13
DC = 8
RST = 12
MOSI = 11
SCK = 10
CS = 9


def rgb565(red, green, blue):
    colour = ((red & 0xf8) << 8) | ((green & 0xfc) << 3) | (blue >> 3)
    return ((colour & 0xff) << 8) | (colour >> 8)


class LCD_1inch14(framebuf.FrameBuffer):
    def __init__(self):
        self.width = 240
        self.height = 135
        self.cs = Pin(CS, Pin.OUT, value=1)
        self.rst = Pin(RST, Pin.OUT, value=1)
        self.dc = Pin(DC, Pin.OUT, value=1)
        self.spi = SPI(1, baudrate=10000000, polarity=0, phase=0,
                       sck=Pin(SCK), mosi=Pin(MOSI), miso=None)
        self.buffer = bytearray(self.height * self.width * 2)
        super().__init__(self.buffer, self.width, self.height, framebuf.RGB565)
        self.red = rgb565(255, 0, 0)
        self.green = rgb565(0, 255, 0)
        self.blue = rgb565(0, 0, 255)
        self.white = rgb565(255, 255, 255)
        self.init_display()

    def write_cmd(self, command):
        self.dc(0)
        self.cs(0)
        try:
            self.spi.write(bytes((command,)))
        finally:
            self.cs(1)

    def write_data(self, data):
        self.dc(1)
        self.cs(0)
        try:
            self.spi.write(bytes((data,)) if isinstance(data, int) else data)
        finally:
            self.cs(1)

    def _command(self, command, data=b''):
        self.write_cmd(command)
        if data:
            self.write_data(data)

    def init_display(self):
        self.rst(1)
        utime.sleep_ms(5)
        self.rst(0)
        utime.sleep_ms(20)  # Reset pulse exceeds the specified 10 us minimum.
        self.rst(1)
        utime.sleep_ms(120)  # Sleep-out may not be sent sooner after reset.
        self._command(0x36, b'\x70')  # Existing landscape orientation; RGB order.
        self._command(0x3a, b'\x05')  # 16-bit RGB565 serial interface.
        self._command(0xb2, b'\x0c\x0c\x00\x33\x33')
        self._command(0xb7, b'\x35')
        self._command(0xbb, b'\x19')
        self._command(0xc0, b'\x2c')
        self._command(0xc2, b'\x01')
        self._command(0xc3, b'\x12')
        self._command(0xc4, b'\x20')
        self._command(0xc6, b'\x0f')
        self._command(0xd0, b'\xa4\xa1')
        self._command(0xe0, b'\xd0\x04\x0d\x11\x13\x2b\x3f\x54\x4c\x18\x0d\x0b\x1f\x23')
        self._command(0xe1, b'\xd0\x04\x0c\x11\x13\x2c\x3f\x44\x51\x2f\x1f\x1f\x20\x23')
        self._command(0x21)  # Panel requires display inversion, as in vendor demo.
        self._command(0x11)  # Sleep out.
        utime.sleep_ms(120)  # Exceeds the required 5 ms command settling time.
        self._command(0x29)  # Display on.

    def show(self):
        # Same visible GRAM window as the original driver: 240 x 135 pixels.
        self._command(0x2a, b'\x00\x28\x01\x17')
        self._command(0x2b, b'\x00\x35\x00\xbb')
        self.write_cmd(0x2c)
        self.write_data(self.buffer)
