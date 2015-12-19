I2C Bit banging bus for raspberry pi with wiringPi. 

I made this C++ Class to use any 2 GPIO Pins of the raspberry pi as an I2C Bus. It was thought to be used with MPU9250 Breakout boards from Drotec to realize a mocap suit. As the Breakout board only supports 2 sensors on one bus, i wanted to have a solution to get out of the rpi as many I2C busses as possible.

I2C bus 1 (gpios 2 and 3 on rpi2 pins 3 and 5) have hard-wired 1k8 pull-ups to 3V3. pinout.xyz/pinout/pin3_gpio2 When you want to use i2c-bit-banging on other GPIO-Pins you need external pull-up resistors.

It is tested and working with the breakout board. The bit banging bus should work with all i2c devices.

The low level functions are copied from:
http://en.wikipedia.org/wiki/I%C2%B2C

On top of that i made linux kernel like smbus functions. Like

int32_t i2c_smbus_write_byte_data(uint8_t i2c_address, uint8_t command, uint8_t value);

to make porting of existing i2c code easier.

Maybe a C Interface would have been nicer. But it should be very easily portable to C.

I hope this helps someone!

compile example with:
g++ -std=c++11 -lwiringPi -o i2cBitBangingBus i2cBitBangingBus.cpp
