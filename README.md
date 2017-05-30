I2C bit banging SMBus for raspberry pi with wiringPi. 

I made this C++ class to use any 2 GPIO pins of the raspberry pi as an I2C Bus. It was thought to be used with MPU9150 breakout boards. As the breakout board only supports 2 sensors on one bus, i wanted to have a solution to get out of the rpi as many I2C busses as possible.

I2C bus 1 (BCM2 and BCM3) have hard-wired 1k8 pull-ups to 3V3. (see http://pinout.xyz/pinout/pin3_gpio2) When you want to use i2c-bit-banging on other GPIO pins you need external pull-up resistors.

It is tested and working with the MPU9150 breakout board. The bit banging bus should work with all i2c devices.

The low level functions are copied from:

http://en.wikipedia.org/wiki/I%C2%B2C#Example_of_bit-banging_the_I.C2.B2C_master_protocol

On top of that i made linux kernel like smbus functions. For example:

`int32_t i2c_smbus_write_byte_data(uint8_t i2c_address, uint8_t command, uint8_t value);`

to make porting of existing i2c code easier.

Maybe a C interface would have been nicer, but it should be very easy to port it to C. I hope this helps someone!

Compile example with:

`g++ -std=c++11 -lwiringPi -o i2cBitBangingBus i2cBitBangingBus.cpp`
