CXX = g++
CXXFLAGS = -std=c++11
LDFLAGS = -lwiringPi
 
all: i2cBitBangingBus

i2cBitBangingBus: src/i2cBitBangingBus.cpp
	$(CXX) $(CXXFLAGS) -o i2cBitBangingBus src/i2cBitBangingBus.cpp $(LDFLAGS)

clean:
	-rm i2cBitBangingBus
