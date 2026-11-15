APP_NAME := tests
ARDUINO_LIBS := AUnit
EPOXY_DUINO_DIR ?= ../EpoxyDuino
AUNIT_DIR ?= ../AUnit

# The drivers need C++17 and the concurrency tests need threads (and
# thread-safe statics, which EpoxyDuino's defaults turn off).
CXXFLAGS := -Wextra -Wall -std=gnu++17 -pthread -g
LDFLAGS += -pthread

# Swap TwoWire for the fake bus in FakeWire.hpp (found via -I.).
CPPFLAGS += -I. -DI2CS_FAKE_WIRE
CPPFLAGS += -I$(AUNIT_DIR)/src

# Library sources. The test files in this directory are picked up automatically.
OBJS += ../src/I2CManager.o
OBJS += ../src/I2CSensor.o
OBJS += ../src/SHT3X/SHT31D.o
OBJS += ../src/BMP3XX/BMP3XX.o
OBJS += ../src/MPU6050/MPU6050.o

# EpoxyDuino doesn't track header dependencies - without this, editing a
# header leaves stale objects and the tests run against old code.
CPPFLAGS += -MMD -MP

include $(EPOXY_DUINO_DIR)/EpoxyDuino.mk

-include $(OBJS:.o=.d)

clean-deps:
	rm -f $(OBJS:.o=.d)
