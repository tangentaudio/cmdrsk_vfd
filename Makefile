# Makefile for cmdrsk_vfd — Commander SK VFD LinuxCNC userspace component
#
# Supports two build modes:
#
#   RIP (Run In Place) — default, builds against ~/devel/linuxcnc
#   make
#   make LCNC_HOME=/path/to/linuxcnc   — override RIP path
#
#   Installed — builds against system-installed LinuxCNC headers
#   make INSTALLED=1
#
# Requires: libmodbus-dev
#   sudo apt install libmodbus-dev

PREFIX    ?= /usr/bin
LCNC_HOME ?= $(HOME)/devel/linuxcnc

# --------------------------------------------------------------------------
# LinuxCNC header / library paths
# --------------------------------------------------------------------------
ifdef INSTALLED
  LCNC_INC  := /usr/include/linuxcnc
  LCNC_LIB  := /lib
  LCNC_RPATH := /lib
else
  LCNC_INC  := $(LCNC_HOME)/include
  LCNC_LIB  := $(LCNC_HOME)/lib
  LCNC_RPATH := $(LCNC_HOME)/lib
endif

# --------------------------------------------------------------------------
# Build flags
# --------------------------------------------------------------------------
CC      ?= gcc

CFLAGS  := -I/usr/include \
           -I$(LCNC_INC) \
           -URTAPI -U__MODULE__ -ULAPI -DULAPI \
           -DDEBUG \
           -Os -Wall -Wextra \
           $(shell pkg-config --cflags libmodbus)

LDFLAGS := -L$(LCNC_LIB) \
           -Wl,-rpath,$(LCNC_RPATH) \
           $(shell pkg-config --libs libmodbus) \
           -llinuxcnchal \
           -llinuxcncini

TARGET  := cmdrsk_vfd
SRCS    := cmdrsk_vfd.c

.PHONY: all install clean

all: $(TARGET)

$(TARGET): $(SRCS) cmdrsk_vfd.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

install: $(TARGET)
	install -m 755 $(TARGET) $(PREFIX)/$(TARGET)

clean:
	rm -f $(TARGET)
