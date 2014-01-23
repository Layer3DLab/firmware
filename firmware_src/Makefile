BOARD_TAG    ?= mega2560
UPLOAD_RATE  ?= 115200
AVRDUDE_ARD_PROGRAMMER ?= wiring
ARDUINO_PORT ?=
ARDUINO_LIBS ?=

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
    # OS X specific variables
    ARDUINO_DIR = /Applications/Arduino.app/Contents/Resources/Java
    ARDMK_PATH = /Applications/Arduino.app/Contents/Resources/Java/
    AVR_TOOLS_DIR = /Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr
    AVRDUDE_CONF = /Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/etc/avrdude.conf
    include /Applications/Arduino.app/Contents/Resources/Java/Arduino.mk
else
    ARDUINO_DIR  = /usr/share/arduino
    include /usr/share/arduino/Arduino.mk
endif

# Automatically add git branch tag and hash to compiled firmware
#GIT_BRANCH := $(shell git rev-parse --abbrev-ref HEAD)
#GIT_TAG_TMP := $(shell git describe --tags --always --match "v[0-9].[0-9]*") #  print out tag or hash if there is no tag
#GIT_HASH := $(shell git rev-parse --short=7 HEAD)

#GIT_TAG := $(strip $(GIT_TAG_TMP))

#CPPFLAGS += -DGIT_BRANCH=\"$(GIT_BRANCH)\" -DGIT_TAG=\"$(GIT_TAG)\" -DGIT_HASH=\"$(GIT_HASH)\"
