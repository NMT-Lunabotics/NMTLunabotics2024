# ---- Configuration section ----

SKETCH_NAME = arduino
SOURCES = some_source.cpp

# Board selection.
PLATFORM_NAME = arduino:renesas_uno
FQBN = $(PLATFORM_NAME):unor4wifi
EXTRA_FLAGS = -DARDUINO_UNOWIFIR4

# --------------------------------------------------------------------
#     Nothing below this point should need to be modified normally.

VARS = \
	HOME=$(shell pwd)/toolchain \
	TMPDIR=$(shell pwd)/build/tmp \

ARDUINO = $(VARS) arduino-cli

SKETCH_PATH = build/$(SKETCH_NAME)
TEMP = build/tmp

build: $(SOURCES) toolchain
	$(RM) -r $(SKETCH_PATH) && mkdir -p $(SKETCH_PATH) $(TEMP)
	cp -t $(SKETCH_PATH) $(SOURCES)
	touch $(SKETCH_PATH)/$(SKETCH_NAME).ino
	$(ARDUINO) compile $(SKETCH_PATH) -b $(FQBN)

toolchain:
	$(RM) -r toolchain
	mkdir -p toolchain $(TEMP)
	$(ARDUINO) core install $(PLATFORM_NAME)
	$(RM) -r build
	cd toolchain && ln -s .arduino* arduino

flash: build
	$(ARDUINO) upload build/$(SKETCH_NAME) -b $(FQBN) -p /dev/ttyACM0

monitor: toolchain
	$(ARDUINO) monitor -p /dev/ttyACM0

compile_flags.txt: toolchain
	find ./toolchain/arduino/packages/arduino/hardware \
	  | grep '\.h$$' \
	  | sed 's:/[^/]*$$::' \
	  | sort \
	  | uniq \
	  | sed 's:^./:-I:' > $@
	echo $(EXTRA_FLAGS) >> $@
	for dir in $$(dirname $(SOURCES)); do echo -I$$dir >> $@; done

clean:
	$(RM) -r toolchain build compile_flags.txt
