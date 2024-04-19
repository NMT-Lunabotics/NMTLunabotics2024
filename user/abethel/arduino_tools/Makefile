SRCDIR = ../../../arduino

mirrors: big_motors_can bucket_actuator_can dual_actuators_can can_test leds
commit: big_motors_can_commit bucket_actuator_can_commit dual_actuators_can_commit can_test_commit leds_commit

big_motors_can: $(SRCDIR)/big_motors_can
	mkdir -p $@
	cp $</big_motors_can.ino $@/big_motors_can.cpp
	cp $</can_msg_def.h $@/can_msg_def.hpp
	cp -L $</main_bus.hpp $@/
	sed -e 's:some_source.cpp:big_motors_can.cpp can_msg_def.hpp main_bus.hpp:' \
	    ./arduino_makefile.mk > $@/Makefile

# this sucks so hard
big_motors_can_commit:
	cp big_motors_can/big_motors_can.cpp $(SRCDIR)/big_motors_can/big_motors_can.ino
	cp big_motors_can/can_msg_def.hpp $(SRCDIR)/big_motors_can/can_msg_def.h

can_test: $(SRCDIR)/can_test
	mkdir -p $@
	cp $</can_test.ino $@/can_test.cpp
	cp $</helpers.hpp $@/helpers.hpp
	cp -L $</main_bus.hpp $@/
	cp -L $</arduino_lib.hpp $@/
	sed -e 's:some_source.cpp:can_test.cpp helpers.hpp main_bus.hpp arduino_lib.hpp:' \
	    ./arduino_makefile.mk > $@/Makefile

can_test_commit:
	cp can_test/can_test.cpp $(SRCDIR)/can_test/can_test.ino
	cp can_test/helpers.hpp $(SRCDIR)/can_test/helpers.hpp

leds: $(SRCDIR)/leds
	mkdir -p $@
	cp $</leds.cpp $@/leds.cpp
	cp -L $</main_bus.hpp $@/
	cp -L $</arduino_lib.hpp $@/
	sed -e 's:some_source.cpp:leds.cpp main_bus.hpp arduino_lib.hpp:' \
	    ./arduino_makefile.mk > $@/Makefile

leds_commit:
	cp leds/leds.cpp $(SRCDIR)/can_test/leds.cpp

bucket_actuator_can: $(SRCDIR)/bucket_actuator_can
	mkdir -p $@
	cp $</bucket_actuator_can.ino $@/bucket_actuator_can.cpp
	cp -L $</main_bus.hpp $@/
	cp -L $</arduino_lib.hpp $@/
	cp $</helpers.hpp $@/
	sed -e 's:some_source.cpp:bucket_actuator_can.cpp main_bus.hpp arduino_lib.hpp helpers.hpp:' \
	    ./arduino_makefile.mk > $@/Makefile

bucket_actuator_can_commit:
	cp bucket_actuator_can/bucket_actuator_can.cpp $(SRCDIR)/bucket_actuator_can/bucket_actuator_can.ino
	cp bucket_actuator_can/helpers.hpp $(SRCDIR)/bucket_actuator_can/helpers.hpp

dual_actuators_can: $(SRCDIR)/dual_actuators_can
	mkdir -p $@
	cp $</dual_actuators_can.ino $@/dual_actuators_can.cpp
	cp -L $</main_bus.hpp $@/
	cp -L $</arduino_lib.hpp $@/
	cp $</helpers.hpp $@/
	sed -e 's:some_source.cpp:dual_actuators_can.cpp main_bus.hpp arduino_lib.hpp helpers.hpp:' \
	    ./arduino_makefile.mk > $@/Makefile

dual_actuators_can_commit:
	cp dual_actuators_can/dual_actuators_can.cpp $(SRCDIR)/dual_actuators_can/dual_actuators_can.ino
	cp dual_actuators_can/helpers.hpp $(SRCDIR)/dual_actuators_can/helpers.hpp
