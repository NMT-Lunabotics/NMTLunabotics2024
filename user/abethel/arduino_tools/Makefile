SRCDIR = ../../../arduino

mirrors: big_motors_can bucket_actuator_can dual_actuators_can

big_motors_can: $(SRCDIR)/big_motors_can
	mkdir -p $@
	cp $</big_motors_can.ino $@/big_motors_can.cpp
	cp $</can_msg_def.h $@/can_msg_def.hpp
	cp -L $</main_bus.hpp $@/
	sed -e 's:some_source.cpp:big_motors_can.cpp can_msg_def.hpp main_bus.hpp:' \
	    ./arduino_makefile.mk > $@/Makefile

bucket_actuator_can: $(SRCDIR)/bucket_actuator_can
	mkdir -p $@
	cp $</bucket_actuator_can.ino $@/bucket_actuator_can.cpp
	cp -L $</main_bus.hpp $@/
	cp -L $</arduino_lib.hpp $@/
	cp $</helpers.hpp $@/
	sed -e 's:some_source.cpp:bucket_actuator_can.cpp main_bus.hpp arduino_lib.hpp helpers.hpp:' \
	    ./arduino_makefile.mk > $@/Makefile

dual_actuators_can: $(SRCDIR)/dual_actuators_can
	mkdir -p $@
	cp $</dual_actuators_can.ino $@/dual_actuators_can.cpp
	cp -L $</main_bus.hpp $@/
	cp -L $</arduino_lib.hpp $@/
	cp $</helpers.hpp $@/
	sed -e 's:some_source.cpp:dual_actuators_can.cpp main_bus.hpp arduino_lib.hpp helpers.hpp:' \
	    ./arduino_makefile.mk > $@/Makefile
