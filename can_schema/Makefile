all: main_bus

%: %.yaml
	canspec.py --hpp $@.hpp --kcd $@.kcd $^
