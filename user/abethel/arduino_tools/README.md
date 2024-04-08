Use `make` to generate mirrors of the sketches in `goliath/arduino/`,
that use my makefile setup to manage toolchains, compiling, flashing,
etc., without a dependency on `~/.arduino15`.

Within the resulting directories, use:
- `make compile_flags.txt` to get clangd working
- `make` to build the project
- `make flash` to flash the project (and compile if necessary)
- `make monitor` to watch the serial monitor
