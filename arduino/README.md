Configuration for a CANbed that drives the robot's motors. To install
required packages:

    arduino-cli core install arduino:avr
    export ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true
    arduino-cli lib install --git-url https://github.com/Longan-Labs/Longan_CAN_MCP2515
    arduino-cli lib install --git-url https://github.com/Longan-Labs/I2C_CAN_Arduino
