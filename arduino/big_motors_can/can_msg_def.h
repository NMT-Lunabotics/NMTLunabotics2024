#ifndef CAN_MSG_DEF_H
#define CAN_MSG_DEF_H

#include <stdexcept> // For std::invalid_argument
#include <algorithm> // For std::copy

enum class CAN_ID {
    ESTOP=0x000,
    MOTOR=0x100
};


// Courtesy of GPT, maybe we can make a base class that does most of this
// and child classes for each type of message?
class MotorMsg {
public:
    uint8_t left_speed;
    uint8_t right_speed;
    uint8_t buf_len;
    uint8_t* buf;

    // Constructor that takes a byte array and its length
    MotorMsg(const uint8_t* input_buf, size_t input_len) {
        if (input_len < 2 || input_len > 8) {
            throw std::invalid_argument("Array length must be between 2 and 8");
        }

        left_speed = input_buf[0];
        right_speed = input_buf[1];
        buf_len = static_cast<uint8_t>(input_len);
        buf = new uint8_t[buf_len];
        std::copy(input_buf, input_buf + buf_len, buf);
    }

    // Constructor that takes left and right speeds
    MotorMsg(uint8_t left, uint8_t right) : left_speed(left), right_speed(right), buf_len(8) {
        buf = new uint8_t[buf_len];
        buf[0] = left_speed;
        buf[1] = right_speed;
        // Initialize the rest of the buffer to zero
        std::fill(buf + 2, buf + buf_len, 0);
    }

    // Destructor to deallocate the buffer
    ~MotorMsg() {
        delete[] buf;
    }

    // Copy constructor
    MotorMsg(const MotorMsg& other) : left_speed(other.left_speed), right_speed(other.right_speed), buf_len(other.buf_len) {
        buf = new uint8_t[buf_len];
        std::copy(other.buf, other.buf + buf_len, buf);
    }

    // Copy assignment operator
    MotorMsg& operator=(const MotorMsg& other) {
        if (this != &other) {
            delete[] buf;
            left_speed = other.left_speed;
            right_speed = other.right_speed;
            buf_len = other.buf_len;
            buf = new uint8_t[buf_len];
            std::copy(other.buf, other.buf + buf_len, buf);
        }
        return *this;
    }
};

#endif
