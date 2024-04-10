#include "main_bus.hpp"
#include <Arduino_CAN.h>
#include "arduino_lib.hpp"

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;
  if (!CAN.begin(CanBitRate::BR_500k))
    panic("Can is not available");

  int msg_cnt = 0;

  while (true) {
    if (CAN.available()) {
      CanMsg const msg = CAN.read();
      Serial.println(msg);
    }

    // can::ActuatorArmPos cmd = {.left_pos = (double)pos_l,
    //                            .right_pos = (double)pos_r};
    // uint8_t buffer[8];
    // can::to_buffer(buffer, can::serialize(cmd));

    // CanMsg const msg((int)can::FrameID::ActuatorArmPos, sizeof(buffer),
    // buffer);

    // if (int const rc = CAN.write(msg); rc < 0) {
    //   Serial.println("CAN.write(...) failed with error code " + String(rc));
    // }
    /* Assemble a CAN message with the format of
     * 0xCA 0xFE 0x00 0x00 [4 byte message counter]
     */
    uint8_t const msg_data[] = {0xCA, 0xFE, 0, 0, 0, 0, 0, 0};
    memcpy((void *)(msg_data + 4), &msg_cnt, sizeof(msg_cnt));
    static uint32_t const CAN_ID = 0x20;
    CanMsg const msg(CanStandardId(CAN_ID), sizeof(msg_data), msg_data);

    /* Transmit the CAN message, capture and display an
     * error core in case of failure.
     */
    if (int const rc = CAN.write(msg); rc < 0) {
      Serial.print("CAN.write(...) failed with error code ");
      Serial.println(rc);
      for (;;) {
      }
    }

    /* Increase the message counter. */
    msg_cnt++;

    /* Only send one message per second. */
    delay(1000);
  }
}

void loop() {}
