// receive a frame from can bus

#include <Arduino.h>

#include "mcp_can.h"
#include <SPI.h>

int main() {
  Serial.begin(115200);
  MCP_CAN CAN(17); // Set CS pin

  while (CAN_OK != CAN.begin(CAN_500KBPS)) // init can bus : baudrate = 500k
  {
    Serial.println("CAN BUS FAIL!");
    delay(100);
  }
  Serial.println("CAN BUS OK!");
  Serial.println("Running new mirror code");

  while (true) {
    unsigned char len = 0;
    unsigned char buf[8];

    static int count = 0;
    count++;
    if (count == 10000) {
      count = 0;
      for (int i = 0; i < 8; i++)
        buf[i] = 0x00;
      CAN.sendMsgBuf(0x00, 0, 8, buf);
      Serial.println("Sent a packet");
    }

    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      CAN.readMsgBuf(&len, buf);

      unsigned long canId = CAN.getCanId();

      Serial.println("-----------------------------");
      Serial.print("Get data from ID: ");
      Serial.println(canId, HEX);

      // print the data
      for (int i = 0; i < len; i++) {
        Serial.print(buf[i], HEX);
        Serial.print("\t");
      }
      Serial.println();

      // Send back the data, mirrored and flipped.
      for (int i = 0; i < len; i++)
        buf[i] = ~buf[i];
      CAN.sendMsgBuf(0x00, 0, 8, buf);
    }
  }
}
