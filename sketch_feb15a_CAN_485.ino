#include <SPI.h>
#include <mcp2515.h>

#include <SoftwareSerial.h>
#include <MAX485.h>

#include "EEPROM.h"

bool interrupt = false;       // interrupt readMessage
struct can_frame canMsg_send; //CAN DATA
struct can_frame canMsg_received;
MCP2515 mcp2515(10); //CAN ENABLE
uint8_t num;         //NUMBER Of received data
uint8_t id[4];       //32bit CANBUS ID

// Setup SoftSerial Port to receive TTL data from receiver
SoftwareSerial FirstSerial(4, 5); // RX(10)->RO, TX(11)->DI
// Setup max485 Control Input
MAX485 FirstMax485(8, 9); // 8->RE, 9->DE

void irqHandler()
{ // interrupt readMessage
  interrupt = true;
  Serial.print("interrupt:");
  Serial.println(interrupt);
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial USB-port to connect
  }
  SPI.begin();

  attachInterrupt(0, irqHandler, FALLING); // interrupt readMessage

  //CAN BUS INIT
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");

  // set the data rate for the SoftwareSerial port connected to the TTL to RS-485 module
  FirstSerial.begin(9600);
  // set Slave-Mode (not really necessary since this is the default library configuration)
  FirstMax485.setSlave();

  Serial.println("EIA-485 slave ready. Receiving...");
}

void loop()
{
  // CAN RECEIVED,485 SEND:
  if (interrupt)
  {
    interrupt = false;

    uint8_t irq = mcp2515.getInterrupts();
    Serial.print("irq: ");
    Serial.println(irq);
    delay(20);

    if (irq & MCP2515::CANINTF_RX0IF)
    {
      Serial.println("In RX0IF ");
      if (mcp2515.readMessage(MCP2515::RXB0, &canMsg_received) == MCP2515::ERROR_OK)
      {
        //485 SET
        FirstMax485.setMaster();

        FirstMax485.sending(true);
        // send to SoftSerial
        FirstSerial.print("Master speaking, message number: ");

        // frame contains received from RXB0 message
        Serial.print(canMsg_received.can_id, HEX); // print ID
        Serial.print(" ");
        Serial.print(canMsg_received.can_dlc, HEX); // print DLC
        Serial.print(" ");

        for (int i = 0; i < canMsg_received.can_dlc; i++)
        { // print the data
          Serial.print(canMsg_received.data[i], HEX);
          Serial.print(" ");
          FirstSerial.print(canMsg_received.data[i], HEX);
        }

        // lower carrier
        FirstMax485.sending(false);
        FirstMax485.setSlave();
      }
      Serial.println();
      delay(100);
    }

    // Serial.println("irq & MCP2515::CANINTF_RX1IF: " + (irq & MCP2515::CANINTF_RX1IF));

    if (irq & MCP2515::CANINTF_RX1IF)
    {
      Serial.println("IN RX1IF");
      if (mcp2515.readMessage(MCP2515::RXB1, &canMsg_received) == MCP2515::ERROR_OK)
      {
        // frame contains received from RXB1 message
        //485 SET
        FirstMax485.setMaster();
        Serial.print("FirstMax485.isSlave: ");
        Serial.println(FirstMax485.isSlave());

        FirstMax485.sending(true);
        // send to SoftSerial
        FirstSerial.print("Master speaking, message number: ");

        // frame contains received from RXB0 message
        Serial.print(canMsg_received.can_id, HEX); // print ID
        Serial.print(" ");
        Serial.print(canMsg_received.can_dlc, HEX); // print DLC
        Serial.print(" ");

        for (int i = 0; i < canMsg_received.can_dlc; i++)
        { // print the data
          Serial.print(canMsg_received.data[i], HEX);
          Serial.print(" ");
          FirstSerial.print(canMsg_received.data[i], HEX);
        }

        // lower carrier
        FirstMax485.sending(false);
        FirstMax485.setSlave();
        Serial.println();
        Serial.print("FirstMax485.isSlave: ");
        Serial.println(FirstMax485.isSlave());
        delay(100);
      }
    }
  }

  //CAN SEND,485 RECEIVED:

  FirstSerial.listen();
  if (FirstSerial.available() > 0)
  {
    Serial.println("FirstSerial.available");
    Serial.print("FirstMax485.isSlave: ");
    Serial.println(FirstMax485.isSlave());
    // number Of Received Data
    num = FirstSerial.available();
    uint8_t i = 0;

    //SET CanBUS ID and Num
    canMsg_send.can_id = 0x0F6;
    if (EEPROM.read(4) == 0x0F)
    {
      for (uint8_t i = 0; i < 4; i++)
      {
        id[i] = EEPROM.read(i);
      }
      canMsg_send.can_id = (id[3] << 0x18 + id[2] << 0x10 + id[1] << 0x08 + id[0]);
    }

    canMsg_send.can_dlc = num;

    while (FirstSerial.available() > 0)
    {
      //HERE TO WRITE 485 TO CAN
      char inByte = FirstSerial.read();
      Serial.write(inByte);

      canMsg_send.data[i] = inByte; // CAN DATA
      i++;
    }
    //CAN sendMessage
    mcp2515.sendMessage(&canMsg_send);

    Serial.println("Messages sent");

    delay(100);
  }

  if (Serial.available() > 0)
  {
    uint8_t val[6];
    uint8_t i = 0;
    while (Serial.available() > 0)
    {
      val[i] = Serial.read();
      i++;
    }
    if (val[0] == 0x0F && val[1] == 0x0F)
    {
      uint8_t k = 2;
      for (int j = 3; j >= 0; j--)
      {
        EEPROM.write(j, val[k]);
        Serial.println(EEPROM.read(j));
        k++;
      }
      EEPROM.write(4, 0x0F);
    }
    delay(100);
  }
  delay(50);
}
