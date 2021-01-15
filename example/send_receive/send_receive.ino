#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>
#include <iso-tp.h>

#define MCP_CS 5 // GPIO5 = VSPI CS0
#define MCP_INT 16 // GPIO16

MCP_CAN CAN0(MCP_CS);
IsoTp isotp(&CAN0, MCP_INT);

struct Message_t txMsg, rxMsg;
uint8_t sf_test[] = { 0x00, 0x01 };
uint8_t mf_test[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, \
                      0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F };
INT32U tx_can_id = 0x7E0;
INT32U rx_can_id = 0x7E8;

void setup()
{
  // serial
  Serial.begin(115200);
  // interrupt
  pinMode(MCP_INT, INPUT);
  // CAN
  CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
  CAN0.setMode(MCP_NORMAL);
  // buffers
  txMsg.Buffer = (uint8_t *)calloc(MAX_MSGBUF, sizeof(uint8_t));
  rxMsg.Buffer = (uint8_t *)calloc(MAX_MSGBUF, sizeof(uint8_t));
}

void loop()
{
  // send
  txMsg.len = sizeof(sf_test);
  txMsg.tx_id = tx_can_id;
  txMsg.rx_id = rx_can_id;
  memcpy(txMsg.Buffer,sf_test,sizeof(sf_test));
  Serial.println(F("Send..."));
  isotp.send(&txMsg);
  // receive
  rxMsg.tx_id = tx_can_id;
  rxMsg.rx_id = rx_can_id;
  Serial.println(F("Receive..."));
  isotp.receive(&rxMsg);
  isotp.print_buffer(rxMsg.rx_id, rxMsg.Buffer, rxMsg.len);
}
