#include "Arduino.h"
#include "iso-tp.h"
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>

IsoTp::IsoTp(MCP_CAN* bus)
{
	_bus = bus;
}

void IsoTp::can_print_frame(void)
{
  Serial.print(rxId, HEX);            // Print the message ID
	Serial.print(F("#"));
  for (uint8_t i = 0; i < rxLen; i++) // Print each uint8_tof the data.
	{
	  if (rxBuffer[i] < 0x10)           // If data uint8_tis less than 0x10, 
	  {																	// add a leading zero.
			Serial.print(F("0"));
		  Serial.print(rxBuffer[i], HEX);
			Serial.print(F(" "));
		}
		Serial.println();
  }
}

uint8_t IsoTp::can_send(const uint16_t id, uint8_t len, uint8_t *data)
{
	return _bus->sendMsgBuf(id, 0, len, data);
}

uint8_t IsoTp::can_receive(void)
{
	  if (!digitalRead(2))     // If pin 2 is low, read receive buffer
		{
		  _bus->readMsgBuf(&rxId, &rxLen, rxBuffer); // Read data: 
																								 // len = data length, 
																						     // buf = data byte(s)
		  return true;
		}
		else return false;
}

uint8_t IsoTp::send_fc(struct Message_t *msg)
{
  uint8_t TxBuf[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	// FC message high nibble = 0x3 , low nibble = FC Status
	TxBuf[0]=(N_PCI_FC | msg->fc_status);
	return can_send(msg->tx_id,3,TxBuf);
}

uint8_t IsoTp::send_sf(struct Message_t *msg) //Send SF Message
{
  uint8_t TxBuf[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	// SF message high nibble = 0x0 , low nibble = Length
	TxBuf[0]=(N_PCI_SF | msg->len);
	memcpy(TxBuf+1,msg->Buffer,msg->len);
  return can_send(msg->tx_id,msg->len+1,TxBuf);// Add PCI length
}

uint8_t IsoTp::send_ff(struct Message_t *msg) // Send FF
{
  uint8_t TxBuf[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  msg->seq_id=1;

	TxBuf[0]=(N_PCI_FF | ((msg->len&0x0F00) >> 8));
	TxBuf[1]=(msg->len&0x00FF);
	memcpy(TxBuf+2,msg->Buffer,6);             // Skip 2 Bytes PCI
	return can_send(msg->tx_id,8,TxBuf);       // First Frame has full length
}

uint8_t IsoTp::send_cf(struct Message_t *msg) // Send SF Message
{
  uint8_t TxBuf[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint16_t len=7;
		 
	TxBuf[0]=(N_PCI_CF | (msg->seq_id & 0x0F));
  if(msg->len>7) len=7; else len=msg->len;
		memcpy(TxBuf+1,msg->Buffer,len);         // Skip 1 Byte PCI
  return can_send(msg->tx_id,len+1,TxBuf);   // Last frame is probably shorter
	                                           // than 8 -> Signals last CF Frame
}

void IsoTp::fc_delay(uint8_t sep_time)
{
	if(sep_time < 0x80)
	  delay(sep_time);
	else
	  delayMicroseconds((sep_time-0xF0)*100);
}

uint8_t IsoTp::rcv_fc(struct Message_t *msg)
{
	if (msg->tp_state != ISOTP_WAIT_FC && msg->tp_state != ISOTP_WAIT_FIRST_FC)
	  return 0;

	/* get communication parameters only from the first FC frame */
	if (msg->tp_state == ISOTP_WAIT_FIRST_FC)
	{
		msg->blocksize = rxBuffer[1];
		msg->min_sep_time = rxBuffer[2];

		/* fix wrong STmin values according spec */
		if ((msg->min_sep_time > 0x7F) && ((msg->min_sep_time < 0xF1) 
				|| (msg->min_sep_time > 0xF9))) msg->min_sep_time = 0x7F;
													}
		Serial.print(F("FC frame: FS "));
		Serial.print(rxBuffer[0]&0x0F);
		Serial.print(F(", Blocksize "));
		Serial.print(msg->blocksize);
		Serial.print(F(", Min. separation Time "));
		Serial.println(msg->min_sep_time);

		switch (rxBuffer[0] & 0x0F)
		{
		  case ISOTP_FC_CTS:
												  msg->tp_state = ISOTP_SEND_CF;
												  break;

			case ISOTP_FC_WT:
												  Serial.println(F("Start waiting for next FC"));
												  break;

		  case ISOTP_FC_OVFLW:
												  Serial.println(F("Overflow in receiver side"));

		  default:
											  	/* stop this tx job. TODO: error reporting? */
												  msg->tp_state = ISOTP_IDLE;
		}
		return 0;
}

uint8_t IsoTp::send(Message_t* msg)
{
	msg->tp_state=ISOTP_SEND;

	while(msg->tp_state!=ISOTP_IDLE && msg->tp_state!=ISOTP_ERROR)
	{
    Serial.print(F("ISO-TP State: ")); Serial.println(msg->tp_state);
    Serial.print(F("Length      : ")); Serial.println(msg->len);

	  switch(msg->tp_state)
	  {
		  case ISOTP_IDLE    : break;
		  case ISOTP_SEND    :
			                     if(msg->len<=7)
												   {
 													   Serial.println(F("Send SF"));
 		                         return send_sf(msg);
 												   }
 										       else
										       {
 													   Serial.println(F("Send FF"));
 											       if(!send_ff(msg)) // FF complete
 													   {
 														   msg->Buffer+=6;
														   msg->len-=6;
                               msg->tp_state=ISOTP_WAIT_FIRST_FC;
													   }
 									         }
 										       break;
 	    case ISOTP_WAIT_FIRST_FC:
												   Serial.println(F("Wait first FC"));
											     break;
	    case ISOTP_WAIT_FC :
												   Serial.println(F("Wait FC"));
		                       break;
		  case ISOTP_SEND_CF : 
												   Serial.println(F("Send CF"));
 												   while(msg->len>7) 
 												   {
													   fc_delay(msg->min_sep_time);
													   if(!send_cf(msg))
 													   {
                               Serial.print(F("Send Seq "));
														   Serial.println(msg->seq_id);
                               msg->seq_id++;
														   msg->seq_id %= 16;
														   msg->Buffer+=7;
														   msg->len-=7;
														   Serial.print(F("Length      : "));
														   Serial.println(msg->len);                        
													   }
												   }
												   fc_delay(msg->min_sep_time);
												   Serial.print(F("Send last Seq "));
												   Serial.println(msg->seq_id);
												   send_cf(msg);
												   msg->tp_state=ISOTP_IDLE;
												   break;
		  default            : break;
   }

	 if(can_receive())
	 {
	   Serial.println("CAN RAW Data");
		 can_print_frame();
     rcv_fc(msg);
		 memset(rxBuffer,0,sizeof(rxBuffer));
	 }
 }

 return 0;
}

uint8_t IsoTp::receive(Message_t* msg)
{
	Serial.println(F("Receive..."));

	return 0;
}
