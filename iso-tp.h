#ifndef _ISOTP_H
#define _ISOTP_H

#include <mcp_can.h>

//#define ISO_TP_DEBUG

typedef enum {
  ISOTP_IDLE = 0,
  ISOTP_SEND,
  ISOTP_SEND_FF,
  ISOTP_SEND_CF,
  ISOTP_WAIT_FIRST_FC,
  ISOTP_WAIT_FC,
  ISOTP_WAIT_DATA,
  ISOTP_FINISHED,
  ISOTP_ERROR
} isotp_states_t;

#define CAN_MAX_DLEN 8  //Not extended CAN

/* N_PCI type values in bits 7-4 of N_PCI bytes */
#define N_PCI_SF  0x00  /* single frame */
#define N_PCI_FF  0x10  /* first frame */
#define N_PCI_CF  0x20  /* consecutive frame */
#define N_PCI_FC  0x30  /* flow control */

#define FC_CONTENT_SZ 3 /* flow control content size in byte (FS/BS/STmin) */

/* Flow Status given in FC frame */
#define ISOTP_FC_CTS  0   /* clear to send */
#define ISOTP_FC_WT 1     /* wait */
#define ISOTP_FC_OVFLW  2 /* overflow */

/* Timeout values */
#define TIMEOUT_SESSION  500 /* Timeout between successfull send and receive */
#define TIMEOUT_FC       250 /* Timeout between FF and FC or Block CF and FC */
#define TIMEOUT_CF       250 /* Timeout between CFs                          */
#define MAX_FCWAIT_FRAME  10   

#define MAX_MSGBUF 128    /* Received Message Buffer. Depends on uC ressources!
                             Should be enough for our needs */
struct Message_t
{
  uint8_t len=0;
  isotp_states_t tp_state=ISOTP_IDLE;
  uint8_t seq_id=1;
  uint8_t fc_status=ISOTP_FC_CTS;
  uint8_t blocksize=0;
  uint8_t min_sep_time=0;
  uint32_t tx_id=0;
  uint32_t rx_id=0;
  uint8_t *Buffer;
};

class IsoTp
{
	public:
		IsoTp(MCP_CAN* bus, uint8_t mcp_int);
		uint8_t send(Message_t* msg);
		uint8_t receive(Message_t* msg);
		void    print_buffer(uint32_t id, uint8_t *buffer, uint16_t len);
	private:
		MCP_CAN* _bus;
                uint8_t  _mcp_int;
		uint32_t rxId;
		uint8_t  rxLen;
		uint8_t  rxBuffer[8];
    uint16_t rest;
		uint8_t  fc_wait_frames=0;
		uint32_t wait_fc=0;
		uint32_t wait_cf=0;
    uint32_t wait_session=0;
		uint8_t  can_send(uint16_t id, uint8_t len, uint8_t *data);
		uint8_t  can_receive(void);
		uint8_t  send_fc(struct Message_t* msg);
		uint8_t  send_sf(struct Message_t* msg);
		uint8_t  send_ff(struct Message_t* msg);
		uint8_t  send_cf(struct Message_t* msg);
		uint8_t  rcv_sf(struct Message_t* msg);
		uint8_t  rcv_ff(struct Message_t* msg);
		uint8_t  rcv_cf(struct Message_t* msg);
		uint8_t  rcv_fc(struct Message_t* msg);
    void     fc_delay(uint8_t sep_time);
};
                             
#endif
