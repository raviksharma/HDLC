#ifndef HDLC_H
#define HDLC_H

/*
* HDLC Frame Detection Refer ITU-T Rec Q.921 $2
*/

/****************************************************************************/
/*                              MODULES USED                                */
/****************************************************************************/

/****************************************************************************/
/*                          DEFINITIONS AND MACROS                          */
/****************************************************************************/

#define TRUE			1
#define FALSE			0

#define MIN_HDLC_FR_LEN				5						/* Minimum HDLC frame length Allowed */
#define MAX_HDLC_FR_LEN				900						/* Maximum HDLC frame length Allowed */
#define MAX_HDLC_FR_LEN_WITH_STUF	(MAX_HDLC_FR_LEN*(4/3))	/* HDLC frame length + Stuffing bits */

/****************************************************************************/
/*                          TYPEDEFS AND STRUCTURES                         */
/****************************************************************************/

/* HDLC FSM States */
typedef enum hdlc_fr_detect_states
{
	FLAG_SEARCH = 6543,			/* looking for flag state */
	FLAG_SYNC_EST,				/* flag sync established state */
	FRAME_RX,					/* receiving frame state */
	MAX_HDLC_FR_DETECT_STATE
}hdlc_fr_detect_states_e;

/* HDLC Error Codes */
typedef enum hdlc_fr_detect_errors
{
	NO_ERR = 55555,
	INVALID_FR_LEN,
	CRC_MISMATCH,
	MAX_HDLC_FR_DETECT_ERRORS
}hdlc_fr_detect_errors_e;

/* context for hdlc link */
typedef struct hdlc_ch_ctxt
{
	hdlc_fr_detect_states_e	state;									/* HDLC frame detection state */
	hdlc_fr_detect_errors_e	err_type;								/* error type received */		
	unsigned short			fr_bit_cnt;								/* number of bits received between opening and closing flags (working copy)*/
	unsigned short			ready_fr_bit_cnt;						/* number of bits received between opening and closing flags (copy for caller)*/
	unsigned char			flag_pos_ctr;							/* counter for flag matching index in flag_lookup table */
	unsigned char			rec_bits;								/* stores last 8 bits received */
	unsigned char			frame_ready;							/* flag to signal a ready frame */
	unsigned char			frame[MAX_HDLC_FR_LEN_WITH_STUF];		/* stores frame received (working copy)*/
	unsigned char			ready_fr[MAX_HDLC_FR_LEN_WITH_STUF];	/* stores frame received (copy for caller)*/
	void					(*callback)(unsigned char *fr, unsigned short len, int offset);
}hdlc_ch_ctxt_t;

/****************************************************************************/
/*                             EXPORTED VARIABLES                           */
/****************************************************************************/

#ifndef HDLC_C
#endif //HDLC_C
/****************************************************************************/
/*                             EXPORTED FUNCTIONS                           */
/****************************************************************************/

/*
* HDLC frame detection algorithm.
*
* inp8			- Byte from HDLC input Stream.
* offset		- Stream Offset (Application Specific)
* hdlc_ch_ctxt	- HDLC Link Context
*/
void HDLC(unsigned char inp8, int offset, hdlc_ch_ctxt_t *hdlc_ch_ctxt);

/*
* Intializes HDLC frame detection algorithm context.
*
* hdlc_ch_ctxt	- HDLC Link Context
* cb			- Application Callback on frame detection
*	fr		- HDLC frame detected on link
*	len		- HDLC frame length
*	offset	- Stream Offset
*/
void HDLC_init(hdlc_ch_ctxt_t *hdlc_ch_ctxt, void (*cb)(unsigned char *fr, unsigned short len, int offset));

#endif //HDLC_H
/****************************************************************************/
/*                                   EOF                                    */
/****************************************************************************/


