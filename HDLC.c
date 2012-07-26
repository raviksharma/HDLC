#define HDLC_C
/****************************************************************************/
/*                              MODULES USED                                */
/****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "HDLC.h"

/****************************************************************************/
/*                          DEFINITIONS AND MACROS                          */
/****************************************************************************/

#define MAX_HDLC_FR_BITS		(MAX_HDLC_FR_LEN_WITH_STUF*8)
#define MIN_HDLC_FR_BITS		(MIN_HDLC_FR_LEN*8)

#define FLAG				(0x7E)		/* HDLC frame start/end flag */
#define HDLC_TERM_FLAG			(0x7F)		/* HDLC terminate flag */

/****************************************************************************/
/*                          TYPEDEFS AND STRUCTURES                         */
/****************************************************************************/

/****************************************************************************/
/*                            EXTERNAL VARIABLES                            */
/****************************************************************************/

/****************************************************************************/
/*                            EXTERNAL FUNCTIONS                            */
/****************************************************************************/

/****************************************************************************/
/*                       PROTOTYPES AND LOCAL FUNCTIONS                     */
/****************************************************************************/

void 	clear_hdlc_ctxt(hdlc_ch_ctxt_t *hdlc_ch_ctxt);
void 	detect_hdlc_frame(hdlc_ch_ctxt_t *hdlc_ch_ctxt, unsigned char rec_byte, int offset);
void 	remove_stuffing(unsigned char dest_fr[], unsigned char src_frame[], unsigned short bit_cnt, unsigned short *byte_cnt, hdlc_fr_detect_errors_e *err, int offset);

int	hdlc_CRC_match(unsigned char hdlc_frame[MAX_HDLC_FR_LEN],int frame_size);
void 	compute_crc16(unsigned char data[MAX_HDLC_FR_LEN], int size, unsigned char crc[2]);

void 	handle_hdlc_frame(hdlc_ch_ctxt_t *ctxt, int offset);

/****************************************************************************/
/*                             EXPORTED VARIABLES                           */
/****************************************************************************/

/****************************************************************************/
/*                              GLOBAL VARIABLES                            */
/****************************************************************************/

const unsigned char bitmask[] = {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE};

/* FLAG cyclic rotation */
const unsigned char flag_lookup[] = {0xFC, 0xF9, 0xF3, 0xE7, 0xCF, 0x9F, 0x3F, 0x7E};

/* CRC lookup table */
const unsigned short CrcTable[256]=
{
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,	0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
	0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 
	0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
	0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
	0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
	0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
	0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
	0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
	0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};	

/****************************************************************************/
/*                             EXPORTED FUNCTIONS                           */
/****************************************************************************/

void HDLC_init(hdlc_ch_ctxt_t *hdlc_ch_ctxt, void (*cb)(unsigned char *fr, unsigned short len, int offset))
{
	clear_hdlc_ctxt(hdlc_ch_ctxt);

	if(cb != NULL)
	{
		hdlc_ch_ctxt->callback = cb;
	}
	else
	{
		printf("\nHDLC FRAME: ERROR HDLC context callback is not initialized");
		exit(1);
	}
}

void HDLC(unsigned char inp8, int offset, hdlc_ch_ctxt_t *hdlc_ch_ctxt)
{
	detect_hdlc_frame(hdlc_ch_ctxt, inp8, offset);
	if(TRUE==hdlc_ch_ctxt->frame_ready)
		handle_hdlc_frame(hdlc_ch_ctxt, offset);
}

/****************************************************************************/
/*                              LOCAL FUNCTIONS                             */
/****************************************************************************/

/* 
* Removes stuffing, handles frame length errors, Checks CRC 
*/
void handle_hdlc_frame(hdlc_ch_ctxt_t *ctxt, int offset)
{
	unsigned char	fr_minus_stuffing[MAX_HDLC_FR_LEN_WITH_STUF]={0};
	unsigned short	valid_bytes = 0;

	ctxt->frame_ready = FALSE;

	remove_stuffing(fr_minus_stuffing, ctxt->ready_fr, ctxt->ready_fr_bit_cnt, &valid_bytes, &ctxt->err_type, offset);
	memset(&ctxt->ready_fr, 0, MAX_HDLC_FR_LEN_WITH_STUF);

	if(NO_ERR == ctxt->err_type)
	{
		if (hdlc_CRC_match(fr_minus_stuffing, valid_bytes))
		{
			if(ctxt->callback != NULL)
				ctxt->callback(fr_minus_stuffing, valid_bytes, offset); /* Call Application */
		}
		else
		{	
			ctxt->err_type = CRC_MISMATCH;
			printf("\n%d\tHDLC FRAME: ERROR %d", offset, ctxt->err_type);
			ctxt->err_type = NO_ERR;	/* Handle Error & Reset */
		}
	}
	else
	{
		printf("\n%d\tHDLC FRAME: ERROR %d", offset, ctxt->err_type);
		ctxt->err_type = NO_ERR;	/* Handle Error & Reset */
	}
}

/*
* clears sinagling context.
*/
void clear_hdlc_ctxt(hdlc_ch_ctxt_t *hdlc_ch_ctxt)
{
	hdlc_ch_ctxt->flag_pos_ctr = 0;
	hdlc_ch_ctxt->rec_bits=0;
	hdlc_ch_ctxt->fr_bit_cnt=0;
	hdlc_ch_ctxt->ready_fr_bit_cnt=0;
	hdlc_ch_ctxt->frame_ready = FALSE;
	hdlc_ch_ctxt->state=FLAG_SEARCH;
	hdlc_ch_ctxt->err_type=NO_ERR;
	memset(&hdlc_ch_ctxt->frame, 0, MAX_HDLC_FR_LEN_WITH_STUF);
	memset(&hdlc_ch_ctxt->ready_fr, 0, MAX_HDLC_FR_LEN_WITH_STUF);
	hdlc_ch_ctxt->callback = NULL;
}

/*
* detects hdlc frame.
*/
void detect_hdlc_frame(hdlc_ch_ctxt_t *ctxt, unsigned char rec_byte, int offset)
{
	unsigned char	i;
	unsigned short	byte_index;

	for(i=0;i<=7;++i)
	{
		/* get next bit */
		ctxt->rec_bits = (ctxt->rec_bits << 1);
		ctxt->rec_bits = (ctxt->rec_bits | (rec_byte>>(7-i) & 0x01));

		switch(ctxt->state)
		{
		case	FLAG_SEARCH: 
			/* look for 7E */
			if(FLAG == ctxt->rec_bits)
			{
				ctxt->state		= FLAG_SYNC_EST;	/* move to flag sync established state */
				ctxt->flag_pos_ctr	= 0;
			}
			break;

		case	FLAG_SYNC_EST:
			/* look for non 7E rec bits */
			if(flag_lookup[ctxt->flag_pos_ctr] == ctxt->rec_bits)
			{
				ctxt->flag_pos_ctr++;
				if(8 == ctxt->flag_pos_ctr)
				{
					ctxt->flag_pos_ctr = 0; /* reset counter */
				}
			}
			else
			{
				/* move to frame rx state and copy last (ctxt->flag_pos_ctr+1) bits update frame bit counter */
				ctxt->frame[0]		= (ctxt->rec_bits&(~bitmask[(8-(ctxt->flag_pos_ctr+1))]))<<(8-(ctxt->flag_pos_ctr+1));
				ctxt->fr_bit_cnt	= (ctxt->flag_pos_ctr+1);
				ctxt->state		= FRAME_RX;
			}
			break;

		case	FRAME_RX:
			/* keep copying frame bits until closing flag is found */
			byte_index				= (ctxt->fr_bit_cnt/8);
			ctxt->frame[byte_index] = (ctxt->frame[byte_index] | (ctxt->rec_bits&0x01)<<(7-(ctxt->fr_bit_cnt%8)));
			ctxt->fr_bit_cnt++;

			if(MAX_HDLC_FR_BITS < ctxt->fr_bit_cnt) /* error: frame length exceeded */
			{
				memset(&ctxt->frame, 0, MAX_HDLC_FR_LEN_WITH_STUF);
				ctxt->fr_bit_cnt	= 0;
				ctxt->state		= FLAG_SEARCH;
				printf("\n%d\tHDLC FRAME: ERROR frame length exceeded during RX", offset);
				break;
			}

			if(FLAG == ctxt->rec_bits)
			{
				/* closing flag found */
				ctxt->state		= FLAG_SYNC_EST;
				ctxt->flag_pos_ctr	= 0;

				if(MIN_HDLC_FR_BITS > ctxt->fr_bit_cnt) /* error: invalid frame length */
				{
					memset(&ctxt->frame, 0, MAX_HDLC_FR_LEN_WITH_STUF);
					ctxt->fr_bit_cnt	= 0;
					ctxt->state		= FLAG_SEARCH;
					printf("\n%d\tHDLC FRAME: ERROR frame length less than %d bits", offset, MIN_HDLC_FR_BITS*8);
					break;
				}

				/* remove last 8 bits from frame and update frame bit counter */
				ctxt->frame[byte_index]	= 0;
				ctxt->fr_bit_cnt	= (ctxt->fr_bit_cnt-8);
				byte_index		= (ctxt->fr_bit_cnt/8);
				ctxt->frame[byte_index]	= ((ctxt->frame[byte_index])&(bitmask[ctxt->fr_bit_cnt%8]));

				/* copy frame to caller's copy */
				memcpy(&ctxt->ready_fr,&ctxt->frame,MAX_HDLC_FR_LEN_WITH_STUF);
				ctxt->ready_fr_bit_cnt	= ctxt->fr_bit_cnt;
				ctxt->frame_ready 	= TRUE;

				/* reset working copy */
				memset(&ctxt->frame, 0, MAX_HDLC_FR_LEN_WITH_STUF);
				ctxt->fr_bit_cnt	= 0;
			}
			else if(HDLC_TERM_FLAG == ctxt->rec_bits)
			{
				memset(&ctxt->frame, 0, MAX_HDLC_FR_LEN_WITH_STUF);
				ctxt->fr_bit_cnt	= 0;
				ctxt->state		= FLAG_SEARCH;
				break;
			}

			break;

		default:
			/* not possible */
			assert(0);
		}
	}
}

/*
* removes stuffed bits. assumption: any bit after 5 1s is ignored
*/
void remove_stuffing(unsigned char dst_fr[], unsigned char src_fr[], unsigned short bit_cnt, unsigned short *byte_cnt, hdlc_fr_detect_errors_e *err, int offset)
{
	unsigned short	i;
	unsigned short	src_byte;
	unsigned short	dst_byte;
	unsigned char	curr_bit;
	unsigned short	pos=0;	
	unsigned char	ones=0;

	for(i=0;i<bit_cnt;++i,++pos)
	{
		src_byte=i/8;
		curr_bit=(src_fr[src_byte]>>(7-(i%8)))&0x01;
		if(curr_bit)
		{
			dst_byte=pos/8;
			dst_fr[dst_byte]= dst_fr[dst_byte] | curr_bit<<(7-(pos%8));
			++ones;		/* inc. 1s count */
			if(5==ones)
			{
				++i;	/* skip next bit */
				ones=0; /* reset 1s count */
			}
		}
		else
		{
			dst_byte=pos/8;
			dst_fr[dst_byte]= dst_fr[dst_byte] | curr_bit<<(7-(pos%8));
			ones=0;		/* reset 1s count */
		}
	}

	*byte_cnt=pos/8;

	/* look for invalid frame length error */
	if(((pos%8)!=0) || (*byte_cnt<5) || (*byte_cnt>MAX_HDLC_FR_LEN))
	{
		if((pos%8)!=0)
		{
			printf("\n%d\tHDLC FRAME: ERROR (frame_length_bits%%8)!=0", offset);
		}

		*err=INVALID_FR_LEN;
	}
}



/*
* runs CRC check
*/
int hdlc_CRC_match(unsigned char hdlc_frame[MAX_HDLC_FR_LEN], int frame_size)
{
	unsigned char hdlc_crc[2] = {0};

	if ((frame_size<5)||(frame_size>=MAX_HDLC_FR_LEN)) return 0;

	compute_crc16(hdlc_frame, frame_size, hdlc_crc);

	hdlc_crc[0]	=	(~(hdlc_crc[0]));
	hdlc_crc[1]	=	(~(hdlc_crc[1]));

	if ((hdlc_crc[0]==hdlc_frame[frame_size-2])&&(hdlc_crc[1]==hdlc_frame[frame_size-1])) return 1;
	else return 0;
}

/*
* Computes CRC
*/
void compute_crc16(unsigned char data[MAX_HDLC_FR_LEN], int size, unsigned char crc[2])
{
	int		j = 0;
	int		byte_cnt = 0;
	unsigned char	crc_data[MAX_HDLC_FR_LEN] = {0};
	unsigned char	crc_bits[2] = {0};
	unsigned short	fcs = 0xFFFF;
	unsigned char	datafcs = 0;
	int				crc_data_size = 0;

	crc_data_size = size;

	for(j=0; j<crc_data_size; j++)
	{
		crc_data[j]=data[j];
	}

	while(byte_cnt<(crc_data_size-2))
	{
		datafcs = crc_data[byte_cnt] ^ (fcs >> 8);
		fcs  = CrcTable[datafcs] ^ (fcs << 8);
		byte_cnt++;
	}

	crc_bits[0] = ((fcs & 0xFF00)>>8);
	crc_bits[1] = (fcs & 0x00FF);
	memcpy(crc, crc_bits, sizeof (crc_bits));
}

/****************************************************************************/
/*                                   EOF                                    */
/****************************************************************************/


