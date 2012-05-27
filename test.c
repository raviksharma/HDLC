#define TEST_C
/****************************************************************************/
/*                              MODULES USED                                */
/****************************************************************************/

#include	<stdio.h>
#include	<stdlib.h>

#include	"HDLC.h"

/****************************************************************************/
/*                          DEFINITIONS AND MACROS                          */
/****************************************************************************/

#define BIT_REV_8(x)\
do{\
	x = ((x) & 0x0F) << 4 | ((x) & 0xF0) >> 4;\
	x = ((x) & 0x33) << 2 | ((x) & 0xCC) >> 2;\
	x = ((x) & 0x55) << 1 | ((x) & 0xAA) >> 1;\
}while(0)

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

static void run_test(FILE *fin);
static void process_frame(unsigned char *fr, unsigned short len, int offset);

/****************************************************************************/
/*                             EXPORTED VARIABLES                           */
/****************************************************************************/

/****************************************************************************/
/*                              GLOBAL VARIABLES                            */
/****************************************************************************/

/* HDLC context */
hdlc_ch_ctxt_t	hdlc_ch_ctxt;

/****************************************************************************/
/*                             EXPORTED FUNCTIONS                           */
/****************************************************************************/

/****************************************************************************/
/*                              LOCAL FUNCTIONS                             */
/****************************************************************************/

int main(int argc, char *argv[])
{
	FILE *fin;	/* Link Stream */

	if(argc != 2){
		printf("Usage: %s <input binary file>\n", argv[0]);
		exit(1);
	}

	printf("FILE: %s\n",argv[1]);

	fin = fopen(argv[1], "rb");
	if(NULL == fin)
	{
		printf("Can't open %s\n",argv[1]);
		exit(1);
	}		

	run_test(fin);

	/* Close Stream */
	fclose(fin);

	return 0;
}

void run_test(FILE *fin)
{
	int				i = 0;
	unsigned char	inp8 = 0;
	
	/* initialize hdlc context and set callback function */
	HDLC_init(&hdlc_ch_ctxt, &process_frame);

	while(!feof(fin))
	{
		fread(&inp8,1,1,fin);
		HDLC(inp8, i, &hdlc_ch_ctxt);
		i++;	/* Offset is byte pointer */
	}
}

void process_frame(unsigned char *fr, unsigned short len, int offset)
{
	int k=0;

	printf("\n%d\tHDLC FRAME: ", offset);
	for(k=0; k<len; ++k)	/* bit reversal is application specific, often req. */
	{
		unsigned char rev = fr[k];
		BIT_REV_8(rev);
		printf("%02X", rev);
	}
}

/****************************************************************************/
/*                                   EOF                                    */
/****************************************************************************/








