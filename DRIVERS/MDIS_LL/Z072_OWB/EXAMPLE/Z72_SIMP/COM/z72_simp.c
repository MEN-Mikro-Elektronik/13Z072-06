/****************************************************************************
 ************                                                    ************
 ************                   Z72_SIMP                         ************
 ************                                                    ************
 ****************************************************************************/
/*!
 *         \file z72_simp.c
 *       \author cs
 *        $Date: 2010/03/12 14:20:43 $
 *    $Revision: 1.3 $
 *
 *       \brief  Simple example program for the Z72 driver
 *
 *               reads ROM, memory and status depending on the --function-- arg
 *
 *     Required: libraries: mdis_api
 *     \switches (none)
 */
 /*-------------------------------[ History ]--------------------------------
 *
 * $Log: z72_simp.c,v $
 * Revision 1.3  2010/03/12 14:20:43  amorbach
 * R: Porting to MDIS5
 * M: changed according to MDIS Porting Guide 0.8
 *
 * Revision 1.2  2006/06/08 17:54:53  cs
 * fixed: disable interrupts when reading in polled mode
 *
 * Revision 1.1  2006/06/02 16:40:23  cs
 * Initial Revision
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 2010 by MEN Mikro Elektronik GmbH, Nuremberg, Germany
 ****************************************************************************/

static const char RCSid[]="$Id: z72_simp.c,v 1.3 2010/03/12 14:20:43 amorbach Exp $";

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <MEN/men_typs.h>
#include <MEN/usr_oss.h>
#include <MEN/usr_utl.h>
#include <MEN/usr_err.h>
#include <MEN/mdis_err.h>
#include <MEN/mdis_api.h>
#include <MEN/z72_drv.h>

/*--------------------------------------+
|   DEFINES                             |
+--------------------------------------*/
#define Z72_SIMP_DS2502_MEMSTART 0	/**< start to read memory from here */


/*--------------------------------------+
|   TYPDEFS                             |
+--------------------------------------*/
/* none */

/*--------------------------------------+
|   EXTERNALS                           |
+--------------------------------------*/
/* none */

/*--------------------------------------+
|   GLOBALS                             |
+--------------------------------------*/
/* none */

/*--------------------------------------+
|   PROTOTYPES                          |
+--------------------------------------*/
static void PrintError(char *info);


/********************************* main ************************************/
/** Program main function
 *
 *  \param argc       \IN  argument counter
 *  \param argv       \IN  argument vector
 *
 *  \return	          success (0) or error (1)
 */
int main(int argc, char *argv[])
{
	MDIS_PATH	path;
	char	*device;
	M_SG_BLOCK msgBlk;
	u_int8  romData[Z72_OWB_ROM_LEN],
			memData[Z72_OWB_DS2502_MEM_SIZE];
	u_int32 function = 0, mode=0;
	int32 i;
	u_int32 error = 0;

	if (argc < 3 || strcmp(argv[1],"-?")==0) {
		printf( "Syntax: z72_simp <device>\n"
				"Function: Z72 example reading a DS2502 ROM and memory region\n"
				"Options:\n"
				"    device       device name\n"
				"    function     function to perform:\n"
				"                 0: simple open/close\n"
				"                 1: read ROM section\n"
				"                 2: read ROM and memory section\n"
				"                 3: read memory section, auto skipRom\n"
				"                 4: read memory & generate CRC\n"
				"                 5: read status section\n"
				"                 6: test auto skipRom function\n"
				"   [ mode ]      0: irq mode (default)\n"
				"                 1: polled mode\n"

				"\n");
		return(1);
	}

	device    = argv[1];
	function  = atoi(argv[2]);
	if( argc >= 3 )
		mode      = atoi(argv[3]);

	/*--------------------+
    |  open path          |
    +--------------------*/
	if ((path = M_open(device)) < 0) {
		PrintError("open");
		return(1);
	}

	/* enable interrupts ? */
	if( (error = M_setstat(path, M_MK_IRQ_ENABLE, mode ? 0 : 1 )) ) {
    	PrintError("enable interrupts");
    	goto abort;
    }

	/*-----------------------------+
	|    read ROM                  |
	+-----------------------------*/
	if( function == 1 || function == 2 || function == 4 || function == 5 ) {
	    msgBlk.data = romData;
	    msgBlk.size = Z72_OWB_ROM_LEN;
	    if( (error = M_getstat(path, Z72_BLK_ROM_READ, (int32 *)&msgBlk)) ) {
	    	PrintError("read ROM");
	    	goto abort;
	    }
		printf( "\nZ72_SIMP: Family: 0x%02x\n"
				"          Serial: ", romData[0]);
		for(i=1; i < msgBlk.size - 2; i++) {
			printf( "%02X ", *(romData+i));
		}
		printf( "\n             CRC: 0x%02x\n", romData[Z72_OWB_ROM_LEN - 1] );
	}

	/*-----------------------------+
	|    read memory               |
	+-----------------------------*/
	if( function == 2 || function == 3 ) {

		*(u_int16*)memData = Z72_SIMP_DS2502_MEMSTART;

	    msgBlk.data = memData;
	    msgBlk.size = Z72_OWB_DS2502_MEM_SIZE;
	    if( (error = M_getstat(path, Z72_BLK_MEM, (int32 *)&msgBlk)) ) {
	    	PrintError("read memory");
	    	goto abort;
	    }
		printf( "\nZ72_SIMP: Memory:\n" );
	}

	/*---------------------------------+
	|    read memory & generate CRC    |
	+---------------------------------*/
	if( function == 4 ) {

		*(u_int16*)memData = Z72_SIMP_DS2502_MEMSTART;

	    msgBlk.data = memData;
	    msgBlk.size = Z72_OWB_DS2502_MEM_SIZE;
	    if( (error = M_getstat(path, Z72_BLK_MEM_CRC, (int32 *)&msgBlk)) ) {
	    	PrintError("read memory (& generate CRC)");
	    	goto abort;
	    }
		printf( "\nZ72_SIMP: Memory (w. CRC):\n" );

	}

	/*-----------------------------+
	|    read status               |
	+-----------------------------*/
	if( function == 5 ) {

		*(u_int16*)memData = 0; /* start reading status from begin of page */

	    msgBlk.data = memData;
	    msgBlk.size = Z72_OWB_DS2502_ST_SIZE;
	    if( (error = M_getstat(path, Z72_BLK_STATUS, (int32 *)&msgBlk)) ) {
	    	PrintError("read status");
	    	goto abort;
	    }
	}

	/*---------------------------------+
	|    test auto skipRom function    |
	+---------------------------------*/
	if( function == 6 ) {

		*(u_int16*)memData = Z72_SIMP_DS2502_MEMSTART;

	    msgBlk.data = memData;
	    msgBlk.size = Z72_OWB_DS2502_MEM_SIZE;
	    /* disable auto skipRom */
	    printf("Auto skipRom Test:\nDisable auto skiprom, read memory\n");
	    if( (error = M_setstat(path, Z72_ROM_SKIP_AUTOEN, 0)) ) {
	    	PrintError("disable auto skipRom");
	    	goto abort;
	    }
	    printf("Read memory (has to return with error)\n");
	    if( (error = M_getstat(path, Z72_BLK_MEM_CRC, (int32 *)&msgBlk)) ) {
	    	PrintError("read memory, this was expected (auto skipRom was disabled)");
	    }
	    /* enable auto skipRom */
	    printf("Enable auto skiprom, read memory again:\n");
	    if( (error = M_setstat(path, Z72_ROM_SKIP_AUTOEN, 1)) ) {
	    	PrintError("enable auto skipRom");
	    	goto abort;
	    }
	    if( (error = M_getstat(path, Z72_BLK_MEM_CRC, (int32 *)&msgBlk)) ) {
	    	PrintError("read memory");
	    	goto abort;
	    }
	}

	if( function == 2 || function == 3 || function == 4 ||
		function == 5 || function == 6 )
	{
		char addrstr[20];

		for(i=0; i < msgBlk.size; i++) {
			sprintf(addrstr, "\n 0x%08X: ", i+Z72_SIMP_DS2502_MEMSTART);
			printf( "%s%02X ",
					!(i%16) ? addrstr : (!(i%8)&&(i!=0)) ? " " : "",
					*(memData+i));
		}
		printf( "\n\n" );
	}

	/*--------------------+
    |  cleanup            |
    +--------------------*/
abort:
	if (M_close(path) < 0)
		PrintError("close");

	return(0);
}

/********************************* PrintError ******************************/
/** Print MDIS error message
 *
 *  \param info       \IN  info string
*/
static void PrintError(char *info)
{
	printf("*** can't %s: %s\n", info, M_errstring(UOS_ErrnoGet()));
}

