/****************************************************************************
 ************                                                    ************
 ************                   Z72_SIMP                         ************
 ************                                                    ************
 ****************************************************************************/
/*!
 *         \file z72_simp.c
 *       \author cs
 *
 *       \brief  Simple example program for the Z72 driver
 *
 *               reads ROM, memory and status depending on the --function-- arg
 *
 *     Required: libraries: mdis_api
 *     \switches (none)
 *
 *
 *---------------------------------------------------------------------------
 * Copyright 2010-2019, MEN Mikro Elektronik GmbH
 ****************************************************************************/

 /*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


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

static const char IdentString[]=MENT_XSTR(MAK_REVISION);

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


/********************************* usage ************************************
 *
 *  Description: Print program usage
 *
 *---------------------------------------------------------------------------
 *  Input......: -
 *  Output.....: -
 *  Globals....: -
 ****************************************************************************/
static void usage(void)
{
 
		printf( "Syntax: z72_simp <device> [options]\n"
				"Function: Z72 example reading (and writing) DS2502 or DS2431 ROM and memory region\n"
				"    device       device name\n\n"
				"Options:\n"
				"   -f=<func>     function to perform:\n"
				"                 0: simple open/close\n"
				"                 1: read ROM section\n"
				"                 2: read ROM and memory section\n"
				"                 3: read memory section, auto skipRom\n"
				"                 4: read memory & generate CRC (DS2502 only)\n"
				"                 5: read status section (DS2502 only)\n"
				"                 6: test auto skipRom function (obsolete)\n"
				"                 7: write memory section (random data, DS2431 only)\n"
				"   [-p]          use polling mode (irq mode is default)\n"
				"   [-o=<offset>] offset in memory for read/write\n"
				"   [-s=<size>]   size of memory to read/write\n");

    printf("\n");
    printf("Copyright 2010-2021, by MEN Mikro Elektronik GmbH\n");
}
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
	u_int8  romData[Z72_OWB_EE_ROM_LEN],
			memData[Z72_OWB_DS2502_MEM_SIZE+2];
	u_int32 function = 0, mode=0;
	u_int16 offset = Z72_SIMP_DS2502_MEMSTART;
	u_int16 size   = Z72_OWB_READ_LEN_MAX;
	int32 i;
	u_int32 error = 0;
    char    *str, *errstr;
    char    buf[80];

	if (argc < 3 || strcmp(argv[1],"-?")==0) {
		usage();
		return(1);
	}

    /*--------------------+
    |  check arguments    |
    +--------------------*/
    if ((errstr = UTL_ILLIOPT("f=o=s=p?", buf))) {   /* check args */
        printf("*** %s\n", errstr);
        return(1);
    }

    if (UTL_TSTOPT("?")) {                      /* help requested ? */
        usage();
        return(1);
    }

	device    = argv[1];
    function  = ((str = UTL_TSTOPT("f=")) ? atoi(str) : 0);
    mode      = !!UTL_TSTOPT("p");

    if( (str = UTL_TSTOPT("o=")) )
        offset = strtol(str, NULL, 16);
    if( (str = UTL_TSTOPT("s=")) )
        size = strtol(str, NULL, 16);

	if( (offset+size) > Z72_OWB_READ_LEN_MAX ) {
		printf("offset and size in memory too large (0x%04x/0x%04x). Sum may not be larger than 0x%08x\n", offset, size, Z72_OWB_READ_LEN_MAX);
		return(1);
	}

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
	    msgBlk.size = Z72_OWB_EE_ROM_LEN;
	    if( (error = M_getstat(path, Z72_BLK_ROM_READ, (int32 *)&msgBlk)) ) {
	    	PrintError("read ROM");
	    	goto abort;
	    }
		printf( "\nZ72_SIMP: Family: 0x%02x\n"
				"          Serial: ", romData[0]);
		for(i=1; i < msgBlk.size - 2; i++) {
			printf( "%02X ", *(romData+i));
		}
		printf( "\n             CRC: 0x%02x\n", romData[Z72_OWB_EE_ROM_LEN - 1] );
	}

	/*-----------------------------+
	|    read memory               |
	+-----------------------------*/
	if( function == 2 || function == 3 ) {

		*(u_int16*)memData = offset;

	    msgBlk.data = memData;
	    msgBlk.size = size;
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

		*(u_int16*)memData = offset;

	    msgBlk.data = memData;
	    msgBlk.size = size;
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
	    /* test obsoleted */
	    printf("Auto skipRom Test: obsoleted\n");
	    goto abort;
    }

	/*-----------------------------------------------------------+
	|    read memory, incr. byte[0] by one and write memory      |
	|    read back for dump                                      |
	+-----------------------------------------------------------*/
	if( function == 7 ) {
		/*-----------------------------+
		|    write memory               |
		+-----------------------------*/
			*(u_int16*)memData = offset;

			msgBlk.data = memData;
			msgBlk.size = size; 
			if( (error = M_getstat(path, Z72_BLK_MEM, (int32 *)&msgBlk)) ) {
				PrintError("read memory");
				goto abort;
			}
			/* shift all data by 2 bytes, data[0..1] will be filled with size */
			for(i=msgBlk.size-1; i >= 0; i--) {
				memData[i+2] = memData[i];
			}
			/* set length of data to be written */
			memData[2] = memData[2]+1;      /* modify one byte so we know we have written something */
			*(u_int16*)memData = offset;
			msgBlk.size = size+2; 
			if( (error = M_setstat(path, Z72_BLK_MEM, (INT32_OR_64)&msgBlk)) ) {
				PrintError("write memory");
				goto abort;
			}
			*(u_int16*)memData = offset;
			msgBlk.size = size; 
			if( (error = M_getstat(path, Z72_BLK_MEM, (int32 *)&msgBlk)) ) {
				PrintError("read memory");
				goto abort;
			}
	}

	if( function == 2 || function == 3 || function == 4 ||
		function == 5 || function == 6 || function == 7 )
	{
		char addrstr[20];

		for(i=0; i < msgBlk.size; i++) {
			sprintf(addrstr, "\n 0x%08lx: ", i+Z72_SIMP_DS2502_MEMSTART);
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

