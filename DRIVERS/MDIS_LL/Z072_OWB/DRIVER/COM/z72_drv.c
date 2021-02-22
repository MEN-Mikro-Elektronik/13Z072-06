/*********************  P r o g r a m  -  M o d u l e ***********************/
/*!
 *        \file  z72_drv.c
 *
 *      \author  cs
 *
 *      \brief   Low-level driver for Z72_OWB module
 *
 *     Required: OSS, DESC, DBG, ID libraries
 *
 *     \switches _ONE_NAMESPACE_PER_DRIVER_
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

#include <string.h>
#include "z72_drv_int.h"	/* Z72 driver internal header file */

static const char IdentString[]=MENT_XSTR(MAK_REVISION);

/*-----------------------------------------+
|  DEFINES                                 |
+-----------------------------------------*/
/* include files which need LL_HANDLE */
#include <MEN/ll_entry.h>   /* low-level driver jump table  */
#include <MEN/z72_drv.h>   /* Z72 driver header file */
/*-----------------------------------------+
|  PROTOTYPES                              |
+-----------------------------------------*/
static int32 Z72_Init(DESC_SPEC *descSpec, OSS_HANDLE *osHdl,
					   MACCESS *ma, OSS_SEM_HANDLE *devSemHdl,
					   OSS_IRQ_HANDLE *irqHdl, LL_HANDLE **llHdlP);
static int32 Z72_Exit(LL_HANDLE **llHdlP );
static int32 Z72_Read(LL_HANDLE *llHdl, int32 ch, int32 *value);
static int32 Z72_Write(LL_HANDLE *llHdl, int32 ch, int32 value);
static int32 Z72_SetStat(LL_HANDLE *llHdl,int32 ch, int32 code, INT32_OR_64 value32_or_64);
static int32 Z72_GetStat(LL_HANDLE *llHdl, int32 ch, int32 code, INT32_OR_64 *value32_or_64P);
static int32 Z72_BlockRead(LL_HANDLE *llHdl, int32 ch, void *buf, int32 size,
							int32 *nbrRdBytesP);
static int32 Z72_BlockWrite(LL_HANDLE *llHdl, int32 ch, void *buf, int32 size,
							 int32 *nbrWrBytesP);
static int32 Z72_Irq(LL_HANDLE *llHdl );
static int32 Z72_Info(int32 infoType, ... );

static char* Ident( void );
static int32 Cleanup(LL_HANDLE *llHdl, int32 retCode);

/* generic EEPROM access functions */
static int32 romRead( LL_HANDLE *llHdl, u_int8 *buf, u_int32 numBytes );
static int32 romSkip( LL_HANDLE *llHdl );
static int32 memRead( LL_HANDLE *llHdl, u_int8 *buf, u_int16 offs, u_int16 numBytes );

/* DS2502 specific EEPROM access functions */
static int32 ds2502_statusRead( LL_HANDLE *llHdl, u_int8 *buf, u_int16 offs, u_int16 numBytes );
static int32 ds2502_memReadCrc( LL_HANDLE *llHdl, u_int8 *buf, u_int16 offs, u_int16 numBytes );

/* DS2431 specific EEPROM functions */
static int32 ds2431_memWrite( LL_HANDLE *llHdl, u_int8 *buf, u_int16 offs, u_int16 numBytes );

/* initialize Funktion pointers for known devices */
static int32 initEeHandle( LL_HANDLE *llHdl );
static int32 readMemory( LL_HANDLE *llHdl,
						 u_int32 majState,
						 u_int8 *buf,
						 u_int16 offs,
						 u_int16*size );
static int32 masterTxReset( LL_HANDLE *llHdl );
static int32 waitOnReady( LL_HANDLE *llHdl );
static int32 getDevError( LL_HANDLE *llHdl );
static int32 execCmd( LL_HANDLE *llHdl, u_int8 *data, u_int32 cmd );
static void byteCrc( u_int8 *crc, u_int8 c );
static void byteCrcFinish( u_int8 *crc );
static u_int8 bufCrc8( u_int8 crcStart, u_int8 *p, u_int32 len);


/****************************** Z72_GetEntry ********************************/
/** Initialize driver's jump table
 *
 *  \param drvP     \OUT Pointer to the initialized jump table structure
 */
extern void Z72_GetEntry( LL_ENTRY* drvP )
{
	drvP->init        = Z72_Init;
	drvP->exit        = Z72_Exit;
	drvP->read        = Z72_Read;
	drvP->write       = Z72_Write;
	drvP->blockRead   = Z72_BlockRead;
	drvP->blockWrite  = Z72_BlockWrite;
	drvP->setStat     = Z72_SetStat;
	drvP->getStat     = Z72_GetStat;
	drvP->irq         = Z72_Irq;
	drvP->info        = Z72_Info;
}

/******************************** Z72_Init **********************************/
/** Allocate and return low-level handle, initialize hardware
 *
 * The function initializes all channels with the definitions made
 * in the descriptor. The interrupt is disabled.
 *
 * The function decodes \ref descriptor_entries "these descriptor entries"
 * in addition to the general descriptor keys.
 *
 *  \param descP      \IN  Pointer to descriptor data
 *  \param osHdl      \IN  OSS handle
 *  \param ma         \IN  HW access handle
 *  \param devSemHdl  \IN  Device semaphore handle
 *  \param irqHdl     \IN  IRQ handle
 *  \param llHdlP     \OUT Pointer to low-level driver handle
 *
 *  \return           \c 0 On success or error code
 */
static int32 Z72_Init(
	DESC_SPEC       *descP,
	OSS_HANDLE      *osHdl,
	MACCESS         *ma,
	OSS_SEM_HANDLE  *devSemHdl,
	OSS_IRQ_HANDLE  *irqHdl,
	LL_HANDLE       **llHdlP
)
{
	LL_HANDLE *llHdl = NULL;
	u_int32 gotsize;
	int32 error;
	u_int32 value;
	u_int8 romBuf[Z72_OWB_EE_ROM_LEN];

	/*------------------------------+
	|  prepare the handle           |
	+------------------------------*/
	*llHdlP = NULL;		/* set low-level driver handle to NULL */

	/* alloc */
	if ((llHdl = (LL_HANDLE*)OSS_MemGet(
					osHdl, sizeof(LL_HANDLE), &gotsize)) == NULL)
	   return(ERR_OSS_MEM_ALLOC);

	/* clear */
	OSS_MemFill(osHdl, gotsize, (char*)llHdl, 0x00);

	/* init */
	llHdl->memAlloc   = gotsize;
	llHdl->osHdl      = osHdl;
	llHdl->irqHdl     = irqHdl;
	llHdl->ma		  = *ma;
	llHdl->devSemHdl  = devSemHdl;

	/*------------------------------+
	|  init id function table       |
	+------------------------------*/
	/* driver's ident function */
	llHdl->idFuncTbl.idCall[0].identCall = Ident;
	/* library's ident functions */
	llHdl->idFuncTbl.idCall[1].identCall = DESC_Ident;
	llHdl->idFuncTbl.idCall[2].identCall = OSS_Ident;
	/* terminator */
	llHdl->idFuncTbl.idCall[3].identCall = NULL;

	/*------------------------------+
	|  prepare debugging            |
	+------------------------------*/
	llHdl->dbgLevel = OSS_DBG_DEFAULT;	/* set OS specific debug level */
	DBGINIT((NULL,&DBH));

	/*------------------------------+
	|  prepare semaphores           |
	+------------------------------*/
	if( (error = OSS_SemCreate( llHdl->osHdl, OSS_SEM_BIN,
								0, &llHdl->exeSem )))
		return( Cleanup(llHdl,error) );
	DBGWRT_2((DBH,"  INIT:  exeSem created whith 0\n" ));

	/*------------------------------+
	|  scan descriptor              |
	+------------------------------*/
	/* prepare access */
	if ((error = DESC_Init(descP, osHdl, &llHdl->descHdl)))
		return( Cleanup(llHdl,error) );

	/* DEBUG_LEVEL_DESC */
	if ((error = DESC_GetUInt32(llHdl->descHdl, OSS_DBG_DEFAULT,
								&value, "DEBUG_LEVEL_DESC")) &&
		error != ERR_DESC_KEY_NOTFOUND)
		return( Cleanup(llHdl,error) );

	DESC_DbgLevelSet(llHdl->descHdl, value);	/* set level */

	/* DEBUG_LEVEL */
	if ((error = DESC_GetUInt32(llHdl->descHdl, OSS_DBG_DEFAULT,
								&llHdl->dbgLevel, "DEBUG_LEVEL")) &&
		error != ERR_DESC_KEY_NOTFOUND)
		return( Cleanup(llHdl, error) );

	/* IRQ_ENABLE */
	if ((error = DESC_GetUInt32(llHdl->descHdl, 0,
								&value, "IRQ_ENABLE")) &&
		error != ERR_DESC_KEY_NOTFOUND)
		return( Cleanup(llHdl, error) );

	if( value == 0 )
		llHdl->mode |= Z72_MODE_POLL;

	/* AUTO_SKIPROM */
	if ((error = DESC_GetUInt32(llHdl->descHdl, 1,
								&value, "AUTO_SKIPROM")) &&
		error != ERR_DESC_KEY_NOTFOUND)
		return( Cleanup(llHdl, error) );

	if( value == 1 )
		llHdl->mode |= Z72_MODE_AUTO_SKIPROM;

	/* READ_RETRIES */
	if ((error = DESC_GetUInt32(llHdl->descHdl, Z72_OWB_RETRY,
								&llHdl->retries, "ACCESS_RETRIES")) &&
		error != ERR_DESC_KEY_NOTFOUND)
		return( Cleanup(llHdl, error) );

	DBGWRT_1((DBH, "LL - Z72_Init (ma=0x%08p) in %s mode\n",
					llHdl->ma,
					(llHdl->mode & Z72_MODE_POLL) ? "polled" : "irq"));

	/*------------------------------+
	|  init hardware                |
	+------------------------------*/
	/* do accesses here in polled mode */
	Z72_SetStat(llHdl, M_MK_IRQ_ENABLE, 0, 0);

	if( (error = masterTxReset( llHdl )) != ERR_SUCCESS ) {
		DBGWRT_ERR((DBH, "LL - Z72_Init: No OWB devices present\n"));
		return( Cleanup(llHdl, error) );
	}

	/* enable interrupts? */
	if( !(llHdl->mode & Z72_MODE_POLL) ) {
		Z72_SetStat(llHdl, M_MK_IRQ_ENABLE, 0, 1);
	}

	/* initialize common function pointers */
	llHdl->eeHdl.romRead   = (int32 (*)( void *, u_int8 *, u_int32  )) romRead;
	llHdl->eeHdl.romMatch  = NULL; /* currently not supported (single-drop systems only)*/					
	llHdl->eeHdl.romSearch = NULL; /* currently not supported (single-drop systems only)*/
	llHdl->eeHdl.romSkip   = (int32 (*)( void * )) romSkip;
	
	llHdl->eeHdl.memRead   = (int32 (*)(void *, u_int8 *, u_int16, u_int16)) memRead;

	/* check for EEPROM type, this will also initialize the device specific fkt. pointers */
	error = romRead( llHdl, romBuf, (u_int32) Z72_OWB_EE_ROM_LEN );
	if( error != ERR_SUCCESS )
		return( Cleanup(llHdl, error) );

	*llHdlP = llHdl;	/* set low-level driver handle */

	return(ERR_SUCCESS);
}

/********************************* initEeHandle ******************************/
/** initialize access function pointers for known OWB devices
 *
 *  \param llHdl      \IN  Low-level handle
 *
 *  \return            \c 0 On success or error code
 */
static int32 initEeHandle( LL_HANDLE *llHdl )
{
	/* NULL what we do not support generic */
	llHdl->eeHdl.statusRead  = NULL;
	llHdl->eeHdl.statusWrite = NULL;

	llHdl->eeHdl.memReadCrc  = NULL;
	llHdl->eeHdl.memWrite    = NULL;

	if( llHdl->eeHdl.familyCode == Z72_OWB_DS2502_FAM_CODE ){
		/* currently no write access functions for memory or status supported *
		 * programming voltage of 12V needed for write access, not available */
		 
		llHdl->eeHdl.statusRead  = (int32 (*)( void *, u_int8 *, u_int16, u_int16 ))ds2502_statusRead;
		llHdl->eeHdl.memReadCrc  = (int32 (*)( void *, u_int8 *, u_int16, u_int16 ))ds2502_memReadCrc;
		
		llHdl->eeHdl.memSize	 = (int16)Z72_OWB_DS2502_MEM_SIZE;
		llHdl->eeHdl.memPageSize = (int8)Z72_OWB_DS2502_MEM_PAGESIZE;
		llHdl->eeHdl.tProg		 = 0; /* writing currently not supported */
		

	} else if( llHdl->eeHdl.familyCode == Z72_OWB_DS2431_FAM_CODE ){
		llHdl->eeHdl.memWrite    = (int32 (*)( void *, u_int8 *, u_int16, u_int16 ))ds2431_memWrite;

		llHdl->eeHdl.memSize	 = (int16)Z72_OWB_DS2431_MEM_SIZE;
		llHdl->eeHdl.memPageSize = (int8)Z72_OWB_DS2431_MEM_PAGESIZE;
		llHdl->eeHdl.tProg		 = (int32)Z72_OWB_DS2431_TPROG;
	} else {
		
		return Z72_ERR_DEV_UNKNOWN;
	}
	
	return ERR_SUCCESS;
}
/****************************** Z72_Exit ************************************/
/** De-initialize hardware and clean up memory
 *
 *  The function deinitializes the hardware.
 *  The interrupt is disabled.
 *
 *  \param llHdlP      \IN  Pointer to low-level driver handle
 *
 *  \return           \c 0 On success or error code
 */
static int32 Z72_Exit(
   LL_HANDLE    **llHdlP
)
{
	LL_HANDLE *llHdl = *llHdlP;
	int32 error = 0;

	DBGWRT_1((DBH, "LL - Z72_Exit\n"));

	/*------------------------------+
	|  de-init hardware             |
	+------------------------------*/
	/* disable interrupts */
	Z72_SetStat(llHdl, M_MK_IRQ_ENABLE, 0, 0);

	/*------------------------------+
	|  clean up memory               |
	+------------------------------*/
	*llHdlP = NULL;		/* set low-level driver handle to NULL */
	error = Cleanup(llHdl,error);

	return(error);
}

/****************************** Z72_Read ************************************/
/** Read a value from the device, not supported by this driver!
 *
 *  \param llHdl      \IN  Low-level handle
 *  \param ch         \IN  Current channel
 *  \param valueP     \OUT Read value
 *
 *  \return           \c 0 On success or error code
 */
static int32 Z72_Read(
	LL_HANDLE *llHdl,
	int32 ch,
	int32 *valueP
)
{
	DBGWRT_1((DBH, "LL - Z72_Read: not supported\n"));

	return(ERR_LL_ILL_FUNC);
}

/****************************** Z72_Write ***********************************/
/** Write a value to the device, not supported by this driver!
 *
 *  \param llHdl      \IN  Low-level handle
 *  \param ch         \IN  Current channel
 *  \param value      \IN  Read value
 *
 *  \return           \c 0 on success or error code
 */
static int32 Z72_Write(
	LL_HANDLE *llHdl,
	int32 ch,
	int32 value
)
{
	DBGWRT_1((DBH, "LL - Z72_Write: not supported\n"));

	return(ERR_LL_ILL_FUNC);
}

/****************************** Z72_SetStat *********************************/
/** Set the driver status
 *
 *  The driver supports \ref getstat_setstat_codes "these status codes"
 *  in addition to the standard codes (see mdis_api.h).
 *
 *  \param llHdl  	        \IN  Low-level handle
 *  \param code             \IN  \ref getstat_setstat_codes "status code"
 *  \param ch               \IN  Current channel
 *  \param value32_or_64    \IN  Data or
 *                               pointer to block data structure (M_SG_BLOCK) for
 *                               block status codes
 *  \return                 \c 0 On success or error code
 */
static int32 Z72_SetStat(
	LL_HANDLE *llHdl,
	int32  code,
	int32  ch,
	INT32_OR_64 value32_or_64
)
{
    int32 value = (int32)value32_or_64;	    /* 32bit value */
    /* INT32_OR_64 valueP = value32_or_64;     stores 32/64bit pointer */
    M_SG_BLOCK *blk = (M_SG_BLOCK*)value32_or_64; 	    /* stores block struct pointer */
	int32 error = ERR_SUCCESS;
	u_int8 retVal;
	u_int8  romBuf[Z72_OWB_EE_ROM_LEN];

	DBGWRT_1((DBH, "LL - Z72_SetStat: ch=%d code=0x%04x value=0x%x\n",
			  ch,code,value));

	switch(code) {
		/*--------------------------+
		|  debug level              |
		+--------------------------*/
		case M_LL_DEBUG_LEVEL:
			llHdl->dbgLevel = value;
			break;
		/*--------------------------+
		|  enable interrupts        |
		+--------------------------*/
		case M_MK_IRQ_ENABLE:
			retVal = (u_int8)MREAD_D32( llHdl->ma, Z72_REG_CTL );
			if( value ) {
				/* reset status bits */
				MWRITE_D32( llHdl->ma, Z72_REG_STS,
							(Z72_STS_MIRQ | Z72_STS_EIRQ) );
				/* enable interrupts */
				MWRITE_D32( llHdl->ma, Z72_REG_CTL,
							retVal | Z72_CTL_MIRQ | Z72_CTL_EIRQ );

				/* use interrupts/semaphores to wait for ready */
				llHdl->mode &= ~Z72_MODE_POLL;
			} else {
				/* use polling mode to wait for ready */
				llHdl->mode |= Z72_MODE_POLL;

				/* disable interrupts */
				MWRITE_D32( llHdl->ma, Z72_REG_CTL,
							retVal & ~(Z72_CTL_MIRQ | Z72_CTL_EIRQ) );
			}
			break;
		/*--------------------------+
		|  set irq counter          |
		+--------------------------*/
		case M_LL_IRQ_COUNT:
			llHdl->irqCount = value;
			break;

		/*-----------------------------+
		|  send skip ROM command       |
		+-----------------------------*/
		case Z72_ROM_SKIP:
			error = llHdl->eeHdl.romSkip(llHdl);
			break;

		/*-----------------------------+
		|  set automatic skip ROM mode |
		+-----------------------------*/
		case Z72_ROM_SKIP_AUTOEN:
			if( value )
				llHdl->mode |= Z72_MODE_AUTO_SKIPROM;
			else
				llHdl->mode &= ~Z72_MODE_AUTO_SKIPROM;
			break;

		/*--------------------------+
		|  (unknown)                |
		+--------------------------*/
		default:
			/* single-drop bus? no EEPROM change danger?
				check for EEPROM type, only handle known EEPROMs */
			if( !(llHdl->mode & Z72_MODE_AUTO_SKIPROM) ) {
				error = llHdl->eeHdl.romRead( llHdl, romBuf, (u_int32) Z72_OWB_EE_ROM_LEN );
				if( error != ERR_SUCCESS )
					return( error );
			}
			
			switch(code)
			{
				/*--------------------------+
				|  write memory content       |
				+--------------------------*/
				case Z72_BLK_MEM:
					if( blk->data == NULL )
					{
						error = ERR_LL_ILL_PARAM;
						break;
					}
					error = llHdl->eeHdl.memWrite( llHdl, (u_int8*)blk->data + 2,
										 *(u_int16*)blk->data, (u_int16)blk->size-2 );
					break;
				/*--------------------------+
				|  unknown                  |
				+--------------------------*/
				default:
					error = ERR_LL_UNK_CODE;
			}
	}

	return(error);
}

/****************************** Z72_GetStat *********************************/
/** Get the driver status
 *
 *  The driver supports \ref getstat_setstat_codes "these status codes"
 *  in addition to the standard codes (see mdis_api.h).
 *
 *  \param llHdl            \IN  Low-level handle
 *  \param code             \IN  \ref getstat_setstat_codes "status code"
 *  \param ch               \IN  Current channel
 *  \param value32_or_64P           \IN  Pointer to block data structure (M_SG_BLOCK) for
 *                               block status codes
 *  \param value32_or_64P   \OUT Data pointer or pointer to block data structure
 *                               (M_SG_BLOCK) for block status codes
 *
 *  \return           \c 0 On success or error code
 */
static int32 Z72_GetStat(
	LL_HANDLE *llHdl,
	int32  code,
	int32  ch,
	INT32_OR_64 *value32_or_64P
)
{
    int32 *valueP = (int32*)value32_or_64P;	            /* pointer to 32bit value  */
    INT32_OR_64	*value64P = value32_or_64P;		 		/* stores 32/64bit pointer  */
    M_SG_BLOCK *blk = (M_SG_BLOCK*)value32_or_64P; 	    /* stores block struct pointer */
	u_int8  romBuf[Z72_OWB_EE_ROM_LEN];

	int32 error = ERR_SUCCESS;

	DBGWRT_1((DBH, "LL - Z72_GetStat: ch=%d code=0x%04x\n",
			  ch,code));

	switch(code)
	{
		/*--------------------------+
		|  debug level              |
		+--------------------------*/
		case M_LL_DEBUG_LEVEL:
			*valueP = llHdl->dbgLevel;
			break;
		/*--------------------------+
		|  number of channels       |
		+--------------------------*/
		case M_LL_CH_NUMBER:
			*valueP = CH_NUMBER;
			break;
		/*--------------------------+
		|  channel direction        |
		+--------------------------*/
		case M_LL_CH_DIR:
			*valueP = M_CH_INOUT;
			break;
		/*--------------------------+
		|  channel length [bits]    |
		+--------------------------*/
		case M_LL_CH_LEN:
			*valueP = 32;
			break;
		/*--------------------------+
		|  channel type info        |
		+--------------------------*/
		case M_LL_CH_TYP:
			*valueP = M_CH_BINARY;
			break;
		/*--------------------------+
		|  irq counter              |
		+--------------------------*/
		case M_LL_IRQ_COUNT:
			*valueP = llHdl->irqCount;
			break;
		/*--------------------------+
		|  ID PROM check enabled    |
		+--------------------------*/
		case M_LL_ID_CHECK:
			*valueP = 0;
			break;
		/*--------------------------+
		|   ident table pointer     |
		|   (treat as non-block!)   |
		+--------------------------*/
		case M_MK_BLK_REV_ID:
		   *value64P = (INT32_OR_64)&llHdl->idFuncTbl;
		   break;
		/*--------------------------+
		|  get ROM content          |
		+--------------------------*/
		case Z72_BLK_ROM_READ:
			if( blk->data == NULL ||
				blk->size < Z72_OWB_EE_ROM_LEN )
			{
				error = ERR_LL_ILL_PARAM;
				break;
			}
			error = llHdl->eeHdl.romRead( llHdl, (u_int8*)blk->data, blk->size );
			break;

		/*-------------------------------------+
		|  this will get device specific       |
		+-------------------------------------*/
		default:
			/* single-drop bus? no EEPROM change danger?
				check for EEPROM type, only handle known EEPROMs */
			if( !(llHdl->mode & Z72_MODE_AUTO_SKIPROM) ) {
				error = llHdl->eeHdl.romRead( llHdl, romBuf, (u_int32) Z72_OWB_EE_ROM_LEN );
				if( error != ERR_SUCCESS )
					return( error );
			}
			
			switch(code)
			{
				/*--------------------------+
				|  get memory content       |
				+--------------------------*/
				case Z72_BLK_MEM:
					if( blk->data == NULL )
					{
						error = ERR_LL_ILL_PARAM;
						break;
					}
					/* memRead is always supported, either generic or specific */
					error = llHdl->eeHdl.memRead( llHdl, (u_int8*)blk->data,
										 *(u_int16*)blk->data, (u_int16)blk->size );
					break;
				/*------------------------------------------------+
				|  get memory content & generate CRC on each page |
				+------------------------------------------------*/
				case Z72_BLK_MEM_CRC:
					if( blk->data == NULL )
					{
						error = ERR_LL_ILL_PARAM;
						break;
					}
					if( !llHdl->eeHdl.memReadCrc ) {
						error = ERR_LL_ILL_FUNC;
						break;
					}
					error = llHdl->eeHdl.memReadCrc( llHdl, (u_int8*)blk->data, 
										*(u_int16*)blk->data, (u_int16)blk->size );
					break;
				/*--------------------------+
				|  get status region content       |
				+--------------------------*/
				case Z72_BLK_STATUS:
					if( blk->data == NULL )
					{
						error = ERR_LL_ILL_PARAM;
						break;
					}
					if( !llHdl->eeHdl.statusRead ) {
						error = ERR_LL_ILL_FUNC;
						break;
					}
					error = llHdl->eeHdl.statusRead( llHdl, (u_int8*)blk->data,
										*(u_int16*)blk->data, (u_int16)blk->size);
					break;
				/*--------------------------+
				|  unknown                  |
				+--------------------------*/
				default:
					error = ERR_LL_UNK_CODE;
			}
	}

	return(error);
}

/******************************* Z72_BlockRead ******************************/
/** Read a data block from the device, not supported by this driver!
 *
 *  \param llHdl       \IN  Low-level handle
 *  \param ch          \IN  Current channel
 *  \param buf         \IN  Data buffer
 *  \param size        \IN  Data buffer size
 *  \param nbrRdBytesP \OUT Number of read bytes
 *
 *  \return            \c 0 On success or error code
 */
static int32 Z72_BlockRead(
	 LL_HANDLE *llHdl,
	 int32     ch,
	 void      *buf,
	 int32     size,
	 int32     *nbrRdBytesP
)
{
	DBGWRT_1((DBH, "LL - Z72_BlockRead: not supported\n"));

	/* return number of read bytes */
	*nbrRdBytesP = 0;

	return(ERR_LL_ILL_FUNC);
}

/****************************** Z72_BlockWrite ******************************/
/** Write a data block from the device, not supported by this driver!
 *
 *  \param llHdl  	   \IN  Low-level handle
 *  \param ch          \IN  Current channel
 *  \param buf         \IN  Data buffer
 *  \param size        \IN  Data buffer size
 *  \param nbrWrBytesP \OUT Number of written bytes
 *
 *  \return            \c 0 On success or error code
 */
static int32 Z72_BlockWrite(
	 LL_HANDLE *llHdl,
	 int32     ch,
	 void      *buf,
	 int32     size,
	 int32     *nbrWrBytesP
)
{
	DBGWRT_1((DBH, "LL - Z72_BlockWrite: not supported \n"));

	/* return number of written bytes */
	*nbrWrBytesP = 0;

	return(ERR_LL_ILL_FUNC);
}


/****************************** Z72_Irq ************************************/
/** Interrupt service routine
 *
 *  The interrupt is triggered when ??? occurs.
 *
 *  If the driver can detect the interrupt's cause it returns
 *  LL_IRQ_DEVICE or LL_IRQ_DEV_NOT, otherwise LL_IRQ_UNKNOWN.
 *
 *  \param llHdl  	   \IN  Low-level handle
 *  \return LL_IRQ_DEVICE	IRQ caused by device
 *          LL_IRQ_DEV_NOT  IRQ not caused by device
 *          LL_IRQ_UNKNOWN  Unknown
 */
static int32 Z72_Irq(
   LL_HANDLE *llHdl
)
{
	u_int8 irqReq;
	IDBGWRT_1((DBH, ">>> Z72_Irq:\n"));

	/* interrupt caused by Z72_OWB ? */
	irqReq = (u_int8)MREAD_D32( llHdl->ma, Z72_REG_STS );

	if( irqReq & (Z72_STS_MIRQ | Z72_STS_EIRQ) ) {

		/* release interrupt */
		MWRITE_D32( llHdl->ma, Z72_REG_STS,
					irqReq & (Z72_STS_MIRQ | Z72_STS_EIRQ) );

		/* save status */
		llHdl->exeStatus = irqReq;

		/* inform waiting task */
		OSS_SemSignal( llHdl->osHdl, llHdl->exeSem );

		llHdl->irqCount++;
		return( LL_IRQ_DEVICE );
	}

	return(LL_IRQ_DEV_NOT);		/* say: not me */
}

/****************************** Z72_Info ***********************************/
/** Get information about hardware and driver requirements
 *
 *  The following info codes are supported:
 *
 * \code
 *  Code                      Description
 *  ------------------------  -----------------------------
 *  LL_INFO_HW_CHARACTER      Hardware characteristics
 *  LL_INFO_ADDRSPACE_COUNT   Number of required address spaces
 *  LL_INFO_ADDRSPACE         Address space information
 *  LL_INFO_IRQ               Interrupt required
 *  LL_INFO_LOCKMODE          Process lock mode required
 * \endcode
 *
 *  The LL_INFO_HW_CHARACTER code returns all address and
 *  data modes (ORed) which are supported by the hardware
 *  (MDIS_MAxx, MDIS_MDxx).
 *
 *  The LL_INFO_ADDRSPACE_COUNT code returns the number
 *  of address spaces used by the driver.
 *
 *  The LL_INFO_ADDRSPACE code returns information about one
 *  specific address space (MDIS_MAxx, MDIS_MDxx). The returned
 *  data mode represents the widest hardware access used by
 *  the driver.
 *
 *  The LL_INFO_IRQ code returns whether the driver supports an
 *  interrupt routine (TRUE or FALSE).
 *
 *  The LL_INFO_LOCKMODE code returns which process locking
 *  mode the driver needs (LL_LOCK_xxx).
 *
 *  \param infoType	   \IN  Info code
 *  \param ...         \IN  Argument(s)
 *
 *  \return            \c 0 On success or error code
 */
static int32 Z72_Info(
   int32  infoType,
   ...
)
{
	int32   error = ERR_SUCCESS;
	va_list argptr;

	va_start(argptr, infoType );

	switch(infoType) {
		/*-------------------------------+
		|  hardware characteristics      |
		|  (all addr/data modes ORed)   |
		+-------------------------------*/
		case LL_INFO_HW_CHARACTER:
		{
			u_int32 *addrModeP = va_arg(argptr, u_int32*);
			u_int32 *dataModeP = va_arg(argptr, u_int32*);

			*addrModeP = MDIS_MA08;
			*dataModeP = MDIS_MD08 | MDIS_MD16;
			break;
		}
		/*-------------------------------+
		|  nr of required address spaces |
		|  (total spaces used)           |
		+-------------------------------*/
		case LL_INFO_ADDRSPACE_COUNT:
		{
			u_int32 *nbrOfAddrSpaceP = va_arg(argptr, u_int32*);

			*nbrOfAddrSpaceP = ADDRSPACE_COUNT;
			break;
		}
		/*-------------------------------+
		|  address space type            |
		|  (widest used data mode)       |
		+-------------------------------*/
		case LL_INFO_ADDRSPACE:
		{
			u_int32 addrSpaceIndex = va_arg(argptr, u_int32);
			u_int32 *addrModeP = va_arg(argptr, u_int32*);
			u_int32 *dataModeP = va_arg(argptr, u_int32*);
			u_int32 *addrSizeP = va_arg(argptr, u_int32*);

			if (addrSpaceIndex >= ADDRSPACE_COUNT)
				error = ERR_LL_ILL_PARAM;
			else {
				*addrModeP = MDIS_MA08;
				*dataModeP = MDIS_MD16;
				*addrSizeP = ADDRSPACE_SIZE;
			}

			break;
		}
		/*-------------------------------+
		|   interrupt required           |
		+-------------------------------*/
		case LL_INFO_IRQ:
		{
			u_int32 *useIrqP = va_arg(argptr, u_int32*);

			*useIrqP = USE_IRQ;
			break;
		}
		/*-------------------------------+
		|   process lock mode            |
		+-------------------------------*/
		case LL_INFO_LOCKMODE:
		{
			u_int32 *lockModeP = va_arg(argptr, u_int32*);

			*lockModeP = LL_LOCK_CALL;
			break;
		}
		/*-------------------------------+
		|   (unknown)                    |
		+-------------------------------*/
		default:
		  error = ERR_LL_ILL_PARAM;
	}

	va_end(argptr);
	return(error);
}

/*******************************  Ident  ***********************************/
/** Return ident string
 *
 *  \return            Pointer to ident string
 */
static char* Ident( void )
{
	return( (char*) IdentString );
}

/********************************* Cleanup *********************************/
/** Close all handles, free memory and return error code
 *
 *	\warning The low-level handle is invalid after this function is called.
 *
 *  \param llHdl      \IN  Low-level handle
 *  \param retCode    \IN  Return value
 *
 *  \return           \IN   retCode
 */
static int32 Cleanup(
   LL_HANDLE    *llHdl,
   int32        retCode
)
{
	/*------------------------------+
	|  close handles                |
	+------------------------------*/
	/* clean up desc */
	if (llHdl->descHdl)
		DESC_Exit(&llHdl->descHdl);

	/* clean up semaphores */
	if (llHdl->exeSem != NULL)
		OSS_SemRemove( llHdl->osHdl, &llHdl->exeSem );

	/* clean up debug */
	DBGEXIT((&DBH));

	/*------------------------------+
	|  free memory                  |
	+------------------------------*/
	/* free my handle */
	OSS_MemFree(llHdl->osHdl, (int8*)llHdl, llHdl->memAlloc);

	/*------------------------------+
	|  return error code            |
	+------------------------------*/
	return(retCode);
}

static int32 owbBlkWrite(LL_HANDLE *llHdl, u_int8 *buf, u_int32 numBytes)
{
	int32 error = ERR_SUCCESS;

	do{
		DBGWRT_5((DBH, "LL - Z72_OWB     owbBlkWrite val=0x%02x\n", buf[0]));
		error = execCmd( llHdl, buf, Z72_CTL_CMD_WBYT );
		if( error != ERR_SUCCESS )
			return error;
		buf += 1;
		numBytes -= 1;
	} while (numBytes);
	return error;
}

static int32 owbBlkRead(LL_HANDLE *llHdl, u_int8 *buf, u_int32 numBytes)
{
	int32 error = ERR_SUCCESS;

	do{
		error = execCmd( llHdl, buf, Z72_CTL_CMD_RBYT );
		if( error != ERR_SUCCESS )
			return error;
		DBGWRT_5((DBH, "LL - Z72_OWB     owbBlkRead val=0x%02x\n", buf[0]));
		buf += 1;
		numBytes -= 1;
	} while (numBytes);
	return error;
}

static int32 owbGetToMemFkt(LL_HANDLE *llHdl)
{
	int32 error;
	u_int8 buf[Z72_OWB_EE_ROM_LEN];
	
	if( (error = masterTxReset( llHdl )) != ERR_SUCCESS )
		goto CLEANUP;
	
	/* get to mem state machine */
	if( llHdl->mode & Z72_MODE_AUTO_SKIPROM ) {
		error = llHdl->eeHdl.romSkip( llHdl );
	} else {
		error = llHdl->eeHdl.romRead( llHdl, buf, Z72_OWB_EE_ROM_LEN );
	}
	
CLEANUP:
	return error;
}
/********************************* romRead *********************************/
/** Read content of OWB devices ROM
 *
 *	The function will always read the full content of the ROM in order to
 *  be able to check the CRC which is only received at the end.
 *
 *  \param llHdl      \IN  Low-level handle
 *  \param buf        \OUT buffer for read ROM data
 *  \param numBytes   \IN  number of bytes to read (>= Z72_OWB_EE_ROM_LEN)
 *
 *  \return            \c 0 On success or error code
 */
static int32 romRead( LL_HANDLE *llHdl, u_int8 *buf, u_int32 numBytes )
{
	int32 error = ERR_SUCCESS;
	u_int32 i;
	u_int8 owbCmd;
	u_int8 crc;
	u_int32 readRetries = llHdl->retries;

	DBGWRT_1((DBH, "LL - Z72_OWB romRead\n"));

	if( numBytes < Z72_OWB_EE_ROM_LEN ) {
		error = ERR_LL_ILL_PARAM;
		goto CLEANUP;
	}

	/* do Reset/Presence Pulse Cycle, wake up device */
	if( (error = masterTxReset( llHdl )) != ERR_SUCCESS )
		goto CLEANUP;

	do {
		/* send command */
		owbCmd = Z72_OWB_ROM_READ;
		error = execCmd( llHdl, &owbCmd, Z72_CTL_CMD_WBYT );
		if( error != ERR_SUCCESS )
			goto RETRY_LOOP_END;

		for( i = 0; error == ERR_SUCCESS && i < Z72_OWB_EE_ROM_LEN; i++) {
			error = execCmd( llHdl, &buf[i], Z72_CTL_CMD_RBYT );
		}
		if( error == ERR_SUCCESS ){

			DBGWRT_4(( DBH, "\nZ72_ROM: Family: 0x%02x\n"
							"         Serial: 0x%02x     0x%02x\n",
							buf[0], buf[1], buf[2] ));
			for( i = 3; i < Z72_OWB_EE_ROM_LEN - 1; i+=2) {
				DBGWRT_4(( DBH,	"                 0x%02x     0x%02x\n",
								buf[i], buf[i+1] ));
			}
			DBGWRT_4((DBH,"            CRC: 0x%02x\n", buf[Z72_OWB_EE_ROM_LEN - 1] ));

			crc = bufCrc8( Z72_OWB_ROMCRC_INIT, buf, 7 );
			if( crc != buf[Z72_OWB_EE_ROM_LEN - 1] ) {
				DBGWRT_ERR((DBH,"LL - Z72_OWB romRead: CRC is 0x%02x; sb 0x%02x\n",
								crc, buf[Z72_OWB_EE_ROM_LEN - 1] ));
				error = Z72_ERR_CRC;
				goto RETRY_LOOP_END;
			}
		} else {
			DBGWRT_ERR(( DBH, "\nZ72_ROM: read ROM failed @i=%d\n", i ));
		}
RETRY_LOOP_END:		 
		readRetries -= 1;
	} while ( readRetries && error != ERR_SUCCESS );
	
CLEANUP:
	if( error == ERR_SUCCESS ) {
		llHdl->majState = Z72_ST_MAJ_IDLE_MEM;
		/* known device? init function pointers */
		llHdl->eeHdl.familyCode = buf[0];
		error = initEeHandle(llHdl);
	} else {
		masterTxReset( llHdl );
	}

	return error;
}

/********************************* romSkip ***********************************/
/** Send skip ROM command to OWB device
 *
 *  \param llHdl      \IN  Low-level handle
 *
 *  \return            \c 0 On success or error code
 */
static int32 romSkip( LL_HANDLE *llHdl )
{
	int32 error   = ERR_LL_ILL_FUNC;
	u_int8 owbCmd;
	DBGWRT_1((DBH, "LL - Z72_OWB romSkip\n"));

	/* do Reset/Presence Pulse Cycle */
	if( (error = masterTxReset( llHdl )) != ERR_SUCCESS )
		goto CLEANUP;

	owbCmd = Z72_OWB_ROM_SKIP;
	error = execCmd( llHdl, &owbCmd, Z72_CTL_CMD_WBYT );

	if( error ) {
		masterTxReset( llHdl );
	} else {
		llHdl->majState = Z72_ST_MAJ_IDLE_MEM;
	}

CLEANUP:
	return( error );
}

/********************************* memRead **********************************/
/** Generic function to read content of OWB devices memory region
 *
 *	The function will read the specified amount of bytes from the devices
 *  memory region.
 *
 *  \param llHdl      \IN  Low-level handle
 *  \param buf        \OUT buffer for read data
 *  \param offs       \IN  start offset in region
 *  \param numBytes   \IN  number of bytes to read
 *
 *  \return            \c 0 On success or error code
 */
static int32 memRead( LL_HANDLE *llHdl, u_int8 *buf, u_int16 offs, u_int16 numBytes )
{
	int32 error = ERR_SUCCESS;
	u_int32 readRetries = 0;
	u_int8 retrBuf[Z72_OWB_READ_LEN_MAX];
	u_int8 *rdBuf = buf;
	u_int16 rdNum = numBytes;

	DBGWRT_1((DBH, "LL - Z72_OWB memRead start=0x%04x len=0x%04x\n", offs, numBytes));

	/* do Reset/Presence Pulse Cycle, wake up device */
	if( (error = owbGetToMemFkt( llHdl )) != ERR_SUCCESS )
		goto CLEANUP;

	do {
		error = readMemory( llHdl, Z72_ACC_MEM_READ, rdBuf, offs, &rdNum );
		/* compare the read data, max. rdNum bytes */
		if( error == ERR_SUCCESS ) {
			if( readRetries && (memcmp(buf, retrBuf, rdNum) != 0) ) 
				error = Z72_ERR_CRC;
			
			/* for next loop: swap read buffers */
			if (rdBuf == buf)
				rdBuf = retrBuf;
			else
				rdBuf = buf;
			
			if( llHdl->retries && !readRetries )
				error = 1; /* reread at least once */
		}
		
		readRetries += 1;
	} while ( (llHdl->retries >= readRetries) && (error != ERR_SUCCESS) );
	
CLEANUP:
	DBGWRT_2((DBH, "LL - Z72_OWB memRead finish error=0x%04x\n", error));
	return error;
}

/********************************* readMemory ********************************/
/** Read content of OWB devices memory or status region
 *
 *	The function will read the specified amount of bytes from the devices
 *  memory or status region. Every received CRC is checked for validity.
 *
 *  \param llHdl      \IN  Low-level handle
 *  \param accType    \IN  kind of read to perform 
 *  \param buf        \OUT buffer for read data
 *  \param offs       \IN  start offset in region
 *  \param size       \IN  number of bytes to read
 *
 *  \return            \c 0 On success or error code
 */
static int32 readMemory( LL_HANDLE *llHdl,
						 u_int32 accType,
						 u_int8 *buf,
						 u_int16 offs,
						 u_int16 *size )
{
	int32 error = ERR_SUCCESS;
	u_int32 rdIdx = 0;
	u_int16 rdSize = *size;
	u_int8 owbCmd, rdVal = 0, crc;

	DBGWRT_2((DBH, "LL - Z72_OWB     readMemory start=0x%04x len=0x%04x type=0x%02x\n",
					offs, rdSize, accType));

	if( (rdSize == 0) || (offs >= llHdl->eeHdl.memSize) ) {
		error = ERR_LL_ILL_PARAM;
		goto CLEANUP;
	}
	if( ((offs + rdSize) >= llHdl->eeHdl.memSize) ) {
		rdSize = *size = llHdl->eeHdl.memSize - offs;
		DBGWRT_ERR((DBH, "LL - Z72_OWB readMemory reduced read to len=0x%04x\n",
					rdSize));
	}

	if( llHdl->majState != Z72_ST_MAJ_IDLE_MEM &&
		( llHdl->majState != Z72_ST_MAJ_IDLE ||
		  !(llHdl->mode & Z72_MODE_AUTO_SKIPROM) ||
		  (error = romSkip( llHdl )) != ERR_SUCCESS) )
	{
		/* !IDLE_MEM & (!IDLE | !AUTO_SKIPROM | romSkip() != SUCC ) */
		/* should never happen since we always check family Code before accessing mem */
		if( error == ERR_SUCCESS )
			error = Z72_ERR_NO_ROMFKT;
		goto CLEANUP;
	}

	switch( accType )
	{
		case( Z72_ACC_MEM_READ ):
			owbCmd = Z72_OWB_MEM_READ;
			break;
		case( Z72_ACC_MEM_READ_CRC ):
			/* currently only supported on DS2502 */
			owbCmd = Z72_OWB_DS2502_MEM_READ_CRC;
			break;
		case( Z72_ACC_STA_READ ):
			/* currently only supported on DS2502 */
			owbCmd = Z72_OWB_DS2502_STATUS_READ;
			break;
		default:
			error = ERR_LL_ILL_PARAM;
			goto CLEANUP;
	}
	error = execCmd( llHdl, &owbCmd, Z72_CTL_CMD_WBYT );
	if( error != ERR_SUCCESS )
		goto CLEANUP;

	/* start CRC computing */
	crc = Z72_OWB_ROMCRC_INIT;
	byteCrc( &crc, owbCmd );

	/* send TA1 */
	owbCmd = (u_int8)(offs & 0x00ff);
	error = execCmd( llHdl, &owbCmd, Z72_CTL_CMD_WBYT );
	if( error != ERR_SUCCESS )
		goto CLEANUP;
	byteCrc( &crc, owbCmd );
	
	/* send TA2 */
	owbCmd = (u_int8)((offs & 0xff00) >> 8);
	error = execCmd( llHdl, &owbCmd, Z72_CTL_CMD_WBYT );
	if( error != ERR_SUCCESS )
		goto CLEANUP;
	byteCrc( &crc, owbCmd );

	byteCrcFinish( &crc );
	
	/* verify CRC of CMD + TA[1-2], if supported */
	if( llHdl->eeHdl.familyCode == Z72_OWB_DS2502_FAM_CODE ) {
		/* DS2502 verifies command and TA by CRC! */
		error = execCmd( llHdl, &rdVal, Z72_CTL_CMD_RBYT );
		if( error != ERR_SUCCESS )
			goto CLEANUP;

		if( crc != rdVal ) {
			error = Z72_ERR_CRC;
			DBGWRT_ERR((DBH,"LL - Z72_OWB readMemory: CMD/TA CRC "
							"is 0x%02x; sb 0x%02x\n", crc, rdVal ));
			goto CLEANUP;
		}	
	}

	/* restart CRC, perform actual read memory/status */
	crc = Z72_OWB_ROMCRC_INIT;

	do {
		error = execCmd( llHdl, &rdVal, Z72_CTL_CMD_RBYT );
		if( error != ERR_SUCCESS )
			goto CLEANUP;

		buf[rdIdx] = rdVal;
		rdSize -= 1;

		byteCrc( &crc, rdVal );

		DBGWRT_4((DBH, "LL - Z72_OWB         readMemory: rdIdx 0x%04x, rdVal=0x%02x    crc=0x%02x\n",
						rdIdx, rdVal, crc ));

		/* ready for CRC? check only DS2502 */
		if( llHdl->eeHdl.familyCode == Z72_OWB_DS2502_FAM_CODE ) {
			if( (accType == Z72_ACC_MEM_READ && !rdSize) ||
				(accType == Z72_ACC_MEM_READ_CRC && !((rdIdx + offs + 1) % llHdl->eeHdl.memPageSize)) ||
				(accType == Z72_ACC_STA_READ && !rdSize) )
			{
				byteCrcFinish( &crc );

				error = execCmd( llHdl, &rdVal, Z72_CTL_CMD_RBYT );
				if( error == ERR_SUCCESS && 
					crc != rdVal ) 
				{
					error = Z72_ERR_CRC;
					DBGWRT_ERR((DBH,"LL - Z72_OWB readMemory: data CRC "
									"is 0x%02x; sb 0x%02x\n", crc, rdVal ));
					goto CLEANUP;
				}
				/* restart CRC for next page */
				crc = Z72_OWB_ROMCRC_INIT;
			}
		}

		rdIdx++;
	} while ( rdSize );


CLEANUP:
	DBGWRT_3((DBH, "LL - Z72_OWB     readMemory finish (error = 0x%04x)\n",error));
	masterTxReset( llHdl );
	return error;
}

/********************************* masterTxReset *****************************/
/** Perform a Reset/Presence Pulse Cycle on the OWBus
 *
 *  \param llHdl      \IN  Low-level handle
 *
 *  \return           \c 0 On success or error code
 */
static int32 masterTxReset( LL_HANDLE *llHdl )
{
	llHdl->majState = Z72_ST_MAJ_IDLE;
	return( execCmd( llHdl, NULL, Z72_CTL_CMD_RPP ) );
}

/********************************* waitOnReady *******************************/
/** wait for started command to finish
 *
 *  \param llHdl      \IN  Low-level handle
 *
 *  \return           \c 0 On success or error code
 */
static int32 waitOnReady( LL_HANDLE *llHdl )
{
	int32 error = ERR_SUCCESS;
	u_int32 retryCnt = Z72_POLL_RETRY_MAX;

	if( !(llHdl->mode & Z72_MODE_POLL) &&
		(llHdl->exeSem != NULL) )
	{
		/* Note: signals are ignored here */
		do {
			DBGCMD( u_int8 ctl );
			DBGCMD( u_int8 sts );

			error = OSS_SemWait( llHdl->osHdl, llHdl->exeSem, Z72_SEM_TOUT );

			DBGCMD( ctl = (u_int8)MREAD_D32( llHdl->ma, Z72_REG_CTL ) );
			DBGCMD( sts = (u_int8)MREAD_D32( llHdl->ma, Z72_REG_STS ) );
			DBGWRT_5((DBH, "LL - Z72_OWB waitOnReady(Sem) finish "
							   "(error = 0x%04x) ctl=0x%02x sts=0x%02x\n",
							   error, ctl, sts));
		} while( (error==ERR_OSS_SIG_OCCURED) || (!(error==ERR_SUCCESS) && retryCnt--) );
	} else
	{
		u_int8 sts;
		DBGCMD( u_int8 ctl );

		do{
			OSS_Delay( llHdl->osHdl, Z72_POLL_TOUT );
			sts = (u_int8)MREAD_D32( llHdl->ma, Z72_REG_STS );
		} while( (sts & Z72_STS_BUSY) && retryCnt-- );
		/* save status */
		llHdl->exeStatus = (u_int8)MREAD_D32( llHdl->ma, Z72_REG_STS );

		DBGCMD( ctl = (u_int8)MREAD_D32( llHdl->ma, Z72_REG_CTL ));
		DBGWRT_5((DBH, "LL - Z72_OWB waitOnReady(Delay) finish "
					   "(error = 0x%04x) ctl=0x%02x sts=0x%02x\n",
					   error, ctl, llHdl->exeStatus ));


		/* reset status bits */
		MWRITE_D32( llHdl->ma, Z72_REG_STS,
					llHdl->exeStatus & (Z72_STS_MIRQ | Z72_STS_EIRQ) );
	}

	return error;
}

/********************************* getDevError *******************************/
/** Parse saved status for errors
 *
 *  \param llHdl      \IN  Low-level handle
 *
 *  \return           \c   0 On success or error code detected
 */
static int32 getDevError( LL_HANDLE *llHdl )
{
	int32 error = ERR_SUCCESS;

	if( llHdl->exeStatus & ( Z72_STS_EEXE | Z72_STS_ERPP ) )
		error = Z72_ERR_NO_DEVICE;
	else if( llHdl->exeStatus & ( Z72_STS_EPP1 | Z72_STS_EPP2) )
		error = Z72_ERR_PGM;
	return error;
}

/********************************* execCmd ***********************************/
/** Execute specified command
 *
 *  \param llHdl      \IN  Low-level handle
 *  \param data       \OUT buffer for data read or to write
 *  \param cmd        \IN  command to execute
 *
 *  \return           \c   0 On success or error code
 */
static int32 execCmd( LL_HANDLE *llHdl, u_int8 *data, u_int32 cmd )
{
	int32 error = ERR_SUCCESS;
	u_int8 ctlReg, stsReg, tryCnt = 0;

	stsReg = (u_int8)MREAD_D32( llHdl->ma, Z72_REG_STS );

	while( (stsReg & Z72_STS_BUSY) &&
		   (tryCnt++ < Z72_RETRY_MAX) ){
		OSS_Delay( llHdl->osHdl, Z72_POLL_TOUT );
		stsReg = (u_int8)MREAD_D32( llHdl->ma, Z72_REG_STS );
	}

	if( tryCnt >= Z72_RETRY_MAX ) {
		error = Z72_ERR_BUSY;
		goto CLEANUP;
	}

	if( cmd == Z72_CTL_CMD_WBIT ||
		cmd == Z72_CTL_CMD_WBYT )
	{
		MWRITE_D32( llHdl->ma, Z72_REG_WRT, *data );
	}

	llHdl->exeStatus = 0;

	ctlReg = (u_int8)MREAD_D32( llHdl->ma, Z72_REG_CTL ) & ~Z72_CTL_CMD_MASK;
	MWRITE_D32( llHdl->ma, Z72_REG_CTL, ctlReg | (u_int8)cmd | Z72_CTL_EXEC );

	error = waitOnReady( llHdl );

	if( error ||
		(error = getDevError( llHdl )) != ERR_SUCCESS )
	{
		goto CLEANUP;
	}

	if( cmd == Z72_CTL_CMD_RBIT ||
		cmd == Z72_CTL_CMD_RBYT )
	{
		*data = (u_int8)MREAD_D32( llHdl->ma, Z72_REG_RD );
	}

CLEANUP:
	return error;
}

/* CRC subroutines */

/********************************* mirrorByte *******************************/
/** mirror byte specified (MSB <---> LSB)
 *
 *  \param c          \IN  byte to mirror
 *
 *  \return           \c   mirrored byte
 */
static u_int8 mirrorByte ( u_int8 c )
{
	/* reflects the Byte c */
	u_int32 i, j = 1;
	u_int8 cout = 0;

	for( i = 0x80; i; i >>= 1) {
		if( c & i )
			cout |= j;
		j <<= 1;
	}
	return( cout );
}

/********************************* byteCrc ***********************************/
/** compute CRC8 for passed byte based on old CRC value
 *
 *  \param crc        \INOUT  buffer with old/new CRC value
 *  \param c          \IN     character to be added to crc
 */
static void byteCrc( u_int8 *crc, u_int8 c )
{
	u_int32 j;
	u_int8 bit;

	c = mirrorByte( c );

	for( j = 0x80; j; j >>= 1)
	{
		bit = *crc & Z72_OWB_ROMCRC_HIGHBIT;
		*crc <<= 1;
		if( c & j )
			*crc |= 1;
		if( bit )
			*crc ^= Z72_OWB_ROMCRC_POLY;
	}
}

/********************************* byteCrcFinish *****************************/
/** finsh crc computation and mirror crc for comparing
 *
 *  \param crc        \INOUT  buffer with old/new CRC value
 */
static void byteCrcFinish( u_int8 *crc )
{
	u_int32 i;
	u_int8 bit;

	for( i = 0; i < Z72_OWB_ROMCRC_ORDER; i++)
	{
		bit = *crc & Z72_OWB_ROMCRC_HIGHBIT;
		*crc <<= 1;
		if( bit )
			*crc ^= Z72_OWB_ROMCRC_POLY;
	}
	*crc = mirrorByte( *crc );
}

/********************************* bufCrc8 ***********************************/
/** compute CRC on specified buffer
 *
 *  \param crcStart   \IN  Init value for CRC
 *  \param buf        \IN  buffer with data
 *  \param len        \IN  number of bytes to consider from buf
 *
 *  \return            \c computed crc
 */
static u_int8 bufCrc8( u_int8 crcStart, u_int8 *buf, u_int32 len)
{
	u_int32 i;
	u_int8 crc = crcStart;

	for( i = 0; i < len; i++)
	{
		byteCrc( &crc, *buf++ );
	}

	byteCrcFinish( &crc );
	return(crc);
}

/********************************* ds2502_statusRead ************************/
/** DS2502: read EEPROM status region
 *
 *	The function will read the specified amount of bytes from the devices
 *  status region.
 *
 *  \param llHdl      \IN  Low-level handle
 *  \param buf        \OUT buffer for read ROM data
 *  \param offs       \IN  start offset in region to read
 *  \param numBytes   \IN  number of bytes to read
 *
 *  \return            \c 0 On success or error code
 */
static int32 ds2502_statusRead( LL_HANDLE *llHdl, u_int8 *buf, u_int16 offs, u_int16 numBytes )
{
	int32 error = ERR_SUCCESS;
	
	if( (offs + numBytes) > Z72_OWB_DS2502_ST_SIZE ) {
		DBGWRT_ERR((DBH,"LL - Z72_OWB DS2502 readStatus: wrong offset/numBytes: 0x%02x/%02x\n", offs, numBytes ));
		return ERR_LL_ILL_PARAM;
	}
	
	error = readMemory( llHdl, Z72_ACC_STA_READ, buf, offs, &numBytes );
	
	return error;
	
}

/********************************* ds2502_memReadCrc ************************/
/** DS2502: read EEPROM memory region, generate CRC
 *
 *	The function will read the specified amount of bytes from the devices
 *  memory region and generate the CRC.
 *
 *  \param llHdl      \IN  Low-level handle
 *  \param buf        \OUT buffer for read ROM data
 *  \param offs       \IN  start offset in region to read
 *  \param numBytes   \IN  number of bytes to read
 *
 *  \return            \c 0 On success or error code
 */
static int32 ds2502_memReadCrc( LL_HANDLE *llHdl, u_int8 *buf, u_int16 offs, u_int16 numBytes )
{
	int32 error = ERR_SUCCESS;
	u_int32 readRetries = llHdl->retries;
	
	if( (offs + numBytes) > Z72_OWB_DS2502_MEM_SIZE ) {
		DBGWRT_ERR((DBH,"LL - Z72_OWB DS2502 memReadCrc: wrong offset/numBytes: 0x%02x 0x%02x\n", offs, numBytes ));
		return ERR_LL_ILL_PARAM;
	}
	
	do{
		error = readMemory( llHdl, Z72_ACC_MEM_READ_CRC, buf, offs, &numBytes );
	} while ( error != ERR_SUCCESS && readRetries-- );

	return error;
	
}

/*************************** ds2431_memWrite ********************************/
/** Write content to OWB devices scratch pad, verify and copy to EEPROM
*   Data must be aligned to scratchpad size already!
*
 *  \param llHdl      \IN  Low-level handle
 *  \param buf        \OUT buffer for read data
 *  \param offs       \IN  start offset in region
 *
 *  \return            \c 0 On success or error code
*/
static int32 ds2431_scpWrite( LL_HANDLE *llHdl, 
							u_int8 *buf, 
							u_int16 offs ) 
{
	int32 error = ERR_SUCCESS;
	u_int8 wrRetries = 0;
	u_int8 wBuf[4];
	u_int8 rBuf[Z72_OWB_DS2431_SCP_SIZE+3];
	u_int8 esByte = 7; /* (offs + numBytes -1) % Z72_OWB_DS2431_SCP_SIZE; */

	DBGWRT_2((DBH, "LL - Z72_OWB     ds2431_scpWrite offset=0x%04x\n", offs));

	do {
		/* do Reset/Presence Pulse Cycle, wake up device, get to MEM state machine */
		if( (error = owbGetToMemFkt( llHdl )) != ERR_SUCCESS )
			goto RETRY_LOOP_END;
		
		wBuf[0] = Z72_OWB_DS2431_SCP_WRITE;
		wBuf[1] = offs & 0xff;
		wBuf[2] = offs >> 8;
		
		error = owbBlkWrite( llHdl, wBuf, 3 );
		if( error != ERR_SUCCESS )
			goto RETRY_LOOP_END;

		error = owbBlkWrite( llHdl, buf, Z72_OWB_DS2431_SCP_SIZE );
		if( error != ERR_SUCCESS )
			goto RETRY_LOOP_END;

#ifdef OWB_WITH_CRC16		
		/* read the CRC and compare to computed one */
		error = owbBlkRead( llHdl, rBuf, 2 );
		if( error != ERR_SUCCESS || rBuf[0] != crc16 ) {
			error = Z72_ERR_CRC;
			goto RETRY_LOOP_END;
		}
#else
		/* do Reset/Presence Pulse Cycle, wake up device, get to MEM state machine */
		if( (error = owbGetToMemFkt( llHdl )) != ERR_SUCCESS )
			goto RETRY_LOOP_END;

		wBuf[0] = Z72_OWB_DS2431_SCP_READ;
		error = execCmd( llHdl, wBuf, Z72_CTL_CMD_WBYT );
		if( error != ERR_SUCCESS )
			goto RETRY_LOOP_END;
		
		error = owbBlkRead( llHdl, rBuf, Z72_OWB_DS2431_SCP_SIZE + 3 );
		if( error != ERR_SUCCESS )
			goto RETRY_LOOP_END;

		/* compare written and read data, instead of CRC! */
		if( rBuf[0] != wBuf[1] || 
			rBuf[1] != wBuf[2] ||
			rBuf[2] != esByte || 
			(memcmp(buf, &rBuf[3], Z72_OWB_DS2431_SCP_SIZE) != 0) )
		{
			DBGWRT_ERR((DBH,"LL - Z72_OWB DS2502     ds2431_scpWrite: failed to write scratch pad (retry=%d)\n", wrRetries ));
			error = Z72_ERR_CRC;
			goto RETRY_LOOP_END;
		}
#endif
		/* do Reset/Presence Pulse Cycle, wake up device, get to MEM state machine */
		if( (error = owbGetToMemFkt( llHdl )) != ERR_SUCCESS )
			goto RETRY_LOOP_END;

		wBuf[0] = Z72_OWB_DS2431_SCP_COPY;
		wBuf[3] = esByte;
		error = owbBlkWrite( llHdl, wBuf, 4 );
		if( error != ERR_SUCCESS )
			goto RETRY_LOOP_END;

		OSS_Delay( llHdl->osHdl, llHdl->eeHdl.tProg ); /* sleep some ms to wait for programming to finish */
		error = execCmd( llHdl, rBuf, Z72_CTL_CMD_RBYT );
		if( error != ERR_SUCCESS || rBuf[0] != 0xAA ) {
			DBGWRT_ERR((DBH, "LL - Z72_OWB     ds2431_scpWrite  SCP_COPY answer=0x%02x sb 0xAA\n", rBuf[0] ));
			error = Z72_ERR_CRC;
		}										   

RETRY_LOOP_END:
		DBGWRT_4((DBH, "LL - Z72_OWB     ds2431_scpWrite error=0x%04x\n", error));
		wrRetries += 1;
	} while( error != ERR_SUCCESS && wrRetries < llHdl->retries );

	DBGWRT_3((DBH, "LL - Z72_OWB     ds2431_scpWrite error=0x%04x retries=%d\n", error, wrRetries-1));
	
	masterTxReset( llHdl );
	return error;
}

/*************************** ds2431_memWrite ********************************/
/** Write content to OWB devices memory region
 *
 *	The function will write the specified amount of bytes to the devices
 *  memory region. If data is not aligned to Z72_OWB_DS2431_SP_SIZE we will
 *  read memory region of size Z72_OWB_DS2431_SP_SIZE and only modify affected
 *  bytes.
 *
 *  \param llHdl      \IN  Low-level handle
 *  \param buf        \OUT buffer for read data
 *  \param offs       \IN  start offset in region
 *  \param numBytes   \IN  number of bytes to write
 *
 *  \return            \c 0 On success or error code
 */
static int32 ds2431_memWrite( LL_HANDLE *llHdl,
						 u_int8 *buf,
						 u_int16 offs,
						 u_int16 numBytes )
{
	int32 error = ERR_SUCCESS;
	u_int16 bWritten = 0;
	u_int16 bAddr = offs;
	u_int16 bLen = numBytes;
	u_int8 cpLen = 0;
	u_int8 rBuf[Z72_OWB_DS2431_SCP_SIZE];

	DBGWRT_1((DBH, "LL - Z72_OWB ds2431_memWrite offset=0x%04x; size=0x%04x\n", offs, numBytes));
	
	if( (offs + numBytes) > llHdl->eeHdl.memSize || numBytes == 0 )
		return ERR_LL_ILL_PARAM;

	/* only full scratchpad pages can be written.  *
	 * align data with data from memory!           */
	do{
		if( bAddr & Z72_OWB_DS2431_SCP_MASK ||
			bLen < Z72_OWB_DS2431_SCP_SIZE ){
			/* read memory to stuff data around data to write */
			error = llHdl->eeHdl.memRead( llHdl, rBuf, (bAddr & ~Z72_OWB_DS2431_SCP_MASK), Z72_OWB_DS2431_SCP_SIZE );
			if( error != ERR_SUCCESS )
				goto CLEANUP;
			
			/* copy over read data, up to SCP boundary or max. bLen */
			cpLen = Z72_OWB_DS2431_SCP_SIZE - (bAddr & Z72_OWB_DS2431_SCP_MASK);
			if (cpLen > bLen)
				cpLen = bLen;
			memcpy(&rBuf[bAddr & Z72_OWB_DS2431_SCP_MASK], buf, cpLen );

			error = ds2431_scpWrite(llHdl, rBuf, (bAddr & ~Z72_OWB_DS2431_SCP_MASK));
			if( error != ERR_SUCCESS )
				goto CLEANUP;
			
		} else {
			/* copy full scratch pad size */
			error = ds2431_scpWrite(llHdl, buf, bAddr);
			cpLen = Z72_OWB_DS2431_SCP_SIZE;
			if( error != ERR_SUCCESS )
				goto CLEANUP;
		}
		DBGWRT_4((DBH, "LL - Z72_OWB ds2431_memWrite bAddr=0x%04x; cpLen=0x%04x\n", bAddr, cpLen));

		buf += cpLen;
		bLen -= cpLen;
		bAddr += cpLen;
		bWritten += cpLen;
	} while ( bWritten < numBytes && error == ERR_SUCCESS );
	
CLEANUP:
	DBGWRT_2((DBH, "LL - Z72_OWB ds2431_memWrite finish (error = 0x%04x)\n",error));
	masterTxReset( llHdl );
	return error;
}


