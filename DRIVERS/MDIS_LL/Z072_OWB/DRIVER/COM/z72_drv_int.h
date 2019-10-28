/***********************  I n c l u d e  -  F i l e  ***********************/
/*!
 *        \file  z72_drv_int.h
 *
 *      \author  cs
 *        $Date: 2006/06/02 16:40:14 $
 *    $Revision: 1.1 $
 *
 *       \brief  Internal header file for Z72_OWB driver containing
 *               defines and forward declarations
 *
 *    \switches
 *
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 2006 by MEN Mikro Elektronik GmbH, Nuremberg, Germany
 ****************************************************************************/

#ifndef _Z72_OWB_DRV_INT_H
#define _Z72_OWB_DRV_INT_H

#ifdef __cplusplus
      extern "C" {
#endif

#define _NO_LL_HANDLE		/* ll_defs.h: don't define LL_HANDLE struct */

#include <MEN/men_typs.h>   /* system dependent definitions   */
#include <MEN/maccess.h>    /* hw access macros and types     */
#include <MEN/dbg.h>        /* debug functions                */
#include <MEN/oss.h>        /* oss functions                  */
#include <MEN/desc.h>       /* descriptor functions           */
#include <MEN/mdis_api.h>   /* MDIS global defs               */
#include <MEN/mdis_com.h>   /* MDIS common defs               */
#include <MEN/mdis_err.h>   /* MDIS error codes               */
#include <MEN/ll_defs.h>    /* low-level driver definitions   */


/*-----------------------------------------+
|  TYPEDEFS                                |
+-----------------------------------------*/
/** low-level handle */
typedef struct {
	/* general */
	int32           memAlloc;		/**< Size allocated for the handle */
	OSS_HANDLE      *osHdl;         /**< OSS handle */
	OSS_IRQ_HANDLE  *irqHdl;        /**< IRQ handle */
	DESC_HANDLE     *descHdl;       /**< DESC handle */
	MACCESS         ma;             /**< HW access handle */
	MDIS_IDENT_FUNCT_TBL idFuncTbl;	/**< ID function table */
	/* debug */
	u_int32         dbgLevel;		/**< Debug level */
	DBG_HANDLE      *dbgHdl;        /**< Debug handle */
	/* misc */
	u_int32			mode;		    /**< mode used (irq/polled) */
	u_int32         irqCount;       /**< Interrupt counter */
	u_int32			exeStatus;		/**< status of last executed cmd */
	OSS_SEM_HANDLE  *exeSem;        /**< command ready */
	OSS_SEM_HANDLE	*devSemHdl;		/**< device semaphore */
	u_int8			*dataP;			/**< data pointer */

	/* state machines */
	u_int32			majState;		/**< major state machine */
	u_int32			fncState;		/**< single functions state machine */
} LL_HANDLE;


/*-----------------------------------------+
|  DEFINES                                 |
+-----------------------------------------*/
/* general defines */
#define CH_NUMBER			1		/**< Number of device channels */
#define USE_IRQ				TRUE	/**< Interrupt required  */
#define ADDRSPACE_COUNT		1		/**< Number of required address spaces */
#define ADDRSPACE_SIZE		0x10	/**< Size of address space */

#define Z72_POLL_TOUT       1 		/**< timeout polled OWB access, msec */
#define Z72_POLL_RETRY_MAX  3		/**< maximum retrys if bus still busy */
#define Z72_SEM_TOUT        2 		/**< timeout irq based OWB access, msec */

#define Z72_RETRY_MAX       5		/**< maximum retrys if bus busy */

#define Z72_EXTRADBG

/* debug defines */
#ifndef Z72_EXTRADBG
#	define DBG_MYLEVEL		llHdl->dbgLevel		/**< Debug level */
#else
#	define DBG_MYLEVEL		DBG_ALL 			/**< Debug level all */
#endif /* Z72_EXTRADBG */
#define DBH					llHdl->dbgHdl		/**< Debug handle */

#define OSH                 llHdl->osHdl		/**< OSS handle */

/** \name Z72 Register offsets and register bit mappings
 *  \anchor z72_reg_offsets
 */
/**@{*/
#define Z72_REG_WRT		0x00	/**< Write Register   */
#define Z72_REG_RD		0x04	/**< Read Register    */
#define Z72_REG_CTL		0x08	/**< Control Register */
#define Z72_REG_STS		0x0C	/**< Status Register  */

#define Z72_CTL_CMD_MASK 0x07	/**< Command mask */
#define Z72_CTL_CMD_WBIT 0x00	/**< Write one bit */
#define Z72_CTL_CMD_RBIT 0x01	/**< Read one bit */
#define Z72_CTL_CMD_RPP  0x02	/**< Reset and presence pulse (rpp) */
#define Z72_CTL_CMD_PRGP 0x03	/**< Program pulse(pp) */
#define Z72_CTL_CMD_WBYT 0x04	/**< Write one byte */
#define Z72_CTL_CMD_RBYT 0x05	/**< Read one byte */

#define Z72_CTL_EXEC 	0x08	/**< Execute commands */
#define Z72_CTL_MIRQ 	0x10	/**< Message interrupt request enable **/
#define Z72_CTL_EIRQ 	0x20	/**< Error interrupt request enable */

#define Z72_STS_EEXE	0x01	/**< Execution Error */
#define Z72_STS_ERPP	0x02	/**< Reset and presence pulse error */
#define Z72_STS_EPP1	0x04	/**< Program phase T_DP timing error */
#define Z72_STS_EPP2	0x08	/**< Program phase T_DV timing error */
#define Z72_STS_MIRQ	0x10	/**< Message interrupt */
#define Z72_STS_EIRQ	0x20	/**< Error interrupt */
#define Z72_STS_BUSY	0x40	/**< Busy Executing command*/
#define Z72_STS_PRES	0x80	/**< Device present */
/**@}*/

/** \name Z72 driver state machines
 *  \anchor z72_drv_state_machines
 */
/**@{*/
/** major state machine */
typedef enum {
	Z72_ST_MAJ_IDLE = 0,			/**< IDLE state */
	Z72_ST_MAJ_RP_PP,				/**< send Reset get Present Pulse */
	Z72_ST_MAJ_ROM_SKIP,			/**< skip ROM */
	Z72_ST_MAJ_ROM_READ,			/**< read ROM */
	Z72_ST_MAJ_ROM_MATCH,			/**< match ROM */
	Z72_ST_MAJ_ROM_SEARCH,			/**< search ROM */
	Z72_ST_MAJ_IDLE_MEM,			/**< IDLE state after succ. ROM function */
	Z72_ST_MAJ_MEM_READ,			/**< read data */
	Z72_ST_MAJ_MEM_READ_CRC,		/**< read data & generate CRC */
	Z72_ST_MAJ_MEM_WRITE,			/**< write data */
	Z72_ST_MAJ_STATUS_READ,			/**< read status */
	Z72_ST_MAJ_STATUS_WRITE,		/**< write status */
	Z72_ST_MAJ_END					/**< send Reset Pulse */
} Z72_STATE_MAJOR;


/** read data (& generate CRC) / read status state machine */
typedef enum {
	Z72_ST_RX_CMD = 0,				/**< Tx command */
	Z72_ST_RX_TA,					/**< Tx TA1 (T7:T0) & TA2 (T15:T8)	*/
	Z72_ST_RX_CRC1,					/**< Rx CRC of CMD and ADDR 		*/
	Z72_ST_RX_DATA,					/**< Rx data/status from memory 	*/
	Z72_ST_RX_CRC2,					/**< Rx CRC of data 				*/
									/*!< only when read to the end
									 *   of data memory 				*/
	Z72_ST_RX_END					/**< Tx Reset pulse					*/
} Z72_STATE_READ;

/** write data / status state machine */
typedef enum {
	Z72_ST_TX_CMD = 0,				/**< Tx command */
	Z72_ST_TX_TA,					/**< Tx TA1 (T7:T0) & TA2 (T15:T8) 	*/
	Z72_ST_TX_DATA,					/**< Tx data			 			*/
	Z72_ST_TX_CRC,					/**< Rx CRC of CMD,ADDR & DATA		*/
	Z72_ST_TX_PGMP,					/**< Tx program pulse				*/
	Z72_ST_TX_DATA_VER,				/**< Rx data for verification		*/
	Z72_ST_TX_END					/**< Tx Reset pulse					*/
} Z72_STATE_WRITE;
/**@}*/


/** mode flags */
#define Z72_MODE_POLL          0x00000001
#define Z72_MODE_AUTO_SKIPROM  0x00000002


/**
 * @defgroup _Z72_DS2502 OWB / DS2502 HW specific defines
 * @{ */
#define Z72_OWB_ROMCRC_ORDER	0x08	/**< order of CRC used by OWB		*/
#define Z72_OWB_ROMCRC_HIGHBIT	0x80	/**< highest bit in CRC used by OWB	*/
#define Z72_OWB_ROMCRC_POLY		0x31    /**< polynome of CRC used by OWB	*/
#define Z72_OWB_ROMCRC_INIT		0x00	/**< init value of CRC used by OWB	*/

/** common OWB commands */
#define Z72_OWB_ROM_READ 	0x33	/**< Read ROM command 				*/
#define Z72_OWB_ROM_MATCH 	0x55	/**< Match ROM command 				*/
#define Z72_OWB_ROM_SEARCH 	0xF0	/**< Search ROM command 			*/
#define Z72_OWB_ROM_SKIP 	0xCC	/**< Skip ROM command 				*/

/** commands of DS2502 */
#define Z72_OWB_DS2502_MEM_READ 	0xF0 /**< Read memory command 			*/
#define Z72_OWB_DS2502_MEM_READ_CRC	0xC3 /**< Read memory & gen. CRC command */
#define Z72_OWB_DS2502_MEM_WRITE 	0x0F /**< Write memory command 			*/
#define Z72_OWB_DS2502_STATUS_READ 	0xAA /**< Read status command 			*/
#define Z72_OWB_DS2502_STATUS_WRITE	0x55 /**< Write status command 			*/
/**@}*/

#ifdef __cplusplus
      }
#endif

#endif /* _Z72_OWB_DRV_INT_H */
