/***********************  I n c l u d e  -  F i l e  ***********************/
/*!
 *        \file  z72_drv.h
 *
 *      \author  cs
 *
 *       \brief  Header file for Z72_OWB driver containing
 *               Z72 specific status codes and
 *               Z72 function prototypes
 *
 *    \switches  _ONE_NAMESPACE_PER_DRIVER_
 *               _LL_DRV_
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

#ifndef _Z72_OWB_DRV_H
#define _Z72_OWB_DRV_H

#ifdef __cplusplus
      extern "C" {
#endif


/*-----------------------------------------+
|  TYPEDEFS                                |
+-----------------------------------------*/
/* none */

/*-----------------------------------------+
|  DEFINES                                 |
+-----------------------------------------*/
/** \name OWB / DSxxxx EEPROM specific defines
 *  \anchor owb_dsxxxx_defines
 *  all lengths are specified in bytes
 */
/**@{*/
#define Z72_OWB_EE_ROM_LEN		 8	/**< Length of ROM in OWB devices	*/
#define Z72_OWB_READ_LEN_MAX	 Z72_OWB_DS2431_MEM_SIZE	/**< max. length of ROM/mem in OWB devices	*/
/**@}*/

/** \name OWB / DS2502 specific defines
 *  \anchor owb_ds2520_defines
 *  all lengths are specified in bytes
 */
/**@{*/
#define Z72_OWB_DS2502_FAM_CODE   0x09	/**< DS2502 family code	*/
#define Z72_OWB_DS2502_MEM_SIZE   0x80	/**< memory section size      (bytes) */
#define Z72_OWB_DS2502_MEM_PAGESIZE	32  /**< memory section page size (bytes) */
#define Z72_OWB_DS2502_ST_SIZE	     8	/**< Status registers size    (bytes) */
/**@}*/

/** \name OWB / DS2431 specific defines
 *  \anchor owb_ds2431_defines
 *  all lengths are specified in bytes
 */
/**@{*/
#define Z72_OWB_DS2431_FAM_CODE   0x2D	/**< family code	                  */
#define Z72_OWB_DS2431_MEM_SIZE   0x90	/**< memory section size      (bytes) */
#define Z72_OWB_DS2431_MEM_PAGESIZE	32  /**< memory section page size (bytes) */
#define Z72_OWB_DS2431_SCP_ADDR_BITS 3	/**< scratch pad address bits (bytes) */
#define Z72_OWB_DS2431_SCP_SIZE	  (1<<Z72_OWB_DS2431_SCP_ADDR_BITS) /**< scratch pad size */
#define Z72_OWB_DS2431_SCP_MASK	  (Z72_OWB_DS2431_SCP_SIZE-1)       /**< scratch pad address mask */


/** \name Z72 specific Getstat/Setstat standard codes
 *  \anchor getstat_setstat_codes
 */
/**@{*/
#define Z72_ROM_SKIP        M_DEV_OF+0x01   /**<  S: send skip ROM           */ 
#define Z72_ROM_SKIP_AUTOEN M_DEV_OF+0x02   /**<  S: enable auto skip ROM    */
                                            /*!<     0: disabled (default)   \n
                                             *       1: enabled              */
/**@}*/

/** \name Z72 specific Getstat/Setstat block codes */
/**@{*/
#define Z72_BLK_ROM_READ     M_DEV_BLK_OF+0x00  /**< G:   get ROM            */
#define Z72_BLK_ROM_MATCH    M_DEV_BLK_OF+0x01  /**< S:   match - Not supported yet */
#define Z72_BLK_ROM_SEARCH   M_DEV_BLK_OF+0x02  /**< G:   search ROMs - Not supported yet */
#define Z72_BLK_MEM          M_DEV_BLK_OF+0x03  /**< G,S: read/write mem     *
                                                  *       first two bytes of msgBlk.data hold offset for access */
#define Z72_BLK_MEM_CRC      M_DEV_BLK_OF+0x04  /**< G:   read mem + CRC     */
#define Z72_BLK_STATUS       M_DEV_BLK_OF+0x05  /**< G,S: read/write status  */
/**@}*/

/**
 * @defgroup _Z72_ERRCODES Z72_OWB driver error codes */
/** @{ */
#define Z72_ERR_NO_DEVICE	(ERR_DEV+1)	/*!< no device answered              */
#define Z72_ERR_NOT_IDLE	(ERR_DEV+2)	/*!< bus not in idle state           */
#define Z72_ERR_BUSY		(ERR_DEV+3)	/*!< bus is busy                     */
#define Z72_ERR_NO_ROMFKT	(ERR_DEV+4)	/*!< no ROM fkt before memory access */
#define Z72_ERR_PGM			(ERR_DEV+5)	/*!< timing err during program phase */
#define Z72_ERR_BEYOND_MEM	(ERR_DEV+6)	/*!< access beyond mem region        */
#define Z72_ERR_CRC			(ERR_DEV+7)	/*!< CRC error                       */
#define Z72_ERR_UNDEF_STATE	(ERR_DEV+8)	/*!< CRC error                       */
#define Z72_ERR_DEV_UNKNOWN	(ERR_DEV+9)	/*!< connected device not known      */
/** @} */

/*-----------------------------------------+
|  PROTOTYPES                              |
+-----------------------------------------*/
#ifdef _LL_DRV_
#	ifdef _ONE_NAMESPACE_PER_DRIVER_
#		define Z72_GetEntry LL_GetEntry
#	else
#		ifdef Z72_SW
#			define Z72_GetEntry Z72_SW_GetEntry
#		endif /* Z72_SW */
		extern void Z72_GetEntry(LL_ENTRY* drvP);
#	endif /* _ONE_NAMESPACE_PER_DRIVER_ */
#endif /* _LL_DRV_ */

/*-----------------------------------------+
|  BACKWARD COMPATIBILITY TO MDIS4         |
+-----------------------------------------*/
#ifndef U_INT32_OR_64
 /* we have an MDIS4 men_types.h and mdis_api.h included */
 /* only 32bit compatibility needed!                     */
 #define INT32_OR_64  int32
 #define U_INT32_OR_64 u_int32
 typedef INT32_OR_64  MDIS_PATH;
#endif /* U_INT32_OR_64 */
#ifdef __cplusplus
      }
#endif

#endif /* _Z72_OWB_DRV_H */


