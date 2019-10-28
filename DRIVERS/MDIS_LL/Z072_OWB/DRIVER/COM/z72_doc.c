/*********************  P r o g r a m  -  M o d u l e ***********************/
/*!
 *        \file  z72_doc.c
 *
 *      \author  cs
 *        $Date: 2006/06/02 16:40:15 $
 *    $Revision: 1.1 $
 *
 *      \brief   User documentation for Z72_OWB module driver
 *
 *     Required: -
 *
 *     \switches -
 *
 *
 *---------------------------------------------------------------------------
 * Copyright 2006-2019, MEN Mikro Elektronik GmbH
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

/*! \mainpage
    This is the documentation of the MDIS low-level driver for the Z72 module.

    The Z72_OWB is a general OneWireBus controller only managing the HW access
    to the OWBus.\n
    This driver primarily supports the Dallas DS2502 device which has the
    mandatory ROM section and an aditional 1024 Bit EPROM.

    In this first version of the driver only read functionality is supported.
    Also the driver only supports ROM functions for busses with only 1 device
    (read ROM, skip ROM).
    The driver is prepared to also support write functionality.

    Supported features:
    	- read ROM, skip ROM
        - read memory, read memory & generate CRC
        - read status

	The buffers passed to and from the single functions are read and written
	byte by byte.

    \n
    \section Variants Variants
     ( no variants )

    \n \section FuncDesc Functional Description

    \n \subsection General General
	No HW initialization is done by the driver. All access to the OWBus can be
	initialized by means of M_getstat/M_setstat calls.

	If enabled, the driver performs a skip ROM function if a memory function
	is called before a ROM function was performed.

    \n \section interrupts Interrupts
    The driver supports interrupts from the FPGA-Module. The Module’s interrupt
    can be enabled/disabled through the M_MK_IRQ_ENABLE SetStat code or the
    device descriptor.

	Occuring interrupts may inform the driver of terminated bus accesses.
	If no interrupts are enabled the polling mode is set up either by
	the M_setstat call or the device descriptor or the M_MK_IRQ_ENABLE SetStat
	with the argument "0". In this case the driver will	poll the device after
	a certain time (defined internally in the driver) for the status.

    \n \section signals Signals
    No signals are supported.

    \n \section api_functions Supported API Functions

    <table border="0">
    <tr>
        <td><b>API function</b></td>
        <td><b>Functionality</b></td>
        <td><b>Corresponding low level function</b></td></tr>

    <tr><td>M_open()</td><td>Open device</td><td>Z72_Init()</td></tr>

    <tr><td>M_close()     </td><td>Close device             </td>
    <td>Z72_Exit())</td></tr>
    <tr><td>M_read()      </td><td>not supported            </td>
    <td>Z72_Read()</td></tr>
    <tr><td>M_write()     </td><td>not supported            </td>
    <td>Z72_Write()</td></tr>
    <tr><td>M_setstat()   </td><td>Set device parameter / Write memory/status </td>
    <td>Z72_SetStat()</td></tr>
    <tr><td>M_getstat()   </td><td>Get device parameter / Read ROM/memory/status </td>
    <td>Z72_GetStat()</td></tr>
    <tr><td>M_getblock()  </td><td>not supported            </td>
    <td>Z72_BlockRead()</td></tr>
    <tr><td>M_setblock()  </td><td>not supported            </td>
    <td>Z72_BlockWrite()</td></tr>
    <tr><td>M_errstringTs() </td><td>Generate error message </td>
    <td>-</td></tr>
    </table>

    \n \section descriptor_entries Descriptor Entries

    The low-level driver initialization routine decodes the following entries
    ("keys") in addition to the general descriptor keys:

    <table border="0">
    <tr><td><b>Descriptor entry</b></td>
        <td><b>Description</b></td>
        <td><b>Values</b></td>
    </tr>

    <tr><td>AUTO_SKIPROM</td>
        <td>perform skip ROM request if IDLE and a memory function is
            requested</td>
        <td>0..1, default: 1</td>
    </tr>
    <tr><td>IRQ_ENABLE</td>
        <td>enable interrupts in LL_Init</td>
        <td>0..1, default: 0</td>
    </tr>
    </table>

    \subsection z72_min   Minimum descriptor
    z72_min.dsc
    Demonstrates the minimum set of options necessary for using the driver.

    \subsection z72_max   Maximum descriptor
    z72_max.dsc
    Shows all possible configuration options for this driver.

    \subsection z72_sw_min   Minimum descriptor (swapped variant)
    z72_sw_min.dsc
    Demonstrates the minimum set of options necessary for using the swapped
    variant of this driver.

    \subsection z72_sw_max   Maximum descriptor (swapped variant)
    z72_sw_max.dsc
    Shows all possible configuration options for the swapped variant of this
    driver.

    \n \section codes Z72 specific Getstat/Setstat codes
    see \ref getstat_setstat_codes "section about Getstat/Setstat codes"

    \n \section programs Overview of provided programs

    \subsection z72_simp  Simple example for using the driver
    z72_simp.c
    \code
    	Options:\n
		    device       device name
		    function     function to perform:
		                 0: simple open/close
		                 1: read ROM section
		                 2: read ROM and memory section
		                 3: read memory section, auto skipRom
		                 4: read memory & generate CRC
		                 5: read status section
		                 6: test auto skipRom function
	\endcode


*/

/** \example z72_simp.c */
/** \example z72_min.dsc */
/** \example z72_max.dsc */
/** \example z72_sw_min.dsc */
/** \example z72_sw_max.dsc */

/*! \page dummy
  \menimages
*/

