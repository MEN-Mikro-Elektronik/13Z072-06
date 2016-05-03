#************************** MDIS5 device descriptor *************************
#
#        Author: cs
#         $Date: 2010/03/12 14:20:34 $
#     $Revision: 1.2 $
#
#   Description: Metadescriptor for Z72
#
#****************************************************************************

owb_1  {
	#------------------------------------------------------------------------
	#	general parameters (don't modify)
	#------------------------------------------------------------------------
    DESC_TYPE        = U_INT32  1           # descriptor type (1=device)
    HW_TYPE          = STRING   Z72         # hardware name of device
    _WIZ_MODEL       = STRING   Z072_OWB

	#------------------------------------------------------------------------
	#	reference to base board
	#------------------------------------------------------------------------
    BOARD_NAME       = STRING   fpga        # device name of baseboard
    DEVICE_SLOT      = U_INT32  0           # used slot on baseboard (0..n)

	#------------------------------------------------------------------------
	#	debug levels (optional)
	#   this keys have only effect on debug drivers
	#------------------------------------------------------------------------
    DEBUG_LEVEL      = U_INT32  0xc0008000  # LL driver
    DEBUG_LEVEL_MK   = U_INT32  0xc0008000  # MDIS kernel
    DEBUG_LEVEL_OSS  = U_INT32  0xc0008000  # OSS calls
    DEBUG_LEVEL_DESC = U_INT32  0xc0008000  # DESC calls
    DEBUG_LEVEL_MBUF = U_INT32  0xc0008000  # MBUF calls

	#------------------------------------------------------------------------
	#	device parameters
	#------------------------------------------------------------------------
    IRQ_ENABLE       = U_INT32  0           # irq enabled after init
    ID_CHECK         = U_INT32  0           # check module ID prom

    # use polling mode to determine when an access is finished -- no interrupts
    # 0 := disabled
    # 1 := enabled
    POLL_MODE = U_INT32 1

    # perform skip ROM request if IDLE and a memory function is requested
    # 0 := disabled
    # 1 := enabled
    AUTO_SKIPROM = U_INT32 1
}
