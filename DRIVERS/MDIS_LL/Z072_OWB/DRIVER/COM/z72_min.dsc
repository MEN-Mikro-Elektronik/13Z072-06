#************************** MDIS5 device descriptor *************************
#
#        Author: cs
#         $Date: 2010/03/12 14:20:36 $
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
	#	device parameters
	#------------------------------------------------------------------------
}
