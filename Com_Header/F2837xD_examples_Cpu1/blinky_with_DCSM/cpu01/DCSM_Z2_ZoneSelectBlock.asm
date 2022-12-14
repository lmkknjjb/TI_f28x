;// TI File $Revision: /main/2 $
;// Checkin $Date: December 7, 2011   18:25:12 $
;//###########################################################################
;//
;// FILE:	DCSM_Z2_ZoneSelectBlock.asm
;//
;// TITLE:	F2837x Dual Code Security Module Zone 2 Zone Select Block Values.
;// 
;// DESCRIPTION:
;//
;//         This file is used to specify Z2 DCSM OTP and zone select
;//         values to program into the Zone 2 zone select locations in OTP
;//         Value depends on Linkpointer.
;//
;//         In addition, the 60 reserved values after the zone select block 
;//         are all programmed to 0x0000 as well.
;//
;//###########################################################################
;//
;// Original source based on D.A.
;// 
;// $TI Release: F2837xD Support Library v210 $
;// $Release Date: Tue Nov  1 14:46:15 CDT 2016 $
;// $Copyright: Copyright (C) 2013-2016 Texas Instruments Incorporated -
;//             http://www.ti.com/ ALL RIGHTS RESERVED $
;//###########################################################################
;
; !!IMPORTANT!! The below memory sections are mapped to OTP (one-time
; programmable) memory with the *dcsm_lnk.cmd linker command file. In order 
; to program the below memory sections, user should uncomment the .long words
; of each section and change the value to what is desired. Additionally, the
; corresponding section of *dcsm_lnk.cmd should no longer be labelled as a 
; dummy section. Remove ", type = DSECT" in SECTIONS from the memory section
; that is being programmed.
;
; !!IMPORTANT!!: The "dcsm_otp_z2_linkpointer" section contains the 
; Z2 LINKPOINTER which determines the location of the Z2 Zone Select block.  
; If the LINKPOINTER is changed, then the "dcsm_zsel_z2" section in the 
; *_dcsm_lnk_cpu1.cmd command linker file must also change to an address 
; decoded from the value specified in the Z2-LINKPOINTER location. 
;
; The "dcsm_zsel_z2" section contains the actual Z2 Zone Select Block values 
; that will be linked and programmed into to the DCSM Z2 OTP Zone Select block
; in OTP.  
; These values must be known in order to unlock the CSM module. 
; All 0xFFFFFFFF's (erased) is the default value for the password locations (PWL).
;
; It is recommended that all values be left as 0xFFFFFFFF during code
; development.  Values of 0xFFFFFFFF do not activate code security and dummy 
; reads of the Z2 DCSM PWL registers is all that is required to unlock the CSM.  
; When code development is complete, modify values to activate the
; code security module.
;
; ******************************WARNING***************************************
; It is recommended not to program 0xFFFFFFFF to user OTP locations, if users 
; intend to comeback and re-program any of the bits to '0' in future. If user 
; programs 0xFFFFFFFF to any of the OTP locations then the ECC locations would 
; get programmed to a non erased state and users won't be able to comeback 
; and re-program the OTP location to another value. Please refer to DCSM 
; chapter of device TRM for more details on ECC for the locations in DCSM.
;
; Hence TI ships this example commenting out the initialization of all the 
; below locations. 

      .sect "dcsm_otp_z2_linkpointer"
;;    .long 0x1FFFFFFF     ;Z2-LINKPOINTER1
;;    .long 0xFFFFFFFF     ;Reserved
;;    .long 0x1FFFFFFF     ;Z2-LINKPOINTER2
;;    .long 0xFFFFFFFF     ;Reserved
;;    .long 0x1FFFFFFF     ;Z2-LINKPOINTER3
;;    .long 0xFFFFFFFF     ;Reserved

      .sect "dcsm_otp_z2_pswdlock"
;;    .long 0xFFFFFFFF     ;Z2-PSWDLOCK
;;    .long 0xFFFFFFFF     ;Reserved

      .sect "dcsm_otp_z2_crclock"
;;    .long 0xFFFFFFFF     ;Z2-CRCLOCK
;;    .long 0xFFFFFFFF     ;Reserved

      .sect "dcsm_otp_z2_bootctrl"
;;    .long 0xFFFFFFFF     ;Reserved
;;    .long 0xFFFFFFFF     ;Z2-BOOTCTRL

      .sect "dcsm_zsel_z2"
;;    .long 0xFFFFFFFF      ;Z2-EXEONLYRAM
;;    .long 0xFFFFFFFF      ;Z2-EXEONLYSECT
;;    .long 0xFFFFFFFF      ;Z2-GRABRAM
;;    .long 0xFFFFFFFF      ;Z2-GRABSECT

;;    .long 0xFFFFFFFF      ;Z2-CSMPSWD0 (LSW of 128-bit password)
;;    .long 0xFFFFFFFF      ;Z2-CSMPSWD1
;;    .long 0xFFFFFFFF      ;Z2-CSMPSWD2
;;    .long 0xFFFFFFFF      ;Z2-CSMPSWD3 (MSW of 128-bit password)

;----------------------------------------------------------------------

; For code security operation,after development has completed, prior to
; production, all other zone select block locations should be programmed
; to 0x0000 for maximum security.        
; If the first zone select block at offset 0x10 is used, the section 
;"dcsm_rsvd_z2" can be used to program these locations to 0x0000.
; This code is commented out for development.

;        .sect "dcsm_rsvd_z2"
;        .loop (1e0h)
;              .int 0x0000
;        .endloop

;//===========================================================================
;// End of file.
;//===========================================================================

      
