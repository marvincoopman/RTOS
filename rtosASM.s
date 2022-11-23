; Marvin Coopman

;-----------------------------------------------------------------------------
; Hardware Target
;-----------------------------------------------------------------------------

; Target Platform: EK-TM4C123GXL Evaluation Board
; Target uC:       TM4C123GH6PM
; System Clock:    40 MHz

; Hardware configuration:
; Red LED:
;   PF1 drives an NPN transistor that powers the red LED
; Green LED:
;   PF3 drives an NPN transistor that powers the green LED
; UART Interface:
;   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
;   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
;   Configured to 115,200 baud, 8N1

;-----------------------------------------------------------------------------
; Device includes, defines, and assembler directives
;-----------------------------------------------------------------------------

	.def setASP
	.def setPSP
	.def getPSP
	.def getMSP
	.def pushPSPRegisterOffset
	.def removePriv
	.def pushToPSPStack
	.def popPSPStack
	.def pushDummyPSPStack
    

;-----------------------------------------------------------------------------
; Register values and large immediate values
;-----------------------------------------------------------------------------

.thumb
.const

RETURN_TO_HANDLER_NF       .field   0xFFFFFFFD


;-----------------------------------------------------------------------------
; Subroutines
;-----------------------------------------------------------------------------

.text

setASP:
            MRS  R0, CONTROL
			ORR  R0, #2
			MSR  CONTROL, R0
		   	ISB
			BX   LR

setPSP:
            MSR  PSP,  R0
            ISB
            BX   LR

getPSP:
            MRS  R0,  PSP
            BX   LR
getMSP:
            MRS  R0,  MSP
            BX   LR

pushPSPRegisterOffset:
            MRS R2, PSP
            LSL R0, #2
            ADD R0, R2
            STR R1, [R0]
            BX LR
            
removePriv:
            MRS  R0, CONTROL
            ORR  R0, #1
            MSR  CONTROL, R0
            ISB
            BX   LR

pushToPSPStack:
            MRS  R0,  PSP
            SUB  R0,  R0,  #4
            STR  R11, [R0]
            SUB  R0,  R0,  #4
            STR  R10, [R0]
            SUB  R0,  R0,  #4
            STR  R9,  [R0]
            SUB  R0,  R0,  #4
            STR  R8,  [R0]
            SUB  R0,  R0,  #4
            STR  R7,  [R0]
            SUB  R0,  R0,  #4
            STR  R6,  [R0]
            SUB  R0,  R0,  #4
            STR  R5,  [R0]
            SUB  R0,  R0,  #4
            STR  R4,  [R0]
            MSR  PSP, R0
            BX   LR

popPSPStack:
            MRS  R0,  PSP
            LDR  R4,  [R0]
            ADD  R0,  R0,  #4
            LDR  R5,  [R0]
            ADD  R0,  R0,  #4
            LDR  R6,  [R0]
            ADD  R0,  R0,  #4
            LDR  R7,  [R0]
            ADD  R0,  R0,  #4
            LDR  R8,  [R0]
            ADD  R0,  R0,  #4
            LDR  R9,  [R0]
            ADD  R0,  R0,  #4
            LDR  R10, [R0]
            ADD  R0,  R0,  #4
            LDR  R11, [R0]
            ADD  R0,  R0,  #4
            MSR  PSP, R0
            BX   LR

pushDummyPSPStack:
            MRS  R2,  PSP
            SUB  R2,  R2,  #4
            STR  R0,  [R2]
            SUB  R2,  R2,  #4
            STR  R1,  [R2]
            SUB  R2,  R2,  #4
            LDR  R3,  RETURN_TO_HANDLER_NF
            STR  R3,  [R2]
            MOV  R1,  #5
loop:       SUB  R2,  R2,  #4
            STR  R1,  [R2]
            SUB  R1,  R1,  #1
            CBZ  R1,  exit
            B    loop
exit:    	MSR  PSP, R2
            BX   LR
.end

