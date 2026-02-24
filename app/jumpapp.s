;******************************************************************************
;* File Name          : JumpApp.s
;* Description        : Jump to application function for STM32F40xxx/41xxx
;*                      This function jumps to the user application at 0x08010000
;******************************************************************************

                PRESERVE8
                THUMB

                AREA    |.text|, CODE, READONLY

; JumpAPP function - Jumps to user application at 0x08010000
JumpApp         PROC
                EXPORT  JumpApp
                ; Load the application start address (0x08010000)
 ;               LDR     R0, =0x08010000
                ; Load the stack pointer from the application's vector table
                LDR     SP, [R0, #0]
                ; Load the reset handler address and jump to it
                LDR     PC, [R0, #4]
                ENDP

                ALIGN

                END
