;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000800

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp    


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000800

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                DCD     WDT_IRQHandler            ; 0:  WWDT 
                DCD     SVD_IRQHandler            ; 1:  SVD     
                DCD     RTC_IRQHandler            ; 2:  RTC     
                DCD     FLASH_IRQHandler          ; 3:  FLASH    
                DCD     FDET_IRQHandler           ; 4:  LFDET    
                DCD     ADC_IRQHandler            ; 5:  ADC    
                DCD     DAC_IRQHandler            ; 6:  DAC
                DCD     SPI0_IRQHandler           ; 7:  SPI0     
                DCD     SPI1_IRQHandler           ; 8:  SPI1
                DCD     SPI2_IRQHandler           ; 9:  SPI2    
                DCD     UART0_IRQHandler          ; 10:  UART0    
                DCD     UART1_IRQHandler          ; 11:  UART1        
                DCD     UART3_IRQHandler          ; 12:  UART3    
                DCD     UART4_IRQHandler          ; 13:  UART4    
                DCD     UART5_IRQHandler          ; 14:  UART5        
                DCD     U7816_IRQHandler          ; 15:  U7816    
                DCD     LPUARTx_IRQHandler        ; 16:  LPUART   
                DCD     I2C_IRQHandler            ; 17:  I2C    
                DCD     CCL_IRQHandler            ; 18:  CCL  
                DCD     AES_IRQHandler            ; 19:  AES    
                DCD     LPTIM_IRQHandler          ; 20:  LPTIM    
                DCD     DMA_IRQHandler            ; 21:  DMA    
                DCD     WKUPx_IRQHandler          ; 22:  WKUP    
                DCD     LUT_IRQHandler            ; 23:  LUT   
                DCD     BSTIM_IRQHandler          ; 24:  BSTIM
                DCD     COMPx_IRQHandler          ; 25:  COMPx
                DCD     GPTIM0_1_IRQHandler       ; 26:  GPTIM0_1    
                DCD     GPTIM2_IRQHandler         ; 27:  GPTIM2    
                DCD     ATIM_IRQHandler           ; 28:  ATIM    
                DCD     VREF_IRQHandler           ; 29:  VREF    
                DCD     GPIO_IRQHandler           ; 30:  GPIO
                DCD     CAN_IRQHandler      	  ; 31:  CAN
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main
                LDR     R0, =SystemInit
                BLX     R0        
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC
                EXPORT  CAN_IRQHandler            [WEAK]
                EXPORT  GPIO_IRQHandler           [WEAK]
                EXPORT  VREF_IRQHandler           [WEAK]
                EXPORT  ATIM_IRQHandler           [WEAK]
                EXPORT  GPTIM2_IRQHandler         [WEAK]
                EXPORT  GPTIM0_1_IRQHandler       [WEAK]
                EXPORT  COMPx_IRQHandler          [WEAK]				
                EXPORT  BSTIM_IRQHandler          [WEAK]
                EXPORT  LUT_IRQHandler            [WEAK]    
                EXPORT  WKUPx_IRQHandler          [WEAK]
                EXPORT  DMA_IRQHandler            [WEAK]
                EXPORT  LPTIM_IRQHandler          [WEAK]
                EXPORT  AES_IRQHandler            [WEAK]
                EXPORT  CCL_IRQHandler            [WEAK]
                EXPORT  I2C_IRQHandler            [WEAK]
                EXPORT  LPUARTx_IRQHandler        [WEAK]    
                EXPORT  U7816_IRQHandler          [WEAK]
                EXPORT  UART5_IRQHandler          [WEAK]    
                EXPORT  UART4_IRQHandler          [WEAK]
                EXPORT  UART3_IRQHandler          [WEAK]
                EXPORT  UART1_IRQHandler          [WEAK]
                EXPORT  UART0_IRQHandler          [WEAK]
                EXPORT  SPI2_IRQHandler           [WEAK]
                EXPORT  SPI1_IRQHandler           [WEAK]
                EXPORT  SPI0_IRQHandler           [WEAK]
                EXPORT  DAC_IRQHandler            [WEAK]
                EXPORT  ADC_IRQHandler            [WEAK]
                EXPORT  FDET_IRQHandler           [WEAK]
                EXPORT  FLASH_IRQHandler          [WEAK]
                EXPORT  RTC_IRQHandler            [WEAK]
                EXPORT  SVD_IRQHandler            [WEAK]
                EXPORT  WDT_IRQHandler            [WEAK]

CAN_IRQHandler                    
GPIO_IRQHandler
VREF_IRQHandler
ATIM_IRQHandler
GPTIM2_IRQHandler
GPTIM0_1_IRQHandler
COMPx_IRQHandler
BSTIM_IRQHandler
LUT_IRQHandler
WKUPx_IRQHandler
DMA_IRQHandler
LPTIM_IRQHandler
AES_IRQHandler
CCL_IRQHandler
I2C_IRQHandler
LPUARTx_IRQHandler
U7816_IRQHandler
UART5_IRQHandler
UART4_IRQHandler
UART3_IRQHandler
UART1_IRQHandler
UART0_IRQHandler
SPI2_IRQHandler
SPI1_IRQHandler
SPI0_IRQHandler
DAC_IRQHandler
ADC_IRQHandler
FDET_IRQHandler
FLASH_IRQHandler
RTC_IRQHandler
SVD_IRQHandler
WDT_IRQHandler

                B       .
                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                 LDR     R0, =  Heap_Mem
                 LDR     R1, =(Stack_Mem + Stack_Size)
                 LDR     R2, = (Heap_Mem +  Heap_Size)
                 LDR     R3, = Stack_Mem
                 BX      LR

                ALIGN

                ENDIF


                END
                    
 *****END OF FILE*****