@
@ Startup code for Cortex-M3 microcontrollers.
@
@ Copyright (C) 2010 Serge Vakulenko, <serge@vak.ru>
@
@ This file is distributed in the hope that it will be useful, but WITHOUT
@ ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
@ FITNESS FOR A PARTICULAR PURPOSE.
@
@ You can redistribute this file and/or modify it under the terms of the GNU
@ General Public License (GPL) as published by the Free Software Foundation;
@ either version 2 of the License, or (at your discretion) any later version.
@ See the accompanying file "COPYING.txt" for more details.
@
@ As a special exception to the GPL, permission is granted for additional
@ uses of the text contained in this file.  See the accompanying file
@ "COPY-UOS.txt" for details.
@
		.syntax unified

@---------------------------------------
@ Vector table for Cortex-M.
@
		.section .init
		.align	2
		.extern	_init_			@ startup
		.extern	_fault_			@ handle faults and software interrupt
		.extern	_svc_			@ handle supervisor call
		.extern	_irq_handler_		@ handle interrupts and systick

		.thumb
_start_:	.global	_start_
		@
		@ End of user mode stack
		@
		.word  _estack			@ 0

		@
		@ Exception handlers.
		@
		.word	_init_			@ 1 - reset
		.word	_fault_			@ 2 - non-maskable interrupt
		.word	_fault_			@ 3 - hard fault
		.word	_fault_			@ 4 - memory management fault
		.word	_fault_			@ 5 - bus fault
		.word	_fault_			@ 6 - usage fault
		.word	_fault_			@ 7 - reserved
		.word	_fault_			@ 8 - reserved
		.word	_fault_			@ 9 - reserved
		.word	_fault_			@ 10 - reserved
		.word	_svc_			@ 11 - supervisor call
		.word	_fault_			@ 12 - reserved
		.word	_fault_			@ 13 - reserved
		.word	_fault_			@ 14 - software interrupt
		.word	_irq_handler_	@ 15 - system timer tick

		@
		@ IRQs.
		@
		.word	_irq_handler_   @ 16	0		WWDG
		.word	_irq_handler_   @ 17	1		PVD
		.word	_irq_handler_   @ 18	2		TAMPER_STAMP
		.word	_irq_handler_   @ 19	3		RTC_WKUP
		.word	_irq_handler_   @ 20	4		FLASH
		.word	_irq_handler_   @ 21	5		RCC
		.word	_irq_handler_   @ 22	6		EXTI0
		.word	_irq_handler_   @ 23	7		EXTI1
		.word	_irq_handler_   @ 24	8		EXTI2
		.word	_irq_handler_   @ 25	9		EXTI3
		.word	_irq_handler_   @ 26	10		EXTI4
		.word	_irq_handler_   @ 27	11		DMA1_Channel1
		.word	_irq_handler_   @ 28	12		DMA1_Channel2
		.word	_irq_handler_   @ 29	13		DMA1_Channel3
		.word	_irq_handler_   @ 30	14		DMA1_Channel4
		.word	_irq_handler_   @ 31	15		DMA1_Channel5
		.word	_irq_handler_   @ 32	16		DMA1_Channel6
		.word	_irq_handler_   @ 33	17		DMA1_Channel7
		.word	_irq_handler_   @ 34	18		ADC1
		.word	_irq_handler_   @ 35	19		USB_HP
		.word	_irq_handler_   @ 36	20		USB_LP
		.word	_irq_handler_   @ 37	21		DAC
		.word	_irq_handler_   @ 38	22		COMP, TSC
		.word	_irq_handler_   @ 39	23		EXTI9_5
		.word	_irq_handler_   @ 40	24		LCD
		.word	_irq_handler_   @ 41	25		TIM9
		.word	_irq_handler_   @ 42	26		TIM10
		.word	_irq_handler_   @ 43	27		TIM11
		.word	_irq_handler_   @ 44	28		TIM2
		.word	_irq_handler_   @ 45	29		TIM3
		.word	_irq_handler_   @ 46	30		TIM4
		.word	_irq_handler_   @ 47	31		I2C1_EV
		.word	_irq_handler_   @ 48	32		I2C1_ER
		.word	_irq_handler_   @ 49	33		I2C2_EV
		.word	_irq_handler_   @ 50	34		I2C2_ER
		.word	_irq_handler_   @ 51	35		SPI1
		.word	_irq_handler_   @ 52	36		SPI2
		.word	_irq_handler_   @ 53	37		USART1
		.word	_irq_handler_   @ 54	38		USART2
		.word	_irq_handler_   @ 55	39		USART3
		.word	_irq_handler_   @ 56	40		EXTI15_10
		.word	_irq_handler_   @ 57	41		RTC_Alarm
		.word	_irq_handler_   @ 58	42		USB_FS_WKUP
		.word	_irq_handler_   @ 59	43		TIM6
		.word	_irq_handler_   @ 60	44		TIM7
		.word	_irq_handler_   @ 61	45		SDIO
		.word	_irq_handler_   @ 62	46		TIM5
		.word	_irq_handler_   @ 63	47		SPI3
		.word	_irq_handler_   @ 64	48		UART4
		.word	_irq_handler_   @ 65	49		UART5
		.word	_irq_handler_   @ 66	50		DMA2_CH1
		.word	_irq_handler_   @ 67	51		DMA2_CH2
		.word	_irq_handler_   @ 68	52		DMA2_CH3
		.word	_irq_handler_   @ 69	53		DMA2_CH4
		.word	_irq_handler_   @ 70	54		DMA2_CH5
		.word	_irq_handler_   @ 71	55		AES
		.word	_irq_handler_   @ 72	56		COMP_ACQ

