/*
 * Register definitions for STM32F4xx.
 *
 * Copyright (C) 2013-2018 Dmitry Podkhvatilin, <vatilin@gmail.com>
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You can redistribute this file and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software Foundation;
 * either version 2 of the License, or (at your discretion) any later version.
 * See the accompanying file "COPYING.txt" for more details.
 *
 * As a special exception to the GPL, permission is granted for additional
 * uses of the text contained in this file.  See the accompanying file
 * "COPY-UOS.txt" for details.
 */
typedef volatile unsigned int arm_reg_t;

#ifndef AHB_DIV
#   define AHB_DIV     1
#endif

#ifndef APB1_DIV
#   define APB1_DIV    4
#endif

#ifndef APB2_DIV
#   define APB2_DIV    2
#endif


#define ARM_SRAM_BASE       0x20000000  // Internal static memory
#define ARM_CCRAM_BASE      0x10000000  // Internal ccm memory
#define ARM_PERIPH_BASE     0x40000000  // Peripheral registers
#define ARM_SYSTEM_BASE     0xE0000000  // Core registers

#define ARM_SRAM_SIZE       (128*1024)   // Internal SRAM size
#define ARM_CCRAM_SIZE      (64*1024)   // Internal CCM RAM size

#include <runtime/cortex-m/io-cortex-m.h>

//
// Peripheral memory map
//
#define STM_TIM2_BASE           0x40000000
#define STM_TIM3_BASE           0x40000400
#define STM_TIM4_BASE           0x40000800
#define STM_TIM5_BASE           0x40000C00
#define STM_TIM6_BASE           0x40001000
#define STM_TIM7_BASE           0x40001400
#define STM_TIM12_BASE          0x40001800
#define STM_TIM13_BASE          0x40001C00
#define STM_TIM14_BASE          0x40002000
#define STM_RTC_BKP_BASE        0x40002800
#define STM_WWDG_BASE           0x40002C00
#define STM_IWDG_BASE           0x40003000
#define STM_I2S2EXT_BASE        0x40003400
#define STM_SPI2_I2S2_BASE      0x40003800
#define STM_SPI3_I2S3_BASE      0x40003C00
#define STM_I2S3EXT_BASE        0x40004000
#define STM_USART2_BASE         0x40004400
#define STM_USART3_BASE         0x40004800
#define STM_UART4_BASE          0x40004C00
#define STM_UART5_BASE          0x40005000
#define STM_I2C1_BASE           0x40005400
#define STM_I2C2_BASE           0x40005800
#define STM_I2C3_BASE           0x40005C00
#define STM_CAN1_BASE           0x40006400
#define STM_CAN2_BASE           0x40006800
#define STM_PWR_BASE            0x40007000
#define STM_DAC_BASE            0x40007400
#define STM_UART7_BASE          0x40007800
#define STM_UART8_BASE          0x40007C00
#define STM_TIM1_BASE           0x40010000
#define STM_TIM8_BASE           0x40010400
#define STM_USART1_BASE         0x40011000
#define STM_USART6_BASE         0x40011400
#define STM_ADC1_BASE           0x40012000
#define STM_ADC2_BASE           0x40012100
#define STM_ADC3_BASE           0x40012200
#define STM_ADC_COM_BASE        0x40012300
#define STM_SDIO_BASE           0x40012C00
#define STM_SPI1_BASE           0x40013000
#define STM_SPI4_BASE           0x40013400
#define STM_SYSCFG_BASE         0x40013800
#define STM_EXTI_BASE           0x40013C00
#define STM_TIM9_BASE           0x40014000
#define STM_TIM10_BASE          0x40014400
#define STM_TIM11_BASE          0x40014800
#define STM_SPI5_BASE           0x40015000
#define STM_SPI6_BASE           0x40015400
#define STM_GPIOA_BASE          0x40020000
#define STM_GPIOB_BASE          0x40020400
#define STM_GPIOC_BASE          0x40020800
#define STM_GPIOD_BASE          0x40020C00
#define STM_GPIOE_BASE          0x40021000
#define STM_GPIOF_BASE          0x40021400
#define STM_GPIOG_BASE          0x40021800
#define STM_GPIOH_BASE          0x40021C00
#define STM_GPIOI_BASE          0x40022000
#define STM_GPIOJ_BASE          0x40022400
#define STM_GPIOK_BASE          0x40022800
#define STM_CRC_BASE            0x40023000
#define STM_RCC_BASE            0x40023800
#define STM_FLASH_IFACE_BASE    0x40023C00
#define STM_BKPSRAM_BASE        0x40024000
#define STM_DMA1_BASE           0x40026000
#define STM_DMA2_BASE           0x40026400
#define STM_ETH_BASE            0x40028000
#define STM_USB_OTG_HS_BASE     0x40040000
#define STM_USB_OTG_FS_BASE     0x50000000
#define STM_DCMI_BASE           0x50050000
#define STM_CRYP_BASE           0x50060000
#define STM_HASH_BASE           0x50060400
#define STM_RNG_BASE            0x50060800
#define STM_FSMC_BASE           0xA0000000


typedef enum
{
    IRQ_WWDG,
    IRQ_PVD,
    IRQ_TAMPER_STAMP,
    IRQ_RTC_WKUP,
    IRQ_FLASH,
    IRQ_RCC,
    IRQ_EXTI0,
    IRQ_EXTI1,
    IRQ_EXTI2,
    IRQ_EXTI3,
    IRQ_EXTI4,
    IRQ_DMA1_STREAM0,
    IRQ_DMA1_STREAM1,
    IRQ_DMA1_STREAM2,
    IRQ_DMA1_STREAM3,
    IRQ_DMA1_STREAM4,
    IRQ_DMA1_STREAM5,
    IRQ_DMA1_STREAM6,
    IRQ_ADC,
    IRQ_CAN1_TX,
    IRQ_CAN1_RX0,
    IRQ_CAN1_RX1,
    IRQ_CAN1_SCE,
    IRQ_EXTI9_5,
    IRQ_TIM1_BRK_TIM9,
    IRQ_TIM1_UP_TIM10,
    IRQ_TIM1_TRG_COM_TIM11,
    IRQ_TIM1_CC,
    IRQ_TIM2,
    IRQ_TIM3,
    IRQ_TIM4,
    IRQ_I2C1_EV,
    IRQ_I2C1_ER,
    IRQ_I2C2_EV,
    IRQ_I2C2_ER,
    IRQ_SPI1,
    IRQ_SPI2,
    IRQ_USART1,
    IRQ_USART2,
    IRQ_USART3,
    IRQ_EXTI15_10,
    IRQ_RTC_ALARM,
    IRQ_OTG_FS_WKUP,
    IRQ_TIM8_BRK_TIM12,
    IRQ_TIM8_UP_TIM13,
    IRQ_TIM8_TRG_COM_TIM14,
    IRQ_TIM8_CC,
    IRQ_DMA1_STREAM7,
    IRQ_FSMC,
    IRQ_SDIO,
    IRQ_TIM5,
    IRQ_SPI3,
    IRQ_UART4,
    IRQ_UART5,
    IRQ_TIM6_DAC,
    IRQ_TIM7,
    IRQ_DMA2_STREAM0,
    IRQ_DMA2_STREAM1,
    IRQ_DMA2_STREAM2,
    IRQ_DMA2_STREAM3,
    IRQ_DMA2_STREAM4,
    IRQ_ETH,
    IRQ_ETH_WKUP,
    IRQ_CAN2_TX,
    IRQ_CAN2_RX0,
    IRQ_CAN2_RX1,
    IRQ_CAN2_SCE,
    IRQ_OTG_FS,
    IRQ_DMA2_STREAM5,
    IRQ_DMA2_STREAM6,
    IRQ_DMA2_STREAM7,
    IRQ_USART6,
    IRQ_I2C3_EV,
    IRQ_I2C3_ER,
    IRQ_HS_EP1_OUT,
    IRQ_HS_EP1_IN,
    IRQ_OTG_HS_WKUP,
    IRQ_OTG_HS,
    IRQ_DCMI,
    IRQ_CRYP,
    IRQ_HASH_RNG,
    IRQ_FPU,
#if defined(ARM_STM32F42xxx) || defined(ARM_STM32F43xxx)
    IRQ_UART7,
    IRQ_UART8,
    IRQ_SPI4,
    IRQ_SPI5,
    IRQ_SPI6,
#endif
    ARCH_TIMER_IRQ,
    ARCH_INTERRUPTS
} IRQn_t;


/////////////////////////////////////////
// External interrupt/event controller
/////////////////////////////////////////
typedef struct
{
    arm_reg_t IMR;      // Interrupt mask
    arm_reg_t EMR;      // Event mask
    arm_reg_t RTSR;     // Rising edge trigger selection
    arm_reg_t FTSR;     // Falling edge trigger selection
    arm_reg_t SWIER;    // Software interrupt event
    arm_reg_t PR;       // Pending
} EXTI_t;

#define EXTI     ((EXTI_t *) STM_EXTI_BASE)

// EXTI lines
#define EXTI_GPIO(n)            (1 << (n))
#define EXTI_PVD                (1 << 16)
#define EXTI_RTC_ALARM          (1 << 17)
#define EXTI_USB_OTG_FS_WKUP    (1 << 18)
#define EXTI_USB_ETH_WKUP       (1 << 19)
#define EXTI_USB_OTG_HS_WKUP    (1 << 20)
#define EXTI_RTC_TAMP_TS        (1 << 21)
#define EXTI_RTC_WKUP           (1 << 22)


/////////////////////////////////////////
// Power control
/////////////////////////////////////////
typedef struct
{
    arm_reg_t CR;       // Power control
    arm_reg_t CSR;      // Power control/status
} PWR_t;

#define PWR     ((PWR_t *) STM_PWR_BASE)

// CR bits
#define PWR_UDEN_DIS        (0 << 18)
#define PWR_UDEN_EN         (3 << 18)
#define PWR_ODSWEN          (1 << 17)
#define PWR_ODEN            (1 << 16)
#define PWR_VOS_SCALE3      (1 << 14)
#define PWR_VOS_SCALE2      (2 << 14)
#define PWR_VOS_SCALE1      (3 << 14)
#define PWR_VOS_MASK        (3 << 14)
#define PWR_ADCDC1          (1 << 13)
#define PWR_MRUDS           (1 << 11)
#define PWR_LPUDS           (1 << 10)
#define PWR_FPDS            (1 << 9)
#define PWR_DBP             (1 << 8)
#define PWR_PLS_2_0         (0 << 5)
#define PWR_PLS_2_1         (1 << 5)
#define PWR_PLS_2_3         (2 << 5)
#define PWR_PLS_2_5         (3 << 5)
#define PWR_PLS_2_6         (4 << 5)
#define PWR_PLS_2_7         (5 << 5)
#define PWR_PLS_2_8         (6 << 5)
#define PWR_PLS_2_9         (7 << 5)
#define PWR_PLS_MASK        (7 << 5)
#define PWR_PVDE            (1 << 4)
#define PWR_CSBF            (1 << 3)
#define PWR_CWUF            (1 << 2)
#define PWR_PDDS            (1 << 1)
#define PWR_LPSD            (1 << 0)

// CSR bits
#define PWR_UDRDY_EN        (3 << 18)
#define PWR_UDRDY_DIS       (0 << 18)
#define PWR_ODSWRDY         (1 << 17)
#define PWR_ODRDY           (1 << 16)
#define PWR_VOSRDY          (1 << 14)
#define PWR_BRE             (1 << 9)
#define PWR_EWUP            (1 << 8)
#define PWR_BRR             (1 << 3)
#define PWR_PVDO            (1 << 2)
#define PWR_SBF             (1 << 1)
#define PWR_WUF             (1 << 0)


/////////////////////////////////////////
// SYSCFG
/////////////////////////////////////////
typedef struct
{
    arm_reg_t MEMRMP;   // memory remap
    arm_reg_t PMC;      // peripheral mode configuration
    arm_reg_t EXTICR1;  // external interrupt configuration 1
    arm_reg_t EXTICR2;  // external interrupt configuration 2
    arm_reg_t EXTICR3;  // external interrupt configuration 3
    arm_reg_t EXTICR4;  // external interrupt configuration 4
    arm_reg_t CMPCR;    // compensation cell control
} SYSCFG_t;

#define SYSCFG      ((SYSCFG_t*) STM_SYSCFG_BASE)

// SYSCFG_MEMRMP bits
#define SYSCFG_SWP_FMC_NO_SWAP      (0 << 10)
#define SYSCFG_SWP_FMC_SDRAM_NAND   (1 << 10)
#define SYSCFG_FB_MODE              (1 << 8)
#define SYSCFG_MEM_MAIN_FLASH       (0 << 0)
#define SYSCFG_MEM_SYSTEM_FLASH     (1 << 0)
#define SYSCFG_MEM_FSMC             (2 << 0)
#define SYSCFG_MEM_SRAM             (3 << 0)

// SYSCFG_PMC bits
#define SYSCFG_MII_RMII_SEL         (1 << 23)
#define SYSCFG_ADCxDC2(x)           ((x) << 16)

// SYSCFG_EXTICRx bits
#define SYSCFG_PA(x)                (0 << (((x) & 3) * 4))
#define SYSCFG_PB(x)                (1 << (((x) & 3) * 4))
#define SYSCFG_PC(x)                (2 << (((x) & 3) * 4))
#define SYSCFG_PD(x)                (3 << (((x) & 3) * 4))
#define SYSCFG_PE(x)                (4 << (((x) & 3) * 4))
#define SYSCFG_PF(x)                (5 << (((x) & 3) * 4))
#define SYSCFG_PG(x)                (6 << (((x) & 3) * 4))
#define SYSCFG_PH(x)                (7 << (((x) & 3) * 4))
#define SYSCFG_PI(x)                (8 << (((x) & 3) * 4))
#define SYSCFG_PJ(x)                (9 << (((x) & 3) * 4))
#define SYSCFG_PK(x)                (10 << (((x) & 3) * 4))
#define SYSCFG_PMASK(x)             (0xF << (((x) & 3) * 4))

// SYSCFG_CMPCR bits
#define SYSCFG_READY                (1 << 8)
#define SYSCFG_CMP_PD(x)            (1 << 0)


/////////////////////////////////////////
// Reset and clock control
/////////////////////////////////////////
typedef struct
{
    arm_reg_t CR;       // Clock control
    arm_reg_t PLLCFGR;  // PLL configuration
    arm_reg_t CFGR;     // Clock configuration
    arm_reg_t CIR;      // Clock interrupt
    arm_reg_t AHB1RSTR; // AHB1 peripheral reset
    arm_reg_t AHB2RSTR; // AHB2 peripheral reset
    arm_reg_t AHB3RSTR; // AHB3 peripheral reset
    arm_reg_t reserve0;
    arm_reg_t APB1RSTR; // APB1 peripheral reset
    arm_reg_t APB2RSTR; // APB2 peripheral reset
    arm_reg_t reserve1[2];
    arm_reg_t AHB1ENR;  // AHB1 peripheral clock enable
    arm_reg_t AHB2ENR;  // AHB2 peripheral clock enable
    arm_reg_t AHB3ENR;  // AHB3 peripheral clock enable
    arm_reg_t reserve2;
    arm_reg_t APB1ENR;  // APB1 peripheral clock enable
    arm_reg_t APB2ENR;  // APB2 peripheral clock enable
    arm_reg_t reserve3[2];
    arm_reg_t AHB1LPENR;// AHB1 peripheral clock enable in low power mode
    arm_reg_t AHB2LPENR;// AHB2 peripheral clock enable in low power mode
    arm_reg_t AHB3LPENR;// AHB3 peripheral clock enable in low power mode
    arm_reg_t reserve4;
    arm_reg_t APB1LPENR;// APB1 peripheral clock enable in low power mode
    arm_reg_t APB2LPENR;// APB2 peripheral clock enable in low power mode
    arm_reg_t reserve5[2];
    arm_reg_t BDCR;     // Backup domain control
    arm_reg_t CSR;      // Clock control & status
    arm_reg_t reserve6[2];
    arm_reg_t SSCGR;    // Spread spectrum clock generation
    arm_reg_t PLLI2SCFGR; // PLL I2S configuration
    arm_reg_t PLLSAICFGR; // PLL SAI configuration
    arm_reg_t DCKCFGR;    // Dedicated clock configuration
} RCC_t;

#define RCC     ((RCC_t *) STM_RCC_BASE)

// CR bits
#define RCC_PLLSAIRDY           (1 << 29)
#define RCC_PLLSAION            (1 << 28)
#define RCC_PLLI2SRDY           (1 << 27)
#define RCC_PLLI2SON            (1 << 26)
#define RCC_PLLRDY              (1 << 25)
#define RCC_PLLON               (1 << 24)
#define RCC_CSSON               (1 << 19)
#define RCC_HSEBYP              (1 << 18)
#define RCC_HSERDY              (1 << 17)
#define RCC_HSEON               (1 << 16)
#define RCC_HSICAL(n)           ((n) << 8)
#define RCC_HSITRIM(n)          ((n) << 3)
#define RCC_HSIRDY              (1 << 1)
#define RCC_HSION               (1 << 0)

// PLLCFGR bits
#define RCC_PLLQ(n)             ((n) << 24)
#define RCC_PLLSRC_HSE          (1 << 22)
#define RCC_PLLP_2              (0 << 16)
#define RCC_PLLP_4              (1 << 16)
#define RCC_PLLP_6              (2 << 16)
#define RCC_PLLP_8              (3 << 16)
#define RCC_PLLN(n)             ((n) << 6)
#define RCC_PLLM(n)             (n)

// CFGR bits
#define RCC_MCO2_SYSCLK         (0 << 30)
#define RCC_MCO2_PLLI2S         (1 << 30)
#define RCC_MCO2_HSE            (2 << 30)
#define RCC_MCO2_PLL            (3 << 30)
#define RCC_MCO2PRE_NODIV       (0 << 27)
#define RCC_MCO2PRE_DIV2        (4 << 27)
#define RCC_MCO2PRE_DIV3        (5 << 27)
#define RCC_MCO2PRE_DIV4        (6 << 27)
#define RCC_MCO2PRE_DIV5        (7 << 27)
#define RCC_MCO1PRE_NODIV       (0 << 24)
#define RCC_MCO1PRE_DIV2        (4 << 24)
#define RCC_MCO1PRE_DIV3        (5 << 24)
#define RCC_MCO1PRE_DIV4        (6 << 24)
#define RCC_MCO1PRE_DIV5        (7 << 24)
#define RCC_I2SSRC_EXT          (1 << 23)
#define RCC_MCO1_HSI            (0 << 21)
#define RCC_MCO1_LSE            (1 << 21)
#define RCC_MCO1_HSE            (2 << 21)
#define RCC_MCO1_PLL            (3 << 21)
#define RCC_RTCPRE              ((n) << 16)
#define RCC_PPRE2_NODIV         (0 << 13)
#define RCC_PPRE2_DIV2          (4 << 13)
#define RCC_PPRE2_DIV4          (5 << 13)
#define RCC_PPRE2_DIV8          (6 << 13)
#define RCC_PPRE2_DIV16         (7 << 13)
#define RCC_PPRE1_NODIV         (0 << 10)
#define RCC_PPRE1_DIV2          (4 << 10)
#define RCC_PPRE1_DIV4          (5 << 10)
#define RCC_PPRE1_DIV8          (6 << 10)
#define RCC_PPRE1_DIV16         (7 << 10)
#define RCC_HPRE_NODIV          (0 << 4)
#define RCC_HPRE_DIV2           (0x8 << 4)
#define RCC_HPRE_DIV4           (0x9 << 4)
#define RCC_HPRE_DIV8           (0xA << 4)
#define RCC_HPRE_DIV16          (0xB << 4)
#define RCC_HPRE_DIV64          (0xC << 4)
#define RCC_HPRE_DIV128         (0xD << 4)
#define RCC_HPRE_DIV256         (0xE << 4)
#define RCC_HPRE_DIV512         (0xF << 4)
#define RCC_SWS_HSI             (0 << 2)
#define RCC_SWS_HSE             (1 << 2)
#define RCC_SWS_PLL             (2 << 2)
#define RCC_SWS_MASK            (3 << 2)
#define RCC_SW_HSI              (0 << 0)
#define RCC_SW_HSE              (1 << 0)
#define RCC_SW_PLL              (2 << 0)
#define RCC_SW_MASK             (3 << 0)

// CIR bits
#define RCC_CSSC                (1 << 23)
#define RCC_PLLSAIRDYC          (1 << 22)
#define RCC_PLLI2SRDYC          (1 << 21)
#define RCC_PLLRDYC             (1 << 20)
#define RCC_HSERDYC             (1 << 19)
#define RCC_HSIRDYC             (1 << 18)
#define RCC_LSERDYC             (1 << 17)
#define RCC_LSIRDYC             (1 << 16)
#define RCC_PLLSAIRDYIE         (1 << 14)
#define RCC_PLLI2SRDYIE         (1 << 13)
#define RCC_PLLRDYIE            (1 << 12)
#define RCC_HSERDYIE            (1 << 11)
#define RCC_HSIRDYIE            (1 << 10)
#define RCC_LSERDYIE            (1 << 9)
#define RCC_LSIRDYIE            (1 << 8)
#define RCC_CSSF                (1 << 7)
#define RCC_PLLSAIRDYF          (1 << 6)
#define RCC_PLLI2SRDYF          (1 << 5)
#define RCC_PLLRDYF             (1 << 4)
#define RCC_HSERDYF             (1 << 3)
#define RCC_HSIRDYF             (1 << 2)
#define RCC_LSERDYF             (1 << 1)
#define RCC_LSIRDYF             (1 << 0)

// AHB1RSTR bits
#define RCC_OTGHSRST            (1 << 29)
#define RCC_ETHMACRST           (1 << 25)
#define RCC_DMA2DRST            (1 << 23)
#define RCC_DMA2RST             (1 << 22)
#define RCC_DMA1RST             (1 << 21)
#define RCC_CRCRST              (1 << 12)
#define RCC_GPIOKRST            (1 << 10)
#define RCC_GPIOJRST            (1 << 9)
#define RCC_GPIOIRST            (1 << 8)
#define RCC_GPIOHRST            (1 << 7)
#define RCC_GPIOGRST            (1 << 6)
#define RCC_GPIOFRST            (1 << 5)
#define RCC_GPIOERST            (1 << 4)
#define RCC_GPIODRST            (1 << 3)
#define RCC_GPIOCRST            (1 << 2)
#define RCC_GPIOBRST            (1 << 1)
#define RCC_GPIOARST            (1 << 0)

// AHB2RSTR bits
#define RCC_OTGFSRST            (1 << 7)
#define RCC_RNGRST              (1 << 6)
#define RCC_HASHRST             (1 << 5)
#define RCC_CRYPRST             (1 << 4)
#define RCC_DCMIRST             (1 << 0)

// AHB3RSTR bits
#define RCC_FSMCRST             (1 << 0)

// APB1RSTR bits
#define RCC_UART8RST            (1 << 31)
#define RCC_UART7RST            (1 << 30)
#define RCC_DACRST              (1 << 29)
#define RCC_PWRRST              (1 << 28)
#define RCC_CAN2RST             (1 << 26)
#define RCC_CAN1RST             (1 << 25)
#define RCC_I2C3RST             (1 << 23)
#define RCC_I2C2RST             (1 << 22)
#define RCC_I2C1RST             (1 << 21)
#define RCC_UART5RST            (1 << 20)
#define RCC_UART4RST            (1 << 19)
#define RCC_USART3RST           (1 << 18)
#define RCC_USART2RST           (1 << 17)
#define RCC_SPI3RST             (1 << 15)
#define RCC_SPI2RST             (1 << 14)
#define RCC_WWDGRST             (1 << 11)
#define RCC_TIM14RST            (1 << 8)
#define RCC_TIM13RST            (1 << 7)
#define RCC_TIM12RST            (1 << 6)
#define RCC_TIM7RST             (1 << 5)
#define RCC_TIM6RST             (1 << 4)
#define RCC_TIM5RST             (1 << 3)
#define RCC_TIM4RST             (1 << 2)
#define RCC_TIM3RST             (1 << 1)
#define RCC_TIM2RST             (1 << 0)

// APB2RSTR bits
#define RCC_LTDCRST             (1 << 26)
#define RCC_SAI1RST             (1 << 22)
#define RCC_SPI6RST             (1 << 21)
#define RCC_SPI5RST             (1 << 20)
#define RCC_TIM11RST            (1 << 18)
#define RCC_TIM10RST            (1 << 17)
#define RCC_TIM9RST             (1 << 16)
#define RCC_SYSCFGRST           (1 << 14)
#define RCC_SPI4RST             (1 << 13)
#define RCC_SPI1RST             (1 << 12)
#define RCC_SDIORST             (1 << 11)
#define RCC_ADC3RST             (1 << 10)
#define RCC_ADC2RST             (1 << 9)
#define RCC_ADC1RST             (1 << 8)
#define RCC_USART6RST           (1 << 5)
#define RCC_USART1RST           (1 << 4)
#define RCC_TIM8RST             (1 << 1)
#define RCC_TIM1RST             (1 << 0)

// AHB1ENR bits
#define RCC_OTGHSULPIEN         (1 << 30)
#define RCC_OTGHSEN             (1 << 29)
#define RCC_ETHMACPTPEN         (1 << 28)
#define RCC_ETHMACRXEN          (1 << 27)
#define RCC_ETHMACTXEN          (1 << 26)
#define RCC_ETHMACEN            (1 << 25)
#define RCC_DMA2DEN             (1 << 23)
#define RCC_DMA2EN              (1 << 22)
#define RCC_DMA1EN              (1 << 21)
#define RCC_CCMDATARAMEN        (1 << 20)
#define RCC_BKPSRAMEN           (1 << 18)
#define RCC_CRCEN               (1 << 12)
#define RCC_GPIOKEN             (1 << 10)
#define RCC_GPIOJEN             (1 << 9)
#define RCC_GPIOIEN             (1 << 8)
#define RCC_GPIOHEN             (1 << 7)
#define RCC_GPIOGEN             (1 << 6)
#define RCC_GPIOFEN             (1 << 5)
#define RCC_GPIOEEN             (1 << 4)
#define RCC_GPIODEN             (1 << 3)
#define RCC_GPIOCEN             (1 << 2)
#define RCC_GPIOBEN             (1 << 1)
#define RCC_GPIOAEN             (1 << 0)

// AHB2ENR bits
#define RCC_OTGFSEN             (1 << 7)
#define RCC_RNGEN               (1 << 6)
#define RCC_HASHEN              (1 << 5)
#define RCC_CRYPEN              (1 << 4)
#define RCC_DCMIEN              (1 << 0)

// AHB3ENR bits
#define RCC_FSMCEN              (1 << 0)

// APB1ENR bits
#define RCC_UART8EN             (1 << 31)
#define RCC_UART7EN             (1 << 30)
#define RCC_DACEN               (1 << 29)
#define RCC_PWREN               (1 << 28)
#define RCC_CAN2EN              (1 << 26)
#define RCC_CAN1EN              (1 << 25)
#define RCC_I2C3EN              (1 << 23)
#define RCC_I2C2EN              (1 << 22)
#define RCC_I2C1EN              (1 << 21)
#define RCC_UART5EN             (1 << 20)
#define RCC_UART4EN             (1 << 19)
#define RCC_USART3EN            (1 << 18)
#define RCC_USART2EN            (1 << 17)
#define RCC_SPI3EN              (1 << 15)
#define RCC_SPI2EN              (1 << 14)
#define RCC_WWDGEN              (1 << 11)
#define RCC_TIM14EN             (1 << 8)
#define RCC_TIM13EN             (1 << 7)
#define RCC_TIM12EN             (1 << 6)
#define RCC_TIM7EN              (1 << 5)
#define RCC_TIM6EN              (1 << 4)
#define RCC_TIM5EN              (1 << 3)
#define RCC_TIM4EN              (1 << 2)
#define RCC_TIM3EN              (1 << 1)
#define RCC_TIM2EN              (1 << 0)

// APB2ENR bits
#define RCC_LTDCEN              (1 << 26)
#define RCC_SAI1EN              (1 << 22)
#define RCC_SPI6EN              (1 << 21)
#define RCC_SPI5EN              (1 << 20)
#define RCC_TIM11EN             (1 << 18)
#define RCC_TIM10EN             (1 << 17)
#define RCC_TIM9EN              (1 << 16)
#define RCC_SYSCFGEN            (1 << 14)
#define RCC_SPI4EN              (1 << 13)
#define RCC_SPI1EN              (1 << 12)
#define RCC_SDIOEN              (1 << 11)
#define RCC_ADC3EN              (1 << 10)
#define RCC_ADC2EN              (1 << 9)
#define RCC_ADC1EN              (1 << 8)
#define RCC_USART6EN            (1 << 5)
#define RCC_USART1EN            (1 << 4)
#define RCC_TIM8EN              (1 << 1)
#define RCC_TIM1EN              (1 << 0)

// AHB1LPENR bits
#define RCC_OTGHSULPILPEN       (1 << 30)
#define RCC_OTGHSLPEN           (1 << 29)
#define RCC_ETHMACPTPLPEN       (1 << 28)
#define RCC_ETHMACRXLPEN        (1 << 27)
#define RCC_ETHMACTXLPEN        (1 << 26)
#define RCC_ETHMACLPEN          (1 << 25)
#define RCC_DMA2DLPEN           (1 << 23)
#define RCC_DMA2LPEN            (1 << 22)
#define RCC_DMA1LPEN            (1 << 21)
#define RCC_SRAM3LPEN           (1 << 19)
#define RCC_BKPSRAMLPEN         (1 << 18)
#define RCC_SRAM2LPEN           (1 << 17)
#define RCC_SRAM1LPEN           (1 << 16)
#define RCC_FLITFLPEN           (1 << 15)
#define RCC_CRCLPEN             (1 << 12)
#define RCC_GPIOKLPEN           (1 << 10)
#define RCC_GPIOJLPEN           (1 << 9)
#define RCC_GPIOILPEN           (1 << 8)
#define RCC_GPIOHLPEN           (1 << 7)
#define RCC_GPIOGLPEN           (1 << 6)
#define RCC_GPIOFLPEN           (1 << 5)
#define RCC_GPIOELPEN           (1 << 4)
#define RCC_GPIODLPEN           (1 << 3)
#define RCC_GPIOCLPEN           (1 << 2)
#define RCC_GPIOBLPEN           (1 << 1)
#define RCC_GPIOALPEN           (1 << 0)

// AHB2LPENR bits
#define RCC_OTGFSLPEN           (1 << 7)
#define RCC_RNGLPEN             (1 << 6)
#define RCC_HASHLPEN            (1 << 5)
#define RCC_CRYPLPEN            (1 << 4)
#define RCC_DCMILPEN            (1 << 0)

// AHB3LPENR bits
#define RCC_FSMCLPEN            (1 << 0)

// APB1LPENR bits
#define RCC_UART8LPEN           (1 << 31)
#define RCC_UART7LPEN           (1 << 30)
#define RCC_DACLPEN             (1 << 29)
#define RCC_PWRLPEN             (1 << 28)
#define RCC_CAN2LPEN            (1 << 26)
#define RCC_CAN1LPEN            (1 << 25)
#define RCC_I2C3LPEN            (1 << 23)
#define RCC_I2C2LPEN            (1 << 22)
#define RCC_I2C1LPEN            (1 << 21)
#define RCC_UART5LPEN           (1 << 20)
#define RCC_UART4LPEN           (1 << 19)
#define RCC_USART3LPEN          (1 << 18)
#define RCC_USART2LPEN          (1 << 17)
#define RCC_SPI3LPEN            (1 << 15)
#define RCC_SPI2LPEN            (1 << 14)
#define RCC_WWDGLPEN            (1 << 11)
#define RCC_TIM14LPEN           (1 << 8)
#define RCC_TIM13LPEN           (1 << 7)
#define RCC_TIM12LPEN           (1 << 6)
#define RCC_TIM7LPEN            (1 << 5)
#define RCC_TIM6LPEN            (1 << 4)
#define RCC_TIM5LPEN            (1 << 3)
#define RCC_TIM4LPEN            (1 << 2)
#define RCC_TIM3LPEN            (1 << 1)
#define RCC_TIM2LPEN            (1 << 0)

// APB2LPENR bits
#define RCC_LTDCLPEN            (1 << 26)
#define RCC_SAI1LPEN            (1 << 22)
#define RCC_SPI6LPEN            (1 << 21)
#define RCC_SPI5LPEN            (1 << 20)
#define RCC_TIM11LPEN           (1 << 18)
#define RCC_TIM10LPEN           (1 << 17)
#define RCC_TIM9LPEN            (1 << 16)
#define RCC_SYSCFGLPEN          (1 << 14)
#define RCC_SPI4LPEN            (1 << 13)
#define RCC_SPI1LPEN            (1 << 12)
#define RCC_SDIOLPEN            (1 << 11)
#define RCC_ADC3LPEN            (1 << 10)
#define RCC_ADC2LPEN            (1 << 9)
#define RCC_ADC1LPEN            (1 << 8)
#define RCC_USART6LPEN          (1 << 5)
#define RCC_USART1LPEN          (1 << 4)
#define RCC_TIM8LPEN            (1 << 1)
#define RCC_TIM1LPEN            (1 << 0)

// BDCR bits
#define RCC_BDRST               (1 << 16)
#define RCC_RTCEN               (1 << 15)
#define RCC_RTCSEL_NOCLK        (0 << 8)
#define RCC_RTCSEL_LSE          (1 << 8)
#define RCC_RTCSEL_LSI          (2 << 8)
#define RCC_RTCSEL_HSE          (3 << 8)
#define RCC_LSEBYP              (1 << 2)
#define RCC_LSERDY              (1 << 1)
#define RCC_LSEON               (1 << 0)

// CSR bits
#define RCC_LPWRRSTF            (1 << 31)
#define RCC_WWDGRSTF            (1 << 30)
#define RCC_IWDGRSTF            (1 << 29)
#define RCC_SFTRSTF             (1 << 28)
#define RCC_PORRSTF             (1 << 27)
#define RCC_PINRSTF             (1 << 26)
#define RCC_BORRSTF             (1 << 25)
#define RCC_RMVF                (1 << 24)
#define RCC_LSIRDY              (1 << 1)
#define RCC_LSION               (1 << 0)

// SSCGR bits
#define RCC_SSCGEN              (1 << 31)
#define RCC_SPREADSEL           (1 << 30)
#define RCC_INCSTEP(n)          ((n) << 13)
#define RCC_MODPER(n)           (n)

// PLLI2SCFGR bits
#define RCC_PLLI2SR(n)          ((n) << 28)
#define RCC_PLLI2SQ(n)          ((n) << 24)
#define RCC_PLLI2SN(n)          ((n) << 6)

// PLLSAICFGR bits
#define RCC_PLLSAIR(n)          ((n) << 28)
#define RCC_PLLSAIQ(n)          ((n) << 24)
#define RCC_PLLSAIN(n)          ((n) << 6)

// DCKCFGR bits
#define RCC_TIMPRE              (1 << 24)
#define RCC_SAI1BSRC(n)         ((n) << 22)
#define RCC_SAI1ASRC(n)         ((n) << 20)
#define RCC_PLLSAIDIVR_2        (0 << 16)
#define RCC_PLLSAIDIVR_4        (1 << 16)
#define RCC_PLLSAIDIVR_8        (2 << 16)
#define RCC_PLLSAIDIVR_16       (3 << 16)
#define RCC_PLLSAIDIVQ(n)       (((n)-1) << 8)
#define RCC_PLLI2SDIVQ(n)       (((n)-1) << 0)


/////////////////////////////////////////
// Flash interface
/////////////////////////////////////////
typedef struct
{
    arm_reg_t ACR;
    arm_reg_t KEYR;
    arm_reg_t OPTKEYR;
    arm_reg_t SR;
    arm_reg_t CR;
    arm_reg_t OPTCR;
    arm_reg_t OPTCR1;
} FLASH_t;

#define FLASH                   ((FLASH_t*) STM_FLASH_IFACE_BASE)

// ACR bits
#define FLASH_DCRST             (1 << 12)
#define FLASH_ICRST             (1 << 11)
#define FLASH_DCEN              (1 << 10)
#define FLASH_ICEN              (1 << 9)
#define FLASH_PRFTEN            (1 << 8)
#define FLASH_LATENCY(n)        (n)

// KEYR bits
#define FLASH_KEY1              0x45670123
#define FLASH_KEY2              0xCDEF89AB

// OPTKEYR bits
#define FLASH_OPTKEY1           0x08192A3B
#define FLASH_OPTKEY2           0x4C5D6E7F

// SR bits
#define FLASH_BSY               (1 << 16)
#define FLASH_RDERR             (1 << 7)
#define FLASH_PGSERR            (1 << 7)
#define FLASH_PGPERR            (1 << 6)
#define FLASH_PGAERR            (1 << 5)
#define FLASH_WRPERR            (1 << 4)
#define FLASH_OPERR             (1 << 1)
#define FLASH_EOP               (1 << 0)

// CR bits
#define FLASH_LOCK              (1 << 31)
#define FLASH_ERRIE             (1 << 25)
#define FLASH_EOPIE             (1 << 24)
#define FLASH_STRT              (1 << 16)
#define FLASH_MER1              (1 << 15)
#define FLASH_PSIZE_8           (0 << 8)
#define FLASH_PSIZE_16          (1 << 8)
#define FLASH_PSIZE_32          (2 << 8)
#define FLASH_PSIZE_64          (3 << 8)
#define FLASH_SNB(n)            ((n) << 3)
#define FLASH_MER               (1 << 2)
#define FLASH_SER               (1 << 1)
#define FLASH_PG                (1 << 0)

// OPTCR bits
#define FLASH_SPRMOD            (1 << 31)
#define FLASH_DB1M              (1 << 30)
#define FLASH_NWRP(n)           (1 << (n) << 16)    // also for OPTCR1
#define FLASH_ALL_WRITABLE      (0xFFF << 16)       // also for OPTCR1
#define FLASH_RDP_LEVEL0        (0xAA << 8)
#define FLASH_RDP_LEVEL1        (0x00 << 8)
#define FLASH_RDP_LEVEL2        (0xCC << 8)
#define FLASH_NRST_STDBY        (1 << 7)
#define FLASH_NRST_STOP         (1 << 6)
#define FLASH_WDG_SW            (1 << 5)
#define FLASH_BFB2              (1 << 4)
#define FLASH_BOR_LEV(n)        ((n) << 2)
#define FLASH_OPTSTRT           (1 << 1)
#define FLASH_OPTLOCK           (1 << 0)


/////////////////////////////////////////
// Option bytes
/////////////////////////////////////////

#define OB_RDP_USER     *((volatile uint16_t *) 0x1FFFC000)
#define OB_WRP_LOW      *((volatile uint16_t *) 0x1FFFC008)
#define OB_WRP_HIGH     *((volatile uint16_t *) 0x1FFEC008)

// RDP and USER bits
#define OB_RDP_LEVEL0           (0xAA << 8)
#define OB_RDP_LEVEL1           (0x00 << 8)
#define OB_RDP_LEVEL2           (0xCC << 8)
#define OB_NRST_STDBY           (1 << 7)
#define OB_NRST_STOP            (1 << 6)
#define OB_WDG_SW               (1 << 5)
#define OB_BFB2                 (1 << 4)
#define OB_BOR_LEV(n)           ((n) << 2)

// WRP_LOW bits
#define OB_SPRMOD               (1 << 15)
#define OB_DB1M                 (1 << 14)
#define OB_NWRP(n)              (1 << (n))


/////////////////////////////////////////
// General purpose I/O
/////////////////////////////////////////
typedef struct
{
    arm_reg_t MODER;    // Mode
    arm_reg_t OTYPER;   // Output type
    arm_reg_t OSPEEDR;  // Output speed
    arm_reg_t PUPDR;    // Pull-up/pull-down
    arm_reg_t IDR;      // Input data
    arm_reg_t ODR;      // Output data
    arm_reg_t BSRR;     // Bit set/reset
    arm_reg_t LCKR;     // Configuration lock
    arm_reg_t AFRL;     // Alternate function low register
    arm_reg_t AFRH;     // Alternate function high register
} GPIO_t;

#define GPIOA       ((GPIO_t*) STM_GPIOA_BASE)
#define GPIOB       ((GPIO_t*) STM_GPIOB_BASE)
#define GPIOC       ((GPIO_t*) STM_GPIOC_BASE)
#define GPIOD       ((GPIO_t*) STM_GPIOD_BASE)
#define GPIOE       ((GPIO_t*) STM_GPIOE_BASE)
#define GPIOF       ((GPIO_t*) STM_GPIOF_BASE)
#define GPIOG       ((GPIO_t*) STM_GPIOG_BASE)
#define GPIOH       ((GPIO_t*) STM_GPIOH_BASE)
#define GPIOI       ((GPIO_t*) STM_GPIOI_BASE)
#define GPIOJ       ((GPIO_t*) STM_GPIOJ_BASE)
#define GPIOK       ((GPIO_t*) STM_GPIOK_BASE)

// MODER bits
#define GPIO_IN(n)          (0 << (2 * n))  // Input mode
#define GPIO_OUT(n)         (1 << (2 * n))  // General purpose output mode
#define GPIO_ALT(n)         (2 << (2 * n))  // Alternate function mode
#define GPIO_ANA(n)         (3 << (2 * n))  // Analog mode
#define GPIO_MODE_MASK(n)   (3 << (2 * n))  // Mask

// OTYPER bits
#define GPIO_PP(n)      (0 << (n))      // Push-pull
#define GPIO_OD(n)      (1 << (n))      // Open drain

// OSPEEDR bits
#define GPIO_2MHz(n)        (0 << (2 * n))  // Low speed
#define GPIO_25MHz(n)       (1 << (2 * n))  // Medium speed
#define GPIO_50MHz(n)       (2 << (2 * n))  // Fast speed
#define GPIO_100MHz(n)      (3 << (2 * n))  // High speed
#define GPIO_OSPEED_MASK(n) (3 << (2 * n))  // Field mask

// PUPDR bits
#define GPIO_NO_PULL(n)   (0 << (2 * n))  // No pull-up, no pull-down
#define GPIO_PULL_UP(n)   (1 << (2 * n))  // Pull-up
#define GPIO_PULL_DOWN(n) (2 << (2 * n))  // Pull-down
#define GPIO_PULL_MASK(n) (2 << (2 * (n)))  // Reset


// BSRR bits
#define GPIO_SET(n)     (1 << (n))      // Atomic set n-th bit of ODR
#define GPIO_RESET(n)   (1 << (n + 16)) // Atomic reset n-th bit of ODR

// LCKR bits
#define GPIO_LCK(n)     (1 << (n))      // Pin configuration lock
#define GPIO_LCKK(n)    (1 << 16)       // Port configuration lock key

// Alternate functions (AFRL and AFRH bits)
#define GPIO_AF_SYSTEM(n)   (0x0 << (4 * ((n) & 7)))
#define GPIO_AF_TIM1(n)     (0x1 << (4 * ((n) & 7)))
#define GPIO_AF_TIM2(n)     (0x1 << (4 * ((n) & 7)))
#define GPIO_AF_TIM3(n)     (0x2 << (4 * ((n) & 7)))
#define GPIO_AF_TIM4(n)     (0x2 << (4 * ((n) & 7)))
#define GPIO_AF_TIM5(n)     (0x2 << (4 * ((n) & 7)))
#define GPIO_AF_TIM8(n)     (0x3 << (4 * ((n) & 7)))
#define GPIO_AF_TIM9(n)     (0x3 << (4 * ((n) & 7)))
#define GPIO_AF_TIM10(n)    (0x3 << (4 * ((n) & 7)))
#define GPIO_AF_TIM11(n)    (0x3 << (4 * ((n) & 7)))
#define GPIO_AF_I2C1(n)     (0x4 << (4 * ((n) & 7)))
#define GPIO_AF_I2C2(n)     (0x4 << (4 * ((n) & 7)))
#define GPIO_AF_I2C3(n)     (0x4 << (4 * ((n) & 7)))
#define GPIO_AF_SPI1(n)     (0x5 << (4 * ((n) & 7)))
#define GPIO_AF_SPI2(n)     (0x5 << (4 * ((n) & 7)))
#define GPIO_AF_SPI3(n)     (0x6 << (4 * ((n) & 7)))
#define GPIO_AF_USART1(n)   (0x7 << (4 * ((n) & 7)))
#define GPIO_AF_USART2(n)   (0x7 << (4 * ((n) & 7)))
#define GPIO_AF_USART3(n)   (0x7 << (4 * ((n) & 7)))
#define GPIO_AF_UART4(n)    (0x8 << (4 * ((n) & 7)))
#define GPIO_AF_UART5(n)    (0x8 << (4 * ((n) & 7)))
#define GPIO_AF_USART6(n)   (0x8 << (4 * ((n) & 7)))
#define GPIO_AF_UART7(n)    (0x8 << (4 * ((n) & 7)))
#define GPIO_AF_UART8(n)    (0x8 << (4 * ((n) & 7)))
#define GPIO_AF_CAN1(n)     (0x9 << (4 * ((n) & 7)))
#define GPIO_AF_CAN2(n)     (0x9 << (4 * ((n) & 7)))
#define GPIO_AF_TIM12(n)    (0x9 << (4 * ((n) & 7)))
#define GPIO_AF_TIM13(n)    (0x9 << (4 * ((n) & 7)))
#define GPIO_AF_TIM14(n)    (0x9 << (4 * ((n) & 7)))
#define GPIO_AF_OTG_FS(n)   (0xA << (4 * ((n) & 7)))
#define GPIO_AF_OTG_HS(n)   (0xA << (4 * ((n) & 7)))
#define GPIO_AF_ETH(n)      (0xB << (4 * ((n) & 7)))
#define GPIO_AF_FSMC(n)     (0xC << (4 * ((n) & 7)))
#define GPIO_AF_SDIO(n)     (0xC << (4 * ((n) & 7)))
#define GPIO_AF_OTG_HS_FS(n) (0xC << (4 * ((n) & 7)))
#define GPIO_AF_DCMI(n)     (0xD << (4 * ((n) & 7)))
#define GPIO_AF_EVENTOUT(n) (0xF << (4 * ((n) & 7)))
#define GPIO_AF_MASK(n)     (0xF << (4 * ((n) & 7)))


/////////////////////////////////////////
// USART
/////////////////////////////////////////
typedef struct
{
    arm_reg_t SR;       // Status
    arm_reg_t DR;       // Data
    arm_reg_t BRR;      // Baud rate
    arm_reg_t CR1;      // Control 1
    arm_reg_t CR2;      // Control 2
    arm_reg_t CR3;      // Control 3
    arm_reg_t GTPR;     // Guard time and prescaler
} USART_t;

#define USART1   ((USART_t*) STM_USART1_BASE)
#define USART2   ((USART_t*) STM_USART2_BASE)
#define USART3   ((USART_t*) STM_USART3_BASE)
#define UART4    ((USART_t*) STM_UART4_BASE)
#define UART5    ((USART_t*) STM_UART5_BASE)
#define USART6   ((USART_t*) STM_USART6_BASE)
#define UART7    ((USART_t*) STM_UART7_BASE)
#define UART8    ((USART_t*) STM_UART8_BASE)

// SR bits
#define USART_CTS           (1 << 9)
#define USART_LBD           (1 << 8)
#define USART_TXE           (1 << 7)
#define USART_TC            (1 << 6)
#define USART_RXNE          (1 << 5)
#define USART_IDLE          (1 << 4)
#define USART_ORE           (1 << 3)
#define USART_NF            (1 << 2)
#define USART_FE            (1 << 1)
#define USART_PE            (1 << 0)

// BRR bits
#define USART_DIV_MANTISSA(n)   ((n) << 4)
#define USART_DIV_FRACTION(n)   (n)

// CR1 bits
#define USART_OVER8         (1 << 15)
#define USART_UE            (1 << 13)
#define USART_M             (1 << 12)
#define USART_WAKE          (1 << 11)
#define USART_PCE           (1 << 10)
#define USART_PS            (1 << 9)
#define USART_PEIE          (1 << 8)
#define USART_TXEIE         (1 << 7)
#define USART_TCIE          (1 << 6)
#define USART_RXNEIE        (1 << 5)
#define USART_IDLEIE        (1 << 4)
#define USART_TE            (1 << 3)
#define USART_RE            (1 << 2)
#define USART_RWU           (1 << 1)
#define USART_SBK           (1 << 0)

// CR2 bits
#define USART_LINEN         (1 << 14)
#define USART_STOP_1        (0 << 12)
#define USART_STOP_05       (1 << 12)
#define USART_STOP_2        (2 << 12)
#define USART_STOP_15       (3 << 12)
#define USART_STOP_MASK     (3 << 12)
#define USART_CLKEN         (1 << 11)
#define USART_CPOL          (1 << 10)
#define USART_CPHA          (1 << 9)
#define USART_LBCL          (1 << 8)
#define USART_LBDIE         (1 << 6)
#define USART_LBDL          (1 << 5)
#define USART_ADD(n)        (n)

// CR3 bits
#define USART_ONEBIT        (1 << 11)
#define USART_CTSIE         (1 << 10)
#define USART_CTSE          (1 << 9)
#define USART_RTSE          (1 << 8)
#define USART_DMAT          (1 << 7)
#define USART_DMAR          (1 << 6)
#define USART_SCEN          (1 << 5)
#define USART_NACK          (1 << 4)
#define USART_HDSEL         (1 << 3)
#define USART_IRLP          (1 << 2)
#define USART_IREN          (1 << 1)
#define USART_EIE           (1 << 0)

// GTPR bits
#define USART_GT(n)         ((n) << 8)
#define USART_PSC(n)        (n)


/////////////////////////////////////////
// ADC
/////////////////////////////////////////
typedef struct
{
    arm_reg_t SR;       // status
    arm_reg_t CR1;      // control 1
    arm_reg_t CR2;      // control 2
    arm_reg_t SMPR1;    // sample time 1
    arm_reg_t SMPR2;    // sample time 2
    arm_reg_t JOFR1;    // injected channel data offset 1
    arm_reg_t JOFR2;    // injected channel data offset 2
    arm_reg_t JOFR3;    // injected channel data offset 3
    arm_reg_t JOFR4;    // injected channel data offset 4
    arm_reg_t HTR;      // watchdog higher threshold
    arm_reg_t LTR;      // watchdog lower threshold
    arm_reg_t SQR1;     // regular sequence 1
    arm_reg_t SQR2;     // regular sequence 2
    arm_reg_t SQR3;     // regular sequence 3
    arm_reg_t JSQR;     // injected sequence
    arm_reg_t JDR1;     // injected data 1
    arm_reg_t JDR2;     // injected data 2
    arm_reg_t JDR3;     // injected data 3
    arm_reg_t JDR4;     // injected data 4
    arm_reg_t DR;       // regular data
} ADC_t;

typedef struct
{
    arm_reg_t CSR;      // common status
    arm_reg_t CCR;      // common control
    arm_reg_t CDR;      // common regular data for dual and triple modes
} ADC_COMMON_t;

#define ADC1        ((ADC_t*) STM_ADC1_BASE)
#define ADC2        ((ADC_t*) STM_ADC2_BASE)
#define ADC3        ((ADC_t*) STM_ADC3_BASE)
#define ADC_COM     ((ADC_COMMON_t*) STM_ADC_COM_BASE)

// ADC_SR
#define ADC_OVR                 (1 << 5)
#define ADC_STRT                (1 << 4)
#define ADC_JSTRT               (1 << 3)
#define ADC_JEOC                (1 << 2)
#define ADC_EOC                 (1 << 1)
#define ADC_AWD                 (1 << 0)

// ADC_CR1
#define ADC_OVRIE               (1 << 26)
#define ADC_RES_12BIT           (0 << 24)
#define ADC_RES_10BIT           (1 << 24)
#define ADC_RES_8BIT            (2 << 24)
#define ADC_RES_6BIT            (3 << 24)
#define ADC_AWDEN               (1 << 23)
#define ADC_JAWDEN              (1 << 22)
#define ADC_DISCNUM(n)          ((n) << 13)
#define ADC_JDISCEN             (1 << 12)
#define ADC_DISCEN              (1 << 11)
#define ADC_JAUTO               (1 << 10)
#define ADC_AWDSGL              (1 << 9)
#define ADC_SCAN                (1 << 8)
#define ADC_JOECIE              (1 << 7)
#define ADC_AWDIE               (1 << 6)
#define ADC_EOCIE               (1 << 5)
#define ADC_AWDCH(n)            ((n) & 0x1F)

// ADC_CR2
#define ADC_SWSTART             (1 << 30)
#define ADC_EXTEN_DISABLE       (0 << 28)
#define ADC_EXTEN_RISING        (1 << 28)
#define ADC_EXTEN_FALLING       (2 << 28)
#define ADC_EXTEN_BOTH          (3 << 28)
#define ADC_EXTSEL_TIM1_CC1     (0 << 24)
#define ADC_EXTSEL_TIM1_CC2     (1 << 24)
#define ADC_EXTSEL_TIM1_CC3     (2 << 24)
#define ADC_EXTSEL_TIM2_CC2     (3 << 24)
#define ADC_EXTSEL_TIM2_CC3     (4 << 24)
#define ADC_EXTSEL_TIM2_CC4     (5 << 24)
#define ADC_EXTSEL_TIM2_TRGO    (6 << 24)
#define ADC_EXTSEL_TIM3_CC1     (7 << 24)
#define ADC_EXTSEL_TIM3_TRGO    (8 << 24)
#define ADC_EXTSEL_TIM4_CC4     (9 << 24)
#define ADC_EXTSEL_TIM5_CC1     (10 << 24)
#define ADC_EXTSEL_TIM5_CC2     (11 << 24)
#define ADC_EXTSEL_TIM5_CC3     (12 << 24)
#define ADC_EXTSEL_TIM8_CC1     (13 << 24)
#define ADC_EXTSEL_TIM8_TRGO    (14 << 24)
#define ADC_EXTSEL_EXTI11       (15 << 24)
#define ADC_JSWSTART            (1 << 22)
#define ADC_JEXTEN_DISABLE      (0 << 20)
#define ADC_JEXTEN_RISING       (1 << 20)
#define ADC_JEXTEN_FALLING      (2 << 20)
#define ADC_JEXTEN_BOTH         (3 << 20)
#define ADC_JEXTSEL_TIM1_CC4    (0 << 16)
#define ADC_JEXTSEL_TIM1_TRGO   (1 << 16)
#define ADC_JEXTSEL_TIM2_CC1    (2 << 16)
#define ADC_JEXTSEL_TIM2_TRGO   (3 << 16)
#define ADC_JEXTSEL_TIM3_CC2    (4 << 16)
#define ADC_JEXTSEL_TIM3_CC4    (5 << 16)
#define ADC_JEXTSEL_TIM4_CC1    (6 << 16)
#define ADC_JEXTSEL_TIM4_CC2    (7 << 16)
#define ADC_JEXTSEL_TIM4_CC3    (8 << 16)
#define ADC_JEXTSEL_TIM4_TRGO   (9 << 16)
#define ADC_JEXTSEL_TIM5_CC4    (10 << 16)
#define ADC_JEXTSEL_TIM5_TRGO   (11 << 16)
#define ADC_JEXTSEL_TIM8_CC1    (12 << 16)
#define ADC_JEXTSEL_TIM8_CC2    (13 << 16)
#define ADC_JEXTSEL_TIM8_CC3    (14 << 16)
#define ADC_JEXTSEL_EXTI15      (15 << 16)
#define ADC_ALIGN               (1 << 11)
#define ADC_EOCS                (1 << 10)
#define ADC_DDS                 (1 << 9)
#define ADC_DMA                 (1 << 8)
#define ADC_CONT                (1 << 1)
#define ADC_ADON                (1 << 0)

// ADC_SMPR1, ADC_SMPR2
#define ADC_SMP_3CYCLES(n)      (0 << (3 * (n%10)))
#define ADC_SMP_15CYCLES(n)     (1 << (3 * (n%10)))
#define ADC_SMP_28CYCLES(n)     (2 << (3 * (n%10)))
#define ADC_SMP_56CYCLES(n)     (3 << (3 * (n%10)))
#define ADC_SMP_84CYCLES(n)     (4 << (3 * (n%10)))
#define ADC_SMP_112CYCLES(n)    (5 << (3 * (n%10)))
#define ADC_SMP_144CYCLES(n)    (6 << (3 * (n%10)))
#define ADC_SMP_480CYCLES(n)    (7 << (3 * (n%10)))

// ADC_SQR1
#define ADC_L(n)                (((n) - 1) << 20)
#define ADC_SQ16(n)             ((n) << 15)
#define ADC_SQ15(n)             ((n) << 10)
#define ADC_SQ14(n)             ((n) << 5)
#define ADC_SQ13(n)             ((n) << 0)

// ADC_SQR2
#define ADC_SQ12(n)             ((n) << 26)
#define ADC_SQ11(n)             ((n) << 20)
#define ADC_SQ10(n)             ((n) << 15)
#define ADC_SQ9(n)              ((n) << 10)
#define ADC_SQ8(n)              ((n) << 5)
#define ADC_SQ7(n)              ((n) << 0)

// ADC_SQR3
#define ADC_SQ6(n)              ((n) << 25)
#define ADC_SQ5(n)              ((n) << 20)
#define ADC_SQ4(n)              ((n) << 15)
#define ADC_SQ3(n)              ((n) << 10)
#define ADC_SQ2(n)              ((n) << 5)
#define ADC_SQ1(n)              ((n) << 0)

// ADC_JSQR
#define ADC_JL(n)               ((n) << 20)
#define ADC_JSQ4(n)             ((n) << 15)
#define ADC_JSQ3(n)             ((n) << 10)
#define ADC_JSQ2(n)             ((n) << 5)
#define ADC_JSQ1(n)             ((n) << 0)

// ADC_CSR
#define ADC_OVR3                (1 << 21)
#define ADC_STRT3               (1 << 20)
#define ADC_JSTRT3              (1 << 19)
#define ADC_JEOC3               (1 << 18)
#define ADC_EOC3                (1 << 17)
#define ADC_AWD3                (1 << 16)
#define ADC_OVR2                (1 << 13)
#define ADC_STRT2               (1 << 12)
#define ADC_JSTRT2              (1 << 11)
#define ADC_JEOC2               (1 << 10)
#define ADC_EOC2                (1 << 9)
#define ADC_AWD2                (1 << 8)
#define ADC_OVR1                (1 << 5)
#define ADC_STRT1               (1 << 4)
#define ADC_JSTRT1              (1 << 3)
#define ADC_JEOC1               (1 << 2)
#define ADC_EOC1                (1 << 1)
#define ADC_AWD1                (1 << 0)

// ADC_CCR
#define ADC_TSVREFE             (1 << 23)
#define ADC_VBATE               (1 << 22)
#define ADC_ADCPRE_DIV2         (0 << 16)
#define ADC_ADCPRE_DIV4         (1 << 16)
#define ADC_ADCPRE_DIV6         (2 << 16)
#define ADC_ADCPRE_DIV8         (3 << 16)
#define ADC_DMA_DISABLE         (0 << 14)
#define ADC_DMA_MODE1           (1 << 14)
#define ADC_DMA_MODE2           (2 << 14)
#define ADC_DMA_MODE3           (3 << 14)
#define ADC_COMMON_DDS          (1 << 13)
#define ADC_DELAY(n)            (((n) - 5) << 8)
#define ADC_MULTI_INDEP                         (0 << 0)
#define ADC_MULTI_DUAL_REG_SIM_INJ_SIM          (1 << 0)
#define ADC_MULTI_DUAL_REG_SIM_INJ_ALT          (2 << 0)
#define ADC_MULTI_DUAL_INJ_SIM                  (5 << 0)
#define ADC_MULTI_DUAL_REG_SIM                  (6 << 0)
#define ADC_MULTI_DUAL_INTERLEAVED              (7 << 0)
#define ADC_MULTI_DUAL_ALT                      (9 << 0)
#define ADC_MULTI_TRIPLE_REG_SIM_INJ_SIM        (17 << 0)
#define ADC_MULTI_TRIPLE_REG_SIM_INJ_ALT        (18 << 0)
#define ADC_MULTI_TRIPLE_INJ_SIM                (21 << 0)
#define ADC_MULTI_TRIPLE_REG_SIM                (22 << 0)
#define ADC_MULTI_TRIPLE_INTERLEAVED            (23 << 0)
#define ADC_MULTI_TRIPLE_ALT                    (25 << 0)

// ADC CDR
#define ADC_GET_DATA2(r)        ((r) >> 16)
#define ADC_GET_DATA1(r)        ((r) & 0xFF)


/////////////////////////////////////////
// DMA
/////////////////////////////////////////
typedef struct
{
    arm_reg_t CR;       // Configuration
    arm_reg_t NDTR;     // Number of data
    arm_reg_t PAR;      // Peripheral address
    arm_reg_t M0AR;     // Memory 0 address
    arm_reg_t M1AR;     // Memory 1 address
    arm_reg_t FCR;      // FIFO control
} DMA_STREAM_t;

typedef struct
{
    arm_reg_t       LISR;       // Low Interrupt Status
    arm_reg_t       HISR;       // High Interrupt Status
    arm_reg_t       LIFCR;      // Low Interrupt Flag Clear
    arm_reg_t       HIFCR;      // High Interrupt Flag Clear
    DMA_STREAM_t    stream[8];  // DMA stream registers
} DMA_t;

#define DMA1    ((DMA_t*) STM_DMA1_BASE)
#define DMA2    ((DMA_t*) STM_DMA2_BASE)

#define DMA1_STR(x)     ((DMA_STREAM_t *) &(DMA1->stream[(x)]))
#define DMA2_STR(x)     ((DMA_STREAM_t *) &(DMA2->stream[(x)]))

// LISR, HISR bits
#define DMA_TCIF(x)         (32 << ((((x) % 2) * 6) + (((x) % 4) >= 2) ? 16 : 0))
#define DMA_HTIF(x)         (16 << ((((x) % 2) * 6) + (((x) % 4) >= 2) ? 16 : 0))
#define DMA_TEIF(x)         ( 8 << ((((x) % 2) * 6) + (((x) % 4) >= 2) ? 16 : 0))
#define DMA_DMEIF(x)        ( 4 << ((((x) % 2) * 6) + (((x) % 4) >= 2) ? 16 : 0))
#define DMA_FEIF(x)         ( 1 << ((((x) % 2) * 6) + (((x) % 4) >= 2) ? 16 : 0))

// LIFSR, HIFSR bits
#define DMA_CTCIF(x)        (32 << ((((x) % 2) * 6) + (((x) % 4) >= 2) ? 16 : 0))
#define DMA_CHTIF(x)        (16 << ((((x) % 2) * 6) + (((x) % 4) >= 2) ? 16 : 0))
#define DMA_CTEIF(x)        ( 8 << ((((x) % 2) * 6) + (((x) % 4) >= 2) ? 16 : 0))
#define DMA_CDMEIF(x)       ( 4 << ((((x) % 2) * 6) + (((x) % 4) >= 2) ? 16 : 0))
#define DMA_CFEIF(x)        ( 1 << ((((x) % 2) * 6) + (((x) % 4) >= 2) ? 16 : 0))

// Stream CR bits
#define DMA_CHSEL(x)        ((x) << 25)
#define DMA_MBURST_SINGLE   (0 << 23)
#define DMA_MBURST_INCR4    (1 << 23)
#define DMA_MBURST_INCR8    (2 << 23)
#define DMA_MBURST_INCR16   (3 << 23)
#define DMA_PBURST_SINGLE   (0 << 21)
#define DMA_PBURST_INCR4    (1 << 21)
#define DMA_PBURST_INCR8    (2 << 21)
#define DMA_PBURST_INCR16   (3 << 21)
#define DMA_CT              (1 << 19)
#define DMA_DBM             (1 << 18)
#define DMA_PL(x)           ((x) << 16)
#define DMA_PINCOS          (1 << 15)
#define DMA_MSIZE_8         (0 << 13)
#define DMA_MSIZE_16        (1 << 13)
#define DMA_MSIZE_32        (2 << 13)
#define DMA_PSIZE_8         (0 << 11)
#define DMA_PSIZE_16        (1 << 11)
#define DMA_PSIZE_32        (2 << 11)
#define DMA_MINC            (1 << 10)
#define DMA_PINC            (1 << 9)
#define DMA_CIRC            (1 << 8)
#define DMA_DIR_P2M         (0 << 6)
#define DMA_DIR_M2P         (1 << 6)
#define DMA_DIR_M2M         (2 << 6)
#define DMA_PFCTRL          (1 << 5)
#define DMA_TCIE            (1 << 4)
#define DMA_HTIE            (1 << 3)
#define DMA_TEIE            (1 << 2)
#define DMA_DMEIE           (1 << 1)
#define DMA_EN              (1 << 0)

// FCR bits
#define DMA_FEIE            (1 << 7)
#define DMA_FS_MASK         0x38
#define DMA_FS_0_TO_14      (0 << 3)
#define DMA_FS_14_TO_12     (1 << 3)
#define DMA_FS_12_TO_34     (2 << 3)
#define DMA_FS_34_TO_FULL   (3 << 3)
#define DMA_FS_EMPTY        (4 << 3)
#define DMA_FS_FULL         (5 << 3)
#define DMA_DMDIS           (1 << 2)
#define DMA_FTH_14          (0 << 0)
#define DMA_FTH_12          (1 << 0)
#define DMA_FTH_34          (2 << 0)
#define DMA_FTH_FULL        (3 << 0)


/////////////////////////////////////////
// I2C
/////////////////////////////////////////
typedef struct
{
    arm_reg_t CR1;      // Control 1
    arm_reg_t CR2;      // Control 2
    arm_reg_t OAR1;     // Own address 1
    arm_reg_t OAR2;     // Own address 2
    arm_reg_t DR;       // Data
    arm_reg_t SR1;      // Status 1
    arm_reg_t SR2;      // Status 2
    arm_reg_t CCR;      // Clock control
    arm_reg_t TRISE;    // TRISE
    arm_reg_t FLTR;     // Filter
} I2C_t;

#define I2C1    ((I2C_t*) STM_I2C1_BASE)
#define I2C2    ((I2C_t*) STM_I2C2_BASE)
#define I2C3    ((I2C_t*) STM_I2C3_BASE)

// I2C_CR1 bits
#define I2C_SWRST           (1 << 15)
#define I2C_ALERT           (1 << 13)
#define I2C_PEC             (1 << 12)
#define I2C_POS             (1 << 11)
#define I2C_ACK             (1 << 10)
#define I2C_STOP            (1 << 9)
#define I2C_START           (1 << 8)
#define I2C_NOSTRETCH       (1 << 7)
#define I2C_ENGC            (1 << 6)
#define I2C_ENPEC           (1 << 5)
#define I2C_ENARR           (1 << 4)
#define I2C_SMBTYPE         (1 << 3)
#define I2C_SMBUS           (1 << 1)
#define I2C_PE              (1 << 0)

// I2C_CR2 bits
#define I2C_LAST            (1 << 12)
#define I2C_DMAEN           (1 << 11)
#define I2C_ITBUFEN         (1 << 10)
#define I2C_ITEVTEN         (1 << 9)
#define I2C_ITERREN         (1 << 8)
#define I2C_FREQ(x)         ((x) << 0)

// I2C_OAR1 bits
#define I2C_ADDMODE         (1 << 15)
#define I2C_ALWAYS_ONE      (1 << 14)
#define I2C_ADD(x)          ((x) << 0)

// I2C_OAR2 bits
#define I2C_ADD2(x)         ((x) << 1)
#define I2C_ENDUAL          (1 << 0)

// I2C_SR1 bits
#define I2C_SMBALERT        (1 << 15)
#define I2C_TIMEOUT         (1 << 14)
#define I2C_PECERR          (1 << 12)
#define I2C_OVR             (1 << 11)
#define I2C_AF              (1 << 10)
#define I2C_ARLO            (1 << 9)
#define I2C_BERR            (1 << 8)
#define I2C_TXE             (1 << 7)
#define I2C_RXNE            (1 << 6)
#define I2C_STOPF           (1 << 4)
#define I2C_ADD10           (1 << 3)
#define I2C_BTF             (1 << 2)
#define I2C_ADDR            (1 << 1)
#define I2C_SB              (1 << 0)

// I2C_SR2 bits
#define I2C_PECR(x)         ((x) << 8)
#define I2C_DUALF           (1 << 7)
#define I2C_SMBHOST         (1 << 6)
#define I2C_SMBDEFAULT      (1 << 5)
#define I2C_GENCALL         (1 << 4)
#define I2C_TRA             (1 << 2)
#define I2C_BUSY            (1 << 1)
#define I2C_MSL             (1 << 0)

// I2C_CCR bits
#define I2C_FS              (1 << 15)
#define I2C_DUTY            (1 << 14)
#define I2C_CCR(x)          ((x) << 0)

// I2C_FLTR bits
#define I2C_ANOFF           (1 << 4)
#define I2C_DNF(x)          ((x) << 0)


/////////////////////////////////////////
// RTC
/////////////////////////////////////////
typedef struct
{
    arm_reg_t TR;       // Time
    arm_reg_t DR;       // Date
    arm_reg_t CR;       // Control
    arm_reg_t ISR;      // Initialization and status
    arm_reg_t PRER;     // Prescaler
    arm_reg_t WUTR;     // Wakeup timer
    arm_reg_t CALIBR;   // Calibration
    arm_reg_t ALRMAR;   // Alarm A
    arm_reg_t ALRMBR;   // Alarm B
    arm_reg_t WPR;      // Write protection
    arm_reg_t SSR;      // Sub second
    arm_reg_t SHIFTR;   // Shift control
    arm_reg_t TSTR;     // Time stamp time
    arm_reg_t TSDR;     // Time stamp date
    arm_reg_t TSSSR;    // Timestamp sub second
    arm_reg_t CALR;     // Calibration
    arm_reg_t TAFCR;    // Tamper and alternate function configuration
    arm_reg_t ALRMASSR; // Alarm A sub second
    arm_reg_t ALRMBSSR; // Alarm B sub second
    arm_reg_t gap0;
    arm_reg_t BKPR[20]; // Backup registers
} RTC_t;

#define RTC   ((RTC_t*) STM_RTC_BKP_BASE)

// TR, TSTR bits
#define RTC_PM          (1 << 22)
#define RTC_HT(x)       ((x) << 20)
#define RTC_GET_HT(r)   (((r) >> 20) & 0x3)
#define RTC_HU(x)       ((x) << 16)
#define RTC_GET_HU(r)   (((r) >> 16) & 0xF)
#define RTC_MNT(x)      ((x) << 12)
#define RTC_GET_MNT(r)  (((r) >> 12) & 0x7)
#define RTC_MNU(x)      ((x) << 8)
#define RTC_GET_MNU(r)  (((r) >> 8) & 0xF)
#define RTC_ST(x)       ((x) << 4)
#define RTC_GET_ST(r)   (((r) >> 4) & 0x7)
#define RTC_SU(x)       ((x))
#define RTC_GET_SU(r)   (((r)) & 0xF)

// DR, TSDR (low 16 bits) bits
#define RTC_YT(x)       ((x) << 20)
#define RTC_GET_YT(r)   (((r) >> 20) & 0xF)
#define RTC_YU(x)       ((x) << 16)
#define RTC_GET_YU(r)   (((r) >> 16) & 0xF)
#define RTC_WDU(x)      ((x) << 13)
#define RTC_GET_WDU(r)  (((r) >> 13) & 0x7)
#define RTC_WDU_MON     1
#define RTC_WDU_TUE     2
#define RTC_WDU_WED     3
#define RTC_WDU_THU     4
#define RTC_WDU_FRI     5
#define RTC_WDU_SAT     6
#define RTC_WDU_SUN     7
#define RTC_MT(x)       ((x) << 12)
#define RTC_GET_MT(r)   (((r) >> 12) & 0x1)
#define RTC_MU(x)       ((x) << 8)
#define RTC_GET_MU(r)   (((r) >> 8) & 0xF)
#define RTC_DT(x)       ((x) << 4)
#define RTC_GET_DT(r)   (((r) >> 4) & 0x3)
#define RTC_DU(x)       ((x))
#define RTC_GET_DU(r)   (((r)) & 0xF)

// CR bits
#define RTC_COE         (1 << 23)
#define RTC_OSEL(x)     ((x) << 21)
#define RTC_POL         (1 << 20)
#define RTC_COSEL       (1 << 19)
#define RTC_BKP         (1 << 18)
#define RTC_SUB1H       (1 << 17)
#define RTC_ADD1H       (1 << 16)
#define RTC_TSIE        (1 << 15)
#define RTC_WUTIE       (1 << 14)
#define RTC_ALRBIE      (1 << 13)
#define RTC_ALRAIE      (1 << 12)
#define RTC_TSE         (1 << 11)
#define RTC_WUTE        (1 << 10)
#define RTC_ALRBE       (1 << 9)
#define RTC_ALRAE       (1 << 8)
#define RTC_DCE         (1 << 7)
#define RTC_FMT         (1 << 6)
#define RTC_BYPSHAD     (1 << 5)
#define RTC_REFCKON     (1 << 4)
#define RTC_TSEDGE      (1 << 3)
#define RTC_WUCKSEL_DIV16           0
#define RTC_WUCKSEL_DIV8            1
#define RTC_WUCKSEL_DIV4            2
#define RTC_WUCKSEL_DIV2            3
#define RTC_WUCKSEL_CK_SPRE         4
#define RTC_WUCKSEL_CK_SPRE_ADDED   6

// ISR bits
#define RTC_TAMP2F      (1 << 14)
#define RTC_TAMP1F      (1 << 13)
#define RTC_TSOVF       (1 << 12)
#define RTC_TSF         (1 << 11)
#define RTC_WUTF        (1 << 10)
#define RTC_ALRBF       (1 << 9)
#define RTC_ALRAF       (1 << 8)
#define RTC_INIT        (1 << 7)
#define RTC_INITF       (1 << 6)
#define RTC_RSF         (1 << 5)
#define RTC_INITS       (1 << 4)
#define RTC_SHPF        (1 << 3)
#define RTC_WUTWF       (1 << 2)
#define RTC_ALRBWF      (1 << 1)
#define RTC_ALRAWF      (1 << 0)

// PRER bits
#define RTC_PREDIV_A(x) ((x) << 16)
#define RTC_PREDIV_S(x) ((x))

// CALIBR bits
#define RTC_DCS         (1 << 7)
#define RTC_DC(x)       ((x))

// ALRMAR, ALRMBR bits
#define RTC_ALRM_MSK4   (1 << 31)
#define RTC_ALRM_WDSEL  (1 << 30)
#define RTC_ALRM_DT(x)  ((x) << 28)
#define RTC_ALRM_DU(x)  ((x) << 24)
#define RTC_ALRM_MSK3   (1 << 23)
#define RTC_ALRM_PM     (1 << 22)
#define RTC_ALRM_HT(x)  ((x) << 20)
#define RTC_ALRM_HU(x)  ((x) << 16)
#define RTC_ALRM_MSK2   (1 << 15)
#define RTC_ALRM_MNT(x) ((x) << 12)
#define RTC_ALRM_MNU(x) ((x) << 8)
#define RTC_ALRM_MSK1   (1 << 7)
#define RTC_ALRM_ST(x)  ((x) << 4)
#define RTC_ALRM_SU(x)  ((x) << 0)

// SHIFTR bits
#define RTC_ADD1S       (1 << 31)
#define RTC_SUBFS(x)    ((x) << 0)

// CALR bits
#define RTC_CALP        (1 << 15)
#define RTC_CALW8       (1 << 14)
#define RTC_CALW16      (1 << 13)
#define RTC_CALM(x)     (x)

// TAFCR bits
#define RTC_ALARMOUTTYPE    (1 << 18)
#define RTC_TSINSEL         (1 << 17)
#define RTC_TAMP1INSEL      (1 << 16)
#define RTC_TAMPPUDIS       (1 << 15)
#define RTC_TAMPPRCH(x)     ((x) << 13)
#define RTC_TAMPFLT(x)      ((x) << 11)
#define RTC_TAMPFREQ(x)     ((x) << 8)
#define RTC_TAMPTS          (1 << 7)
#define RTC_TAMP2TRG        (1 << 4)
#define RTC_TAMP2E          (1 << 3)
#define RTC_TAMPIE          (1 << 2)
#define RTC_TAMP1TRG        (1 << 1)
#define RTC_TAMP1E          (1 << 0)

// ALRMASSR, ALRMASSR bits
#define RTC_MASKSS(x)       ((x) << 24)
#define RTC_SS(x)           ((x) & 0x7FFF)


/////////////////////////////////////////
// Independent watchdog (IWDG)
/////////////////////////////////////////
typedef struct
{
    arm_reg_t KR;       // Key
    arm_reg_t DR;       // Prescaler
    arm_reg_t RLR;      // Reload
    arm_reg_t SR;      // Status
} IWDG_t;

#define IWDG   ((IWDG_t*) STM_IWDG_BASE)

// IWDG_KR bits
#define IWDG_ALIVE_KEY      0xAAAA
#define IWDG_ACCESS_KEY     0x5555
#define IWDG_START_KEY      0xCCCC

// IWDG_PR bits
#define IWDG_PR_DIV_4       0
#define IWDG_PR_DIV_8       1
#define IWDG_PR_DIV_16      2
#define IWDG_PR_DIV_32      3
#define IWDG_PR_DIV_64      4
#define IWDG_PR_DIV_128     5
#define IWDG_PR_DIV_256     6

// IWDG_SR bits
#define IWDG_RVU            (1 << 1)
#define IWDG_PVU            (1 << 0)


/////////////////////////////////////////
// Window watchdog (WWDG)
/////////////////////////////////////////
typedef struct
{
    arm_reg_t CR;       // Control
    arm_reg_t CFR;      // Configuration
    arm_reg_t ISR;      // Status
} WWDG_t;

#define WWDG   ((WWDG_t*) STM_WWDG_BASE)

// WWDG_CR bits
#define WWDG_WDGA           (1 << 7)
#define WWDG_T(n)           (n)

// WWDG_CFR bits
#define WWDG_EWI            (1 << 9)
#define WWDG_WDGTB_DIV_1    (0 << 7)
#define WWDG_WDGTB_DIV_2    (1 << 7)
#define WWDG_WDGTB_DIV_4    (2 << 7)
#define WWDG_WDGTB_DIV_8    (3 << 7)
#define WWDG_W(n)           (n)

// WWDG_SR bits
#define WWDG_EWIF           (1 << 0)


/////////////////////////////////////////
// SPI
/////////////////////////////////////////
typedef struct
{
    arm_reg_t CR1;      // Control 1
    arm_reg_t CR2;      // Control 2
    arm_reg_t SR;       // Status
    arm_reg_t DR;       // Data
    arm_reg_t CRCPR;    // CRC polynomial
    arm_reg_t RXCRCR;   // RX CRC
    arm_reg_t TXCRCR;   // TX CRC
    arm_reg_t I2SCFGR;  // I2S configuration
    arm_reg_t I2SPR;    // I2S prescaler
} SPI_t;

#define SPI1    ((SPI_t*) STM_SPI1_BASE)
#define SPI2    ((SPI_t*) STM_SPI2_I2S2_BASE)
#define SPI3    ((SPI_t*) STM_SPI3_I2S3_BASE)
#define SPI4    ((SPI_t*) STM_SPI4_BASE)
#define SPI5    ((SPI_t*) STM_SPI5_BASE)
#define SPI6    ((SPI_t*) STM_SPI6_BASE)

// SPI_CR1 bits
#define SPI_BIDIMODE        (1 << 15)
#define SPI_BIDIOE          (1 << 14)
#define SPI_CRCEN           (1 << 13)
#define SPI_CRCNEXT         (1 << 12)
#define SPI_DFF             (1 << 11)
#define SPI_RXONLY          (1 << 10)
#define SPI_SSM             (1 << 9)
#define SPI_SSI             (1 << 8)
#define SPI_LSBFIRST        (1 << 7)
#define SPI_SPE             (1 << 6)
#define SPI_BR(x)           ((x) << 3)
#define SPI_MSTR            (1 << 2)
#define SPI_CPOL            (1 << 1)
#define SPI_CPHA            (1 << 0)

// SPI_CR2 bits
#define SPI_TXEIE           (1 << 7)
#define SPI_RXNEIE          (1 << 6)
#define SPI_ERRIE           (1 << 5)
#define SPI_FRF             (1 << 4)
#define SPI_SSOE            (1 << 2)
#define SPI_TXDMAEN         (1 << 1)
#define SPI_RXDMAEN         (1 << 0)

// SPI_SR bits
#define SPI_FRE             (1 << 8)
#define SPI_BSY             (1 << 7)
#define SPI_OVR             (1 << 6)
#define SPI_MODF            (1 << 5)
#define SPI_CRCERR          (1 << 4)
#define SPI_UDR             (1 << 3)
#define SPI_CHSIDE          (1 << 2)
#define SPI_TXE             (1 << 1)
#define SPI_RXNE            (1 << 0)

// SPI_I2SCFGR bits
#define SPI_I2SMOD              (1 << 11)
#define SPI_I2SE                (1 << 10)
#define SPI_I2SCFG_SLAVE_TX     (0 << 8)
#define SPI_I2SCFG_SLAVE_RX     (1 << 8)
#define SPI_I2SCFG_MASTER_TX    (2 << 8)
#define SPI_I2SCFG_MASTER_RX    (3 << 8)
#define SPI_PCMSYNC             (1 << 7)
#define SPI_I2SSTD_PHILIPS      (0 << 4)
#define SPI_I2SSTD_MSB_JUST     (1 << 4)
#define SPI_I2SSTD_LSB_JUST     (2 << 4)
#define SPI_I2SSTD_PCM          (3 << 4)
#define SPI_CKPOL               (1 << 3)
#define SPI_DATLEN_16BIT        (0 << 1)
#define SPI_DATLEN_24BIT        (1 << 1)
#define SPI_DATLEN_32BIT        (2 << 1)
#define SPI_CHLEN_16BIT         (0 << 0)
#define SPI_CHLEN_32BIT         (1 << 0)

// SPI_I2SPR bits
#define SPI_MCKOE           (1 << 9)
#define SPI_ODD             (1 << 8)
#define SPI_I2SDIV(x)       (x)

/////////////////////////////////////////
// CAN
/////////////////////////////////////////
typedef struct
{
    arm_reg_t IR;
    arm_reg_t DTR;
    arm_reg_t LR;
    arm_reg_t HR;
} CAN_mailbox_t;

typedef struct
{
    arm_reg_t R1;
    arm_reg_t R2;
} CAN_filter_t;

typedef struct
{
    arm_reg_t MCR;
    arm_reg_t MSR;
    arm_reg_t TSR;
    arm_reg_t RFxR[2];
    arm_reg_t IER;
    arm_reg_t ESR;
    arm_reg_t BTR;
    
    arm_reg_t reserved0[88];
    
    CAN_mailbox_t   TMB[3];
    CAN_mailbox_t   RMB[2];
    
    arm_reg_t reserved1[12];
    
    arm_reg_t FMR;
    arm_reg_t FM1R;
    
    arm_reg_t reserved2;
    
    arm_reg_t FS1R;
    
    arm_reg_t reserved3;
    
    arm_reg_t FFA1R;
    
    arm_reg_t reserved4;
    
    arm_reg_t FA1R;
    
    arm_reg_t reserved5;
    
    arm_reg_t reserved6[7];

    CAN_filter_t    FILT[14];
} CAN_t;

#define CAN1   ((CAN_t*) STM_CAN1_BASE)
#define CAN2   ((CAN_t*) STM_CAN2_BASE)

// MCR bits
#define CAN_DBF             (1 << 16)
#define CAN_RESET           (1 << 15)
#define CAN_TTCM            (1 << 7)
#define CAN_ABOM            (1 << 6)
#define CAN_AWUM            (1 << 5)
#define CAN_NART            (1 << 4)
#define CAN_RFLM            (1 << 3)
#define CAN_TXFP            (1 << 2)
#define CAN_SLEEP           (1 << 1)
#define CAN_INRQ            (1 << 0)

// MSR bits
#define CAN_RX              (1 << 11)
#define CAN_SAMP            (1 << 10)
#define CAN_RXM             (1 << 9)
#define CAN_TXM             (1 << 8)
#define CAN_SLAKI           (1 << 4)
#define CAN_WKUI            (1 << 3)
#define CAN_ERRI            (1 << 2)
#define CAN_SLAK            (1 << 1)
#define CAN_INAK            (1 << 0)

// TSR bits
#define CAN_LOW(n)          (1 << (29 + (n)))
#define CAN_TME(n)          (1 << (26 + (n)))
#define CAN_GET_CODE(r)     (((r) >> 24) & 0x3)
#define CAN_ABRQ(n)         (1 << (7 + 8 * (n)))
#define CAN_TERR(n)         (1 << (3 + 8 * (n)))
#define CAN_ALST(n)         (1 << (2 + 8 * (n)))
#define CAN_TXOK(n)         (1 << (1 + 8 * (n)))
#define CAN_RQCP(n)         (1 << (0 + 8 * (n)))

// RFxR bits
#define CAN_RFOM            (1 << 5)
#define CAN_FOVR            (1 << 4)
#define CAN_FULL            (1 << 3)
#define CAN_GET_FMP(r)      ((r) & 0x3)

// IER bits
#define CAN_SLKIE           (1 << 17)
#define CAN_WKUIE           (1 << 16)
#define CAN_ERRIE           (1 << 15)
#define CAN_LECIE           (1 << 11)
#define CAN_BOFIE           (1 << 10)
#define CAN_EPVIE           (1 << 9)
#define CAN_EWGIE           (1 << 8)
#define CAN_FOVIE(n)        (1 << (3 + 3 * (n)))
#define CAN_FFIE(n)         (1 << (2 + 3 * (n)))
#define CAN_FMPIE(n)        (1 << (1 + 3 * (n)))
#define CAN_TMEIE           (1 << 0)

// ESR bits
#define CAN_GET_REC(r)      (((r) >> 24) & 0xFF)
#define CAN_GET_TEC(r)      (((r) >> 16) & 0xFF)
#define CAN_GET_LEC(r)      (((r) >> 4) & 0x7)
#define CAN_BOFF            (1 << 2)
#define CAN_EPVF            (1 << 1)
#define CAN_EWGF            (1 << 0)
// LEC field codes
#define CAN_LEC_NO_ERROR                0
#define CAN_LEC_STUFF_ERROR             1
#define CAN_LEC_FORM_ERROR              2
#define CAN_LEC_ACKNOWLEDGMENT_ERROR    3
#define CAN_LEC_BIT_RECESSIVE_ERROR     4
#define CAN_LEC_BIT_DOMINANT_ERROR      5
#define CAN_LEC_CRC_ERROR               6
#define CAN_LEC_RESET                   7

// BTR bits
#define CAN_SILM            (1 << 31)
#define CAN_LBKM            (1 << 30)
#define CAN_SJW(x)          ((x) << 24)
#define CAN_TS2(x)          ((x) << 20)
#define CAN_TS1(x)          ((x) << 16)
#define CAN_BRP(x)          ((x) << 0)

// Mailbox IR bits
#define CAN_STID(x)         ((x) << 21)
#define CAN_GET_STID(r)     (((r) >> 21) & 0x7FF)
#define CAN_EXID(x)         ((x) << 3)
#define CAN_GET_EXID(r)     (((r) >> 3) & 0x1FFFFFFF)
#define CAN_IDE             (1 << 2)
#define CAN_RTR             (1 << 1)
#define CAN_TXRQ            (1 << 0)

// Mailbox DTR bits
#define CAN_TIME(x)         ((x) << 16)
#define CAN_GET_TIME(r)     ((r) >> 16)
#define CAN_TGT             (1 << 8)
#define CAN_GET_FMI(r)      (((r) >> 8) & 0xFF)
#define CAN_DLC(x)          (x)
#define CAN_GET_DLC(r)      ((r) & 0xF)

// FMR bits
#define CAN_CAN2SB(x)       ((x) << 8)
#define CAN_FINIT           (1 << 0)

// FM1R bits
#define CAN_FBM(n)          (1 << (n))

// FS1R bits
#define CAN_FSC(n)          (1 << (n))

// FFA1R bits
#define CAN_FFA(n)          (1 << (n))

// FA1R bits
#define CAN_FACT(n)         (1 << (n))



/*--------------------------------------------------------------------------
 * Timers
 */
typedef struct {
	arm_reg_t CR1; /*!< TIM control register 1,              Address offset: 0x00 */
	arm_reg_t CR2; /*!< TIM control register 2,              Address offset: 0x04 */
	arm_reg_t SMCR; /*!< TIM slave mode control register,     Address offset: 0x08 */
	arm_reg_t DIER; /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
	arm_reg_t SR; /*!< TIM status register,                 Address offset: 0x10 */
	arm_reg_t EGR; /*!< TIM event generation register,       Address offset: 0x14 */
	arm_reg_t CCMR1; /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
	arm_reg_t CCMR2; /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
	arm_reg_t CCER; /*!< TIM capture/compare enable register, Address offset: 0x20 */
	arm_reg_t CNT; /*!< TIM counter register,                Address offset: 0x24 */
	arm_reg_t PSC; /*!< TIM prescaler,                       Address offset: 0x28 */
	arm_reg_t ARR; /*!< TIM auto-reload register,            Address offset: 0x2C */
	arm_reg_t RCR; /*!< TIM repetition counter register,     Address offset: 0x30 */
	arm_reg_t CCR1; /*!< TIM capture/compare register 1,      Address offset: 0x34 */
	arm_reg_t CCR2; /*!< TIM capture/compare register 2,      Address offset: 0x38 */
	arm_reg_t CCR3; /*!< TIM capture/compare register 3,      Address offset: 0x3C */
	arm_reg_t CCR4; /*!< TIM capture/compare register 4,      Address offset: 0x40 */
	arm_reg_t BDTR; /*!< TIM break and dead-time register,    Address offset: 0x44 */
	arm_reg_t DCR; /*!< TIM DMA control register,            Address offset: 0x48 */
	arm_reg_t DMAR; /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
	arm_reg_t OR; /*!< TIM option register,                 Address offset: 0x50 */
} TIM_t;

#define TIM1	((TIM_t*) STM_TIM1_BASE)
#define TIM2	((TIM_t*) STM_TIM2_BASE)
#define TIM3	((TIM_t*) STM_TIM3_BASE)
#define TIM4	((TIM_t*) STM_TIM4_BASE)
#define TIM5	((TIM_t*) STM_TIM5_BASE)
#define TIM6	((TIM_t*) STM_TIM6_BASE)
#define TIM7	((TIM_t*) STM_TIM7_BASE)
#define TIM8	((TIM_t*) STM_TIM8_BASE)
#define TIM9	((TIM_t*) STM_TIM9_BASE)
#define TIM10	((TIM_t*) STM_TIM10_BASE)
#define TIM11	((TIM_t*) STM_TIM11_BASE)
#define TIM12	((TIM_t*) STM_TIM12_BASE)
#define TIM13	((TIM_t*) STM_TIM13_BASE)
#define TIM14	((TIM_t*) STM_TIM14_BASE)

// CR1
#define TIM_CR1_EN				(1 << 0)            /*!<Counter enable        */
#define  TIM_CR1_UDIS			((uint16_t)0x0002)            /*!<Update disable        */
#define  TIM_CR1_URS			((uint16_t)0x0004)            /*!<Update request source */
#define  TIM_CR1_OPM			((uint16_t)0x0008)            /*!<One pulse mode        */
#define  TIM_CR1_DIR			((uint16_t)0x0010)            /*!<Direction             */

#define TIM_OCMode_Timing                  ((uint16_t)0x0000)
#define TIM_OCMode_Active                  ((uint16_t)0x0010)
#define TIM_OCMode_Inactive                ((uint16_t)0x0020)
#define TIM_OCMode_Toggle                  ((uint16_t)0x0030)
#define TIM_OCMode_PWM1                    ((uint16_t)0x0060)
#define TIM_OCMode_PWM2                    ((uint16_t)0x0070

// CR
#define TIM_CK_DIV4			(0b10 << 8)
#define TIM_CK_DIV2			(0b01 << 8)
#define TIM_CK_DIV1			(0b00 << 8)
#define TIM_ARPE			(1 << 7)
#define TIM_CMS(n)			((n) << 5)
#define TIM_DIR_DWN			(1 << 4)
#define TIM_OPM				(1 << 3)
#define TIM_URS				(1 << 2)
#define TIM_UDIS			(1 << 1)
#define TIM_CEN				(1 << 0)

// CR2
#define TIM_TI1S			(1 << 7)
#define TIM_MMS(n)			((n) << 4)
#define TIM_CCDS			(1 << 2)

// SMCR
#define TIM_ETR_INV			(1 << 15)
#define TIM_ECE				(1 << 14)
#define TIM_EPS_1			(0b00 << 12)
#define TIM_EPS_2			(0b01 << 12)
#define TIM_EPS_4			(0b10 << 12)
#define TIM_EPS_8			(0b11 << 12)
#define TIM_ETF(n)			((n) << 8)
#define TIM_MSM				(1 << 7)
#define TIM_TS(n)			((n) << 4)
#define TIM_OCCS			(1 << 3)
#define TIM_SL_DIS			(0b000 << 0)
#define TIM_ENCOD_MODE1		(0b001 << 0)
#define TIM_ENCOD_MODE2		(0b010 << 0)
#define TIM_ENCOD_MODE3		(0b011 << 0)
#define TIM_RESET_MODE		(0b100 << 0)
#define TIM_GATED_MODE		(0b101 << 0)
#define TIM_TRIG_MODE		(0b110 << 0)
#define TIM_EXT_CLK_MODE	(0b111 << 0)

// SR
#define TIM_CC4OF			(1 << 12)
#define TIM_CC3OF			(1 << 11)
#define TIM_CC2OF			(1 << 10)
#define TIM_CC1OF			(1 << 9)
#define TIM_TIF				(1 << 6)
#define TIM_CC4IF			(1 << 4)
#define TIM_CC3IF			(1 << 3)
#define TIM_CC2IF			(1 << 2)
#define TIM_CC1IF			(1 << 1)
#define TIM_CUF				(1 << 0)

// EGR
#define TIM_TG				(1 << 6)
#define TIM_CC4F			(1 << 5)
#define TIM_CC3F			(1 << 4)
#define TIM_CC2F			(1 << 3)
#define TIM_CC1F			(1 << 2)
#define TIM_UF				(1 << 1)

// CCMR1

#define TIM_IC2F(n)			((n) << 12)
#define TIM_IC2PSC(n)		((n) << 10)

#define TIM_OC2CE			(1 << 15)
#define TIM_OC2M(n)			((n) << 12)
#define TIM_OC2PE			(1 << 11)
#define TIM_OC2FE			(1 << 10)
#define TIM_CC2S(n)			((n) << 8)


#define TIM_IC1F(n)			((n) << 4)
#define TIM_IC1PSC(n)		((n) << 2)

#define TIM_OC1CE			(1 << 7)
#define TIM_OC1M(n)			((n) << 4)
#define TIM_OC1PE			(1 << 3)
#define TIM_OC1FE			(1 << 2)
#define TIM_CC1S(n)			((n) << 0)

// CCMR2
#define TIM_IC4F(n)			((n) << 12)
#define TIM_IC4PSC(n)		((n) << 10)

#define TIM_OC4CE			(1 << 15)
#define TIM_OC4M(n)			((n) << 12)
#define TIM_OC4PE			(1 << 11)
#define TIM_OC4FE			(1 << 10)
#define TIM_CC4S(n)			((n) << 8)


#define TIM_IC3F(n)			((n) << 4)
#define TIM_IC3PSC(n)		((n) << 2)

#define TIM_OC3CE			(1 << 7)
#define TIM_OC3M(n)			((n) << 4)
#define TIM_OC3PE			(1 << 3)
#define TIM_OC3FE			(1 << 2)
#define TIM_CC3S(n)			((n) << 0)

#define TIM_CCMR_FROZEN			0b000
#define TIM_CCMR_CH1_ACTIVE		0b001
#define TIM_CCMR_CH1_INACTIVE	0b010
#define TIM_CCMR_TOGLE			0b011
#define TIM_CCMR_FORCE_INACT	0b100
#define TIM_CCMR_FORCE_ACT		0b101
#define TIM_CCMR_PWM1			0b110
#define TIM_CCMR_PWM2			0b111

// CCER
#define TIM_CC4P			(1 << 13)
#define TIM_CC4E			(1 << 12)
#define TIM_CC3P			(1 << 9)
#define TIM_CC3E			(1 << 8)
#define TIM_CC2P			(1 << 5)
#define TIM_CC2E			(1 << 4)
#define TIM_CC1P			(1 << 1)
#define TIM_CC1E			(1 << 0)


/*------------------------------------------------------------------------
 * Макроопределения для возможности указания привязки сигналов к контактам
 * из target.cfg
 */

/*
 * Каждому контакту микроконтроллера ставится в соответствие уникальное
 * число, имеющее также и символическое имя. Например, PA0, PC12 и т.д.
 * Пользователь драйвера должен установить значения констант, обозначающих
 * сигналы, в файле target.cfg своего проекта, в переменной CFLAGS.
 * Например,
 *     CFLAGS += -DSSP1RXD=PD11
 * В коде драйвера можно препроцессором разобрать значение константы для
 * сигнала и получить, к какому контакту пользователь привязал сигнал, и
 * правильно проинициализировать контакт.
 * Для примера см. драйвер SPI
 */
#define PORT_A  0
#define PORT_B  1
#define PORT_C  2
#define PORT_D  3
#define PORT_E  4
#define PORT_F  5
#define PORT_G  6
#define PORT_H  7
#define PORT_I  8

#define STM32_PIN(port,pin)    (((port) << 4) | (pin))
#define PORT(x)             (((x) >> 4) & 0xF)
#define PIN(x)              ((x) & 0xF)

#define PA0     STM32_PIN(PORT_A, 0)
#define PA1     STM32_PIN(PORT_A, 1)
#define PA2     STM32_PIN(PORT_A, 2)
#define PA3     STM32_PIN(PORT_A, 3)
#define PA4     STM32_PIN(PORT_A, 4)
#define PA5     STM32_PIN(PORT_A, 5)
#define PA6     STM32_PIN(PORT_A, 6)
#define PA7     STM32_PIN(PORT_A, 7)
#define PA8     STM32_PIN(PORT_A, 8)
#define PA9     STM32_PIN(PORT_A, 9)
#define PA10    STM32_PIN(PORT_A, 10)
#define PA11    STM32_PIN(PORT_A, 11)
#define PA12    STM32_PIN(PORT_A, 12)
#define PA13    STM32_PIN(PORT_A, 13)
#define PA14    STM32_PIN(PORT_A, 14)
#define PA15    STM32_PIN(PORT_A, 15)

#define PB0     STM32_PIN(PORT_B, 0)
#define PB1     STM32_PIN(PORT_B, 1)
#define PB2     STM32_PIN(PORT_B, 2)
#define PB3     STM32_PIN(PORT_B, 3)
#define PB4     STM32_PIN(PORT_B, 4)
#define PB5     STM32_PIN(PORT_B, 5)
#define PB6     STM32_PIN(PORT_B, 6)
#define PB7     STM32_PIN(PORT_B, 7)
#define PB8     STM32_PIN(PORT_B, 8)
#define PB9     STM32_PIN(PORT_B, 9)
#define PB10    STM32_PIN(PORT_B, 10)
#define PB11    STM32_PIN(PORT_B, 11)
#define PB12    STM32_PIN(PORT_B, 12)
#define PB13    STM32_PIN(PORT_B, 13)
#define PB14    STM32_PIN(PORT_B, 14)
#define PB15    STM32_PIN(PORT_B, 15)

#define PC0     STM32_PIN(PORT_C, 0)
#define PC1     STM32_PIN(PORT_C, 1)
#define PC2     STM32_PIN(PORT_C, 2)
#define PC3     STM32_PIN(PORT_C, 3)
#define PC4     STM32_PIN(PORT_C, 4)
#define PC5     STM32_PIN(PORT_C, 5)
#define PC6     STM32_PIN(PORT_C, 6)
#define PC7     STM32_PIN(PORT_C, 7)
#define PC8     STM32_PIN(PORT_C, 8)
#define PC9     STM32_PIN(PORT_C, 9)
#define PC10    STM32_PIN(PORT_C, 10)
#define PC11    STM32_PIN(PORT_C, 11)
#define PC12    STM32_PIN(PORT_C, 12)
#define PC13    STM32_PIN(PORT_C, 13)
#define PC14    STM32_PIN(PORT_C, 14)
#define PC15    STM32_PIN(PORT_C, 15)

#define PD0     STM32_PIN(PORT_D, 0)
#define PD1     STM32_PIN(PORT_D, 1)
#define PD2     STM32_PIN(PORT_D, 2)
#define PD3     STM32_PIN(PORT_D, 3)
#define PD4     STM32_PIN(PORT_D, 4)
#define PD5     STM32_PIN(PORT_D, 5)
#define PD6     STM32_PIN(PORT_D, 6)
#define PD7     STM32_PIN(PORT_D, 7)
#define PD8     STM32_PIN(PORT_D, 8)
#define PD9     STM32_PIN(PORT_D, 9)
#define PD10    STM32_PIN(PORT_D, 10)
#define PD11    STM32_PIN(PORT_D, 11)
#define PD12    STM32_PIN(PORT_D, 12)
#define PD13    STM32_PIN(PORT_D, 13)
#define PD14    STM32_PIN(PORT_D, 14)
#define PD15    STM32_PIN(PORT_D, 15)

#define PE0     STM32_PIN(PORT_E, 0)
#define PE1     STM32_PIN(PORT_E, 1)
#define PE2     STM32_PIN(PORT_E, 2)
#define PE3     STM32_PIN(PORT_E, 3)
#define PE4     STM32_PIN(PORT_E, 4)
#define PE5     STM32_PIN(PORT_E, 5)
#define PE6     STM32_PIN(PORT_E, 6)
#define PE7     STM32_PIN(PORT_E, 7)
#define PE8     STM32_PIN(PORT_E, 8)
#define PE9     STM32_PIN(PORT_E, 9)
#define PE10    STM32_PIN(PORT_E, 10)
#define PE11    STM32_PIN(PORT_E, 11)
#define PE12    STM32_PIN(PORT_E, 12)
#define PE13    STM32_PIN(PORT_E, 13)
#define PE14    STM32_PIN(PORT_E, 14)
#define PE15    STM32_PIN(PORT_E, 15)

#define PF0     STM32_PIN(PORT_F, 0)
#define PF1     STM32_PIN(PORT_F, 1)
#define PF2     STM32_PIN(PORT_F, 2)
#define PF3     STM32_PIN(PORT_F, 3)
#define PF4     STM32_PIN(PORT_F, 4)
#define PF5     STM32_PIN(PORT_F, 5)
#define PF6     STM32_PIN(PORT_F, 6)
#define PF7     STM32_PIN(PORT_F, 7)
#define PF8     STM32_PIN(PORT_F, 8)
#define PF9     STM32_PIN(PORT_F, 9)
#define PF10    STM32_PIN(PORT_F, 10)
#define PF11    STM32_PIN(PORT_F, 11)
#define PF12    STM32_PIN(PORT_F, 12)
#define PF13    STM32_PIN(PORT_F, 13)
#define PF14    STM32_PIN(PORT_F, 14)
#define PF15    STM32_PIN(PORT_F, 15)

#define PG0     STM32_PIN(PORT_G, 0)
#define PG1     STM32_PIN(PORT_G, 1)
#define PG2     STM32_PIN(PORT_G, 2)
#define PG3     STM32_PIN(PORT_G, 3)
#define PG4     STM32_PIN(PORT_G, 4)
#define PG5     STM32_PIN(PORT_G, 5)
#define PG6     STM32_PIN(PORT_G, 6)
#define PG7     STM32_PIN(PORT_G, 7)
#define PG8     STM32_PIN(PORT_G, 8)
#define PG9     STM32_PIN(PORT_G, 9)
#define PG10    STM32_PIN(PORT_G, 10)
#define PG11    STM32_PIN(PORT_G, 11)
#define PG12    STM32_PIN(PORT_G, 12)
#define PG13    STM32_PIN(PORT_G, 13)
#define PG14    STM32_PIN(PORT_G, 14)
#define PG15    STM32_PIN(PORT_G, 15)

#define PH0     STM32_PIN(PORT_H, 0)
#define PH1     STM32_PIN(PORT_H, 1)
#define PH2     STM32_PIN(PORT_H, 2)
#define PH3     STM32_PIN(PORT_H, 3)
#define PH4     STM32_PIN(PORT_H, 4)
#define PH5     STM32_PIN(PORT_H, 5)
#define PH6     STM32_PIN(PORT_H, 6)
#define PH7     STM32_PIN(PORT_H, 7)
#define PH8     STM32_PIN(PORT_H, 8)
#define PH9     STM32_PIN(PORT_H, 9)
#define PH10    STM32_PIN(PORT_H, 10)
#define PH11    STM32_PIN(PORT_H, 11)
#define PH12    STM32_PIN(PORT_H, 12)
#define PH13    STM32_PIN(PORT_H, 13)
#define PH14    STM32_PIN(PORT_H, 14)
#define PH15    STM32_PIN(PORT_H, 15)

#define PI0     STM32_PIN(PORT_I, 0)
#define PI1     STM32_PIN(PORT_I, 1)
#define PI2     STM32_PIN(PORT_I, 2)
#define PI3     STM32_PIN(PORT_I, 3)
#define PI4     STM32_PIN(PORT_I, 4)
#define PI5     STM32_PIN(PORT_I, 5)
#define PI6     STM32_PIN(PORT_I, 6)
#define PI7     STM32_PIN(PORT_I, 7)
#define PI8     STM32_PIN(PORT_I, 8)
#define PI9     STM32_PIN(PORT_I, 9)
#define PI10    STM32_PIN(PORT_I, 10)
#define PI11    STM32_PIN(PORT_I, 11)
#define PI12    STM32_PIN(PORT_I, 12)
#define PI13    STM32_PIN(PORT_I, 13)
#define PI14    STM32_PIN(PORT_I, 14)
#define PI15    STM32_PIN(PORT_I, 15)


#define stm32f4_alt_config_pin(port, pin, funk) {                                                       \
	RCC->AHB1ENR |= (1 << PORT(pin));	/* Включение тактовой порта	*/								\
	/* Настройка порта  */																			\
	port->MODER = (port->MODER & ~GPIO_MODE_MASK(PIN(pin))) | GPIO_ALT(PIN(pin)); 					\
	if (PIN(pin) < 8)																				\
		port->AFRL = (port->AFRL & ~GPIO_AF_MASK(PIN(pin))) | funk;									\
	else																							\
		port->AFRH = (port->AFRH & ~GPIO_AF_MASK(PIN(pin))) | funk;									\
	port->OSPEEDR |= GPIO_100MHz(PIN(pin));															\
	port->OTYPER &= ~GPIO_OD(PIN(pin));																\
	port->PUPDR &= ~GPIO_PULL_MASK(PIN(pin));														\
}


/* Сигнал UART1_TXD */
#ifndef UART1_TX
#define UART1_TX    PB6		/* PA9  */
#endif

#if ((PORT(UART1_TX)==PORT_A) && (PIN(UART1_TX) == 9))
#   define UART1_TX_GPIO 	GPIOA
#elif ((PORT(UART1_TX)==PORT_B) && (PIN(UART1_TX) == 6))
#   define UART1_TX_GPIO 	GPIOB
#else
#   error "UART1_TX pin is not assigned in CFLAGS of target.cfg"
#endif

/* Сигнал UART1_RXD */
#ifndef UART1_RX
#define UART1_RX	PB7		/* PA10 */
#endif
#if ((PORT(UART1_RX)==PORT_A) && (PIN(UART1_RX) == 10))
#   define UART1_RX_GPIO 	GPIOA
#elif ((PORT(UART1_RX)==PORT_B) && (PIN(UART1_RX) == 7))
#   define UART1_RX_GPIO 	GPIOB
#else
#   error "UART1_RX pin is not assigned in CFLAGS of target.cfg"
#endif

/* Сигнал UART2_TXD */
#ifndef UART2_TX
#define UART2_TX	PA2	/* PD5		 PA2 */
#endif
#if ((PORT(UART2_TX)==PORT_A) && (PIN(UART2_TX) == 2))
#   define UART2_TX_GPIO 	GPIOA
#elif ((PORT(UART2_TX)==PORT_D) && (PIN(UART2_TX) == 5))
#   define UART2_TX_GPIO 	GPIOD
#else
#   error "UART2_TX pin is not assigned in CFLAGS of target.cfg"
#endif

/* Сигнал UART2_RXD */
#ifndef UART2_RX
#define UART2_RX	PA3		/* PD6		 PA3 */
#endif
#if ((PORT(UART2_RX)==PORT_A) && (PIN(UART2_RX) == 3))
#   define UART2_RX_GPIO 	GPIOA
#elif ((PORT(UART2_RX)==PORT_D) && (PIN(UART2_RX) == 6))
#   define UART2_RX_GPIO 	GPIOD
#else
#   error "UART2_RX pin is not assigned in CFLAGS of target.cfg"
#endif

/* Сигнал UART3_TXD */
#ifndef UART3_TX
#define UART3_TX	PD8		/* PC10	 PB10  PD8*/
#endif
#if ((PORT(UART3_TX)==PORT_B) && (PIN(UART3_TX) == 10))
#   define UART3_TX_GPIO 	GPIOB
#elif ((PORT(UART3_TX)==PORT_C) && (PIN(UART3_TX) == 10))
#   define UART3_TX_GPIO 	GPIOC
#elif ((PORT(UART3_TX)==PORT_D) && (PIN(UART3_TX) == 8))
#   define UART3_TX_GPIO 	GPIOD
#else
#   error "UART3_TX pin is not assigned in CFLAGS of target.cfg"
#endif

/* Сигнал UART3_RXD */
#ifndef UART3_RX
#define UART3_RX	PD9		/* PC11  	PB11	PD9 */
#endif
#if ((PORT(UART3_RX)==PORT_B) && (PIN(UART3_RX) == 11))
#   define UART3_RX_GPIO 	GPIOB
#elif ((PORT(UART3_RX)==PORT_C) && (PIN(UART3_RX) == 11))
#   define UART3_RX_GPIO 	GPIOC
#elif ((PORT(UART3_RX)==PORT_D) && (PIN(UART3_RX) == 9))
#   define UART3_RX_GPIO 	GPIOD
#else
#   error "UART3_RX pin is not assigned in CFLAGS of target.cfg"
#endif

/* Сигнал UART4_TXD */
#ifndef UART4_TX
#define UART4_TX	PA0		/* PA0	 PC10	 */
#endif
#if ((PORT(UART4_TX)==PORT_A) && (PIN(UART4_TX) == 0))
#   define UART4_TX_GPIO 	GPIOA
#elif ((PORT(UART4_TX)==PORT_C)  && (PIN(UART4_TX) == 11))
#   define UART4_TX_GPIO 	GPIOC
#else
#   error "UART4_TX pin is not assigned in CFLAGS of target.cfg"
#endif

/* Сигнал UART4_RXD */
#ifndef UART4_RX
#define UART4_RX	PA1		/* PA1	 PC11  */
#endif
#if ((PORT(UART4_RX)==PORT_A) && (PIN(UART4_RX) == 1))
#   define UART4_RX_GPIO 	GPIOA
#elif ( (PORT(UART4_RX)==PORT_C) && (PIN(UART4_RX) == 11))
#   define UART4_RX_GPIO 	GPIOC
#else
#   error "UART4_RX pin is not assigned in CFLAGS of target.cfg"
#endif

/* Сигнал UART5_TXD */
#ifndef UART5_TX
#define UART5_TX	PC12
#endif
#if ((PORT(UART5_TX)==PORT_C) && (PIN(UART5_TX) == 12))
#   define UART5_TX_GPIO 	GPIOC
#else
#   error "UART5_TX pin is not assigned in CFLAGS of target.cfg"
#endif

/* Сигнал UART5_RXD */
#ifndef UART5_RX
#define UART5_RX	PD2
#endif
#if ((PORT(UART5_RX)==PORT_D)  && (PIN(UART5_RX) == 2))
#   define UART5_RX_GPIO 	GPIOD
#else
#   error "UART5_RX pin is not assigned in CFLAGS of target.cfg"
#endif

/* Сигнал UART6_TXD */
#ifndef UART6_TX
#define UART6_TX	PC6		/* PC6	 PG14	 */
#endif
#if ((PORT(UART6_TX)==PORT_C)  && (PIN(UART6_TX) == 6))
#   define UART6_TX_GPIO 	GPIOC
#elif ((PORT(UART6_TX)==PORT_G)  && (PIN(UART6_TX) == 14))
#   define UART6_TX_GPIO 	GPIOG
#else
#   error "UART6_TX pin is not assigned in CFLAGS of target.cfg"
#endif

/* Сигнал UART6_RXD */
#ifndef UART6_RX
#define UART6_RX	PC7		/* PC7	 PG9  */
#endif
#if ((PORT(UART6_RX)==PORT_C) && (PIN(UART6_RX) == 7))
#   define UART6_RX_GPIO 	GPIOC
#elif ((PORT(UART6_RX)==PORT_G)  && (PIN(UART6_RX) == 9))
#   define UART6_RX_GPIO 	GPIOG
#else
#   error "UART6_RX pin is not assigned in CFLAGS of target.cfg"
#endif

/* SPI1_CSS  AF5*/
#ifndef SPI1_CSS
# define SPI1_CSS		PA4		/* PA15 */
#endif

#if ((PORT(SPI1_CSS)==PORT_A)  && (PIN(SPI1_CSS) == 4))
#   define SPI1_CSS_GPIO 	GPIOA
#elif ((PORT(SPI1_CSS)==PORT_A)  && (PIN(SPI1_CSS) == 15))
#   define SPI1_CSS_GPIO 	GPIOA
#else
#   error "SPI1_CSS pin is not assigned in CFLAGS of target.cfg"
#endif


/* SPI1_CLK */
#ifndef SPI1_CLK
# define SPI1_CLK		PA5		/* PB3 */
#endif

#if ((PORT(SPI1_CLK)==PORT_A)  && (PIN(SPI1_CLK) == 5))
#   define SPI1_CLK_GPIO 	GPIOA
#elif ((PORT(SPI1_CLK)==PORT_B)  && (PIN(SPI1_CLK) == 3))
#   define SPI1_CLK_GPIO 	GPIOB
#else
#   error "SPI1_CLK pin is not assigned in CFLAGS of target.cfg"
#endif


/* SPI1_MISO */
#ifndef SPI1_MISO
# define SPI1_MISO		PA6		/* PB4 */
#endif

#if ((PORT(SPI1_MISO)==PORT_A)  && (PIN(SPI1_MISO) == 6))
#   define SPI1_MISO_GPIO 	GPIOA
#elif ((PORT(SPI1_MISO)==PORT_B)  && (PIN(SPI1_MISO) == 4))
#   define SPI1_MISO_GPIO 	GPIOB
#else
#   error "SPI1_MISO pin is not assigned in CFLAGS of target.cfg"
#endif


/* SPI1_MOSI */
#ifndef SPI1_MOSI
# define SPI1_MOSI		PA7		/* PB5 */
#endif

#if ((PORT(SPI1_MOSI)==PORT_A)  && (PIN(SPI1_MOSI) == 7))
#   define SPI1_MOSI_GPIO 	GPIOA
#elif ((PORT(SPI1_MOSI)==PORT_B)  && (PIN(SPI1_MOSI) == 5))
#   define SPI1_MOSI_GPIO 	GPIOB
#else
#   error "SPI1_MOSI pin is not assigned in CFLAGS of target.cfg"
#endif



/* SPI2_CSS  AF5*/
#ifndef SPI2_CSS
# define SPI2_CSS		PB9		/* PB12   PI0 */
#endif

#if ((PORT(SPI2_CSS)==PORT_B)  && (PIN(SPI2_CSS) == 9))
#   define SPI2_CSS_GPIO 	GPIOB
#elif ((PORT(SPI2_CSS)==PORT_B)  && (PIN(SPI2_CSS) == 12))
#   define SPI2_CSS_GPIO 	GPIOB
#elif ((PORT(SPI2_CSS)==PORT_I)  && (PIN(SPI2_CSS) == 0))
#   define SPI2_CSS_GPIO 	GPIOI
#else
#   error "SPI2_CSS pin is not assigned in CFLAGS of target.cfg"
#endif


/* SPI2_CLK */
#ifndef SPI2_CLK
# define SPI2_CLK		PB10	/* PB13   PI1 */
#endif

#if ((PORT(SPI2_CLK)==PORT_B)  && (PIN(SPI2_CLK) == 10))
#   define SPI2_CLK_GPIO 	GPIOB
#elif ((PORT(SPI2_CLK)==PORT_B)  && (PIN(SPI2_CLK) == 13))
#   define SPI2_CLK_GPIO 	GPIOB
#elif ((PORT(SPI2_CLK)==PORT_I)  && (PIN(SPI2_CLK) == 1))
#   define SPI2_CLK_GPIO 	GPIOI
#else
#   error "SPI2_CLK pin is not assigned in CFLAGS of target.cfg"
#endif


/* SPI2_MISO */
#ifndef SPI2_MISO
# define SPI2_MISO		PB14	/* PC2    PI2 */
#endif

#if ((PORT(SPI2_MISO)==PORT_B)  && (PIN(SPI2_MISO) == 14))
#   define SPI2_MISO_GPIO 	GPIOB
#elif ((PORT(SPI2_MISO)==PORT_C)  && (PIN(SPI2_MISO) == 2))
#   define SPI2_MISO_GPIO 	GPIOC
#elif ((PORT(SPI2_MISO)==PORT_I)  && (PIN(SPI2_MISO) == 2))
#   define SPI2_MISO_GPIO 	GPIOI
#else
#   error "SPI2_MISO pin is not assigned in CFLAGS of target.cfg"
#endif


/* SPI2_MOSI */
#ifndef SPI2_MOSI
# define SPI2_MOSI		PB15	/* PC3    PI3 */
#endif

#if ((PORT(SPI2_MOSI)==PORT_B)  && (PIN(SPI2_MOSI) == 15))
#   define SPI2_MOSI_GPIO 	GPIOB
#elif ((PORT(SPI2_MOSI)==PORT_C)  && (PIN(SPI2_MOSI) == 3))
#   define SPI2_MOSI_GPIO 	GPIOC
#elif ((PORT(SPI2_MOSI)==PORT_I)  && (PIN(SPI2_MOSI) == 3))
#   define SPI2_MOSI_GPIO 	GPIOI
#else
#   error "SPI2_MOSI pin is not assigned in CFLAGS of target.cfg"
#endif

//////////
/* SPI3_CSS  AF5*/
#ifndef SPI3_CSS
# define SPI3_CSS		PA4		/* PA15 */
#endif

#if ((PORT(SPI3_CSS)==PORT_A)  && (PIN(SPI3_CSS) == 4))
#   define SPI3_CSS_GPIO 	GPIOA
#elif ((PORT(SPI3_CSS)==PORT_A)  && (PIN(SPI3_CSS) == 15))
#   define SPI3_CSS_GPIO 	GPIOA
#else
#   error "SPI3_CSS pin is not assigned in CFLAGS of target.cfg"
#endif


/* SPI3_CLK */
#ifndef SPI3_CLK
# define SPI3_CLK		PB3	/* PC10 */
#endif

#if ((PORT(SPI3_CLK)==PORT_B)  && (PIN(SPI3_CLK) == 3))
#   define SPI3_CLK_GPIO 	GPIOB
#elif ((PORT(SPI3_CLK)==PORT_C)  && (PIN(SPI3_CLK) == 10))
#   define SPI3_CLK_GPIO 	GPIOC
#else
#   error "SPI3_CLK pin is not assigned in CFLAGS of target.cfg"
#endif


/* SPI3_MISO */
#ifndef SPI3_MISO
# define SPI3_MISO		PB4	/* PC11 */
#endif

#if ((PORT(SPI3_MISO)==PORT_B)  && (PIN(SPI3_MISO) == 4))
#   define SPI3_MISO_GPIO 	GPIOB
#elif ((PORT(SPI3_MISO)==PORT_C)  && (PIN(SPI3_MISO) == 11))
#   define SPI3_MISO_GPIO 	GPIOC
#else
#   error "SPI3_MISO pin is not assigned in CFLAGS of target.cfg"
#endif


/* SPI3_MOSI */
#ifndef SPI3_MOSI
# define SPI3_MOSI		PB5	/* PC12 */
#endif

#if ((PORT(SPI3_MOSI)==PORT_B)  && (PIN(SPI3_MOSI) == 5))
#   define SPI3_MOSI_GPIO 	GPIOB
#elif ((PORT(SPI3_MOSI)==PORT_C)  && (PIN(SPI3_MOSI) == 12))
#   define SPI3_MOSI_GPIO 	GPIOC
#else
#   error "SPI3_MOSI pin is not assigned in CFLAGS of target.cfg"
#endif

/* I2C1_SCL */
#ifndef I2C1_SCL
# define I2C1_SCL		PB6 	/* PB8 */
#endif
#if ((PORT(I2C1_SCL)==PORT_B)  && (PIN(I2C1_SCL) == 6))
#   define I2C1_SCL_GPIO 	GPIOB
#elif ((PORT(I2C1_SCL)==PORT_B)  && (PIN(I2C1_SCL) == 8))
#   define I2C1_SCL_GPIO 	GPIOB
#else
#   error "I2C1_SCL pin is not assigned in CFLAGS of target.cfg"
#endif

/* I2C1_SDA */
#ifndef I2C1_SDA
# define I2C1_SDA		PB7		/* PB9 */
#endif
#if ((PORT(I2C1_SDA)==PORT_B)  && (PIN(I2C1_SDA) == 7))
#   define I2C1_SDA_GPIO 	GPIOB
#elif ((PORT(I2C1_SDA)==PORT_B)  && (PIN(I2C1_SDA) == 9))
#   define I2C1_SDA_GPIO 	GPIOB
#else
#   error "I2C1_SDA pin is not assigned in CFLAGS of target.cfg"
#endif

/* I2C2_SCL */
#ifndef I2C2_SCL
# define I2C2_SCL		PB10	/* PF1   PH4*/
#endif
#if ((PORT(I2C2_SCL)==PORT_B)  && (PIN(I2C2_SCL) == 10))
#   define I2C2_SCL_GPIO 	GPIOB
#elif ((PORT(I2C2_SCL)==PORT_F)  && (PIN(I2C2_SCL) == 1))
#   define I2C2_SCL_GPIO 	GPIOF
#elif ((PORT(I2C2_SCL)==PORT_H)  && (PIN(I2C2_SCL) == 4))
#   define I2C2_SCL_GPIO 	GPIOH
#else
#   error "I2C2_SCL pin is not assigned in CFLAGS of target.cfg"
#endif

/* I2C2_SDA */
#ifndef I2C2_SDA
# define I2C2_SDA		PB11	/* PF0   PH5*/
#endif
#if ((PORT(I2C2_SDA)==PORT_B)  && (PIN(I2C2_SDA) == 11))
#   define I2C2_SDA_GPIO 	GPIOB
#elif ((PORT(I2C2_SDA)==PORT_F)  && (PIN(I2C2_SDA) == 0))
#   define I2C2_SCL_GPIO 	GPIOF
#elif ((PORT(I2C2_SDA)==PORT_H)  && (PIN(I2C2_SDA) == 5))
#   define I2C2_SDA_GPIO 	GPIOH
#else
#   error "I2C2_SDA pin is not assigned in CFLAGS of target.cfg"
#endif

/* I2C3_SCL */
#ifndef I2C3_SCL
# define I2C3_SCL		PA8		/* PH7 */
#endif
#if ((PORT(I2C3_SCL)==PORT_A)  && (PIN(I2C3_SCL) == 8))
#   define I2C3_SCL_GPIO 	GPIOA
#elif ((PORT(I2C3_SCL)==PORT_H)  && (PIN(I2C3_SCL) == 7))
#   define I2C3_SCL_GPIO 	GPIOH
#else
#   error "I2C3_SCL pin is not assigned in CFLAGS of target.cfg"
#endif

/* I2C3_SDA */
#ifndef I2C3_SDA
# define I2C3_SDA		PC10	/* PH8 */
#endif
#if ((PORT(I2C3_SDA)==PORT_C)  && (PIN(I2C3_SDA) == 10))
#   define I2C3_SDA_GPIO 	GPIOC
#elif ((PORT(I2C3_SDA)==PORT_H)  && (PIN(I2C3_SDA) == 8))
#   define I2C3_SDA_GPIO 	GPIOH
#else
#   error "I2C3_SDA pin is not assigned in CFLAGS of target.cfg"
#endif

