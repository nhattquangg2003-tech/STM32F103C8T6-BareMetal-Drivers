/*
 * STM32F103C8T6.h
 *
 *  Created on: FEB 9, 2026
 *      Author: Owner
 */
#include <stdint.h>
#include <stddef.h>
#ifndef INC_STM32F103C8T6_H_
#define INC_STM32F103C8T6_H_

#define __weak __attribute__((weak))

/***********************************START: PROCESSOR SPECIFIC DETAILS *****************************/
/*
 *
 *
 * ARM CORTEX MX PROCESSOR NVIC ISERx REGISTER ADDRESSES
 *
 * */
// ĐỊA CHỈ CỦA NVIC REGISTER, CÁC THANH NÀY CONFIG CÁC VỊ TRÍ IRQ
#define NVIC_ISER0   ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1   ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2   ((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3   ((volatile uint32_t*)0xE000E10C)
/*
 *
 *
 * ARM CORTEX MX PROCESSOR NVIC ICERx REGISTER ADDRESSES
 *
 * */
#define NVIC_ICER0   ((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1   ((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2   ((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3   ((volatile uint32_t*)0xE000E18C)
/*
 *
 *
 * ARM CORTEX MX PROCESSOR Priority Register Address Calculation
 *
 * */
#define NVIC_PR_BASE_ADDR ((volatile uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENT 				4
// BASE ADDRESS OF fLASH AND SRAM MEMORIES

#define FLASH_BASEADDR                             0x08000000U
#define SRAM1_BASEADDR                              0x20000000U
#define ROM_BASEADDR                                 0x1FFFF000U
#define SRAM                                        SRAM1_BASEADDR

// AHBx AND APBx BUS PERIPHERAL BASE ADDRESSES

#define PERIPH_BASE                                   0x40000000U
#define APB1PERIPH_BASE                               PERIPH_BASE
#define APB2PERIPH_BASE                               0x40010000U
#define AHBPERIPH_BASE                                0x40018000U

/* APB1 Peripheral base addresses */
#define TIM2_BASEADDR     (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASEADDR     (APB1PERIPH_BASE + 0x0400)
#define TIM4_BASEADDR     (APB1PERIPH_BASE + 0x0800)
#define TIM5_BASEADDR     (APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASEADDR     (APB1PERIPH_BASE + 0x1000)
#define TIM7_BASEADDR     (APB1PERIPH_BASE + 0x1400)
#define WWDG_BASEADDR     (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASEADDR     (APB1PERIPH_BASE + 0x3000)
#define SPI2_BASEADDR     (APB1PERIPH_BASE + 0x3800)
#define USART2_BASEADDR   (APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR   (APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR    (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR    (APB1PERIPH_BASE + 0x5000)
#define I2C1_BASEADDR     (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR     (APB1PERIPH_BASE + 0x5800)
#define CAN1_BASEADDR     (APB1PERIPH_BASE + 0x6400)
#define BKP_BASEADDR      (APB1PERIPH_BASE + 0x6C00)
#define PWR_BASEADDR      (APB1PERIPH_BASE + 0x7000)
#define DAC_BASEADDR      (APB1PERIPH_BASE + 0x7400)

/* APB2 Peripheral base addresses */
#define AFIO_BASEADDR     (APB2PERIPH_BASE + 0x0000)
#define EXTI_BASEADDR     (APB2PERIPH_BASE + 0x0400)
#define GPIOA_BASEADDR    (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASEADDR    (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASEADDR    (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASEADDR    (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASEADDR    (APB2PERIPH_BASE + 0x1800)
#define ADC1_BASEADDR     (APB2PERIPH_BASE + 0x2400)
#define ADC2_BASEADDR     (APB2PERIPH_BASE + 0x2800)
#define TIM1_BASEADDR     (APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASEADDR     (APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR   (APB2PERIPH_BASE + 0x3800)

/* AHB Peripheral base addresses */
#define DMA1_BASEADDR     (AHBPERIPH_BASE + 0x0000)
#define DMA2_BASEADDR     (AHBPERIPH_BASE + 0x0400)
#define RCC_BASEADDR      0x40021000U
#define FLASH_BASEADDR1    (AHBPERIPH_BASE + 0x2000)
#define CRC_BASEADDR      (AHBPERIPH_BASE + 0x3000)


/***********************************PERIPHERAL REGISTER DEFINITION STRUCTURES *****************************/
/* GPIO register definition structure */
typedef struct
{
  volatile uint32_t CRL;   // Configuration register low (pins 0–7)
  volatile uint32_t CRH;   // Configuration register high (pins 8–15)
  volatile uint32_t IDR;   // Input data register
  volatile uint32_t ODR;   // Output data register
  volatile uint32_t BSRR;  // Bit set/reset register
  volatile uint32_t BRR;   // Bit reset register
  volatile uint32_t LCKR;  // Configuration lock register
} GPIO_RegDef_t;

// GPIO Modes (MODE[1:0])
#define GPIO_MODE_INPUT        0x0
#define GPIO_MODE_OUTPUT_10MHZ 0x1
#define GPIO_MODE_OUTPUT_2MHZ  0x2
#define GPIO_MODE_OUTPUT_50MHZ 0x3

// GPIO Configurations (CNF[1:0])
// Input
#define GPIO_CNF_INPUT_ANALOG      0x0
#define GPIO_CNF_INPUT_FLOATING    0x1
#define GPIO_CNF_INPUT_PUPD        0x2
// Output
#define GPIO_CNF_OUTPUT_PP         0x0
#define GPIO_CNF_OUTPUT_OD         0x1
#define GPIO_CNF_AF_PP             0x2
#define GPIO_CNF_AF_OD             0x3
// Pin numbers
#define GPIO_PIN_0   0
#define GPIO_PIN_1   1
#define GPIO_PIN_2   2
#define GPIO_PIN_3   3
#define GPIO_PIN_4   4
#define GPIO_PIN_5   5
#define GPIO_PIN_6   6
#define GPIO_PIN_7   7
#define GPIO_PIN_8   8
#define GPIO_PIN_9   9
#define GPIO_PIN_10  10
#define GPIO_PIN_11  11
#define GPIO_PIN_12  12
#define GPIO_PIN_13  13
#define GPIO_PIN_14  14
#define GPIO_PIN_15  15


/*
 * peripheral definitions
 */

#define GPIOA ( (GPIO_RegDef_t*) GPIOA_BASEADDR )
#define GPIOB ( (GPIO_RegDef_t*) GPIOB_BASEADDR )
#define GPIOC ( (GPIO_RegDef_t*) GPIOC_BASEADDR )
#define GPIOD ( (GPIO_RegDef_t*) GPIOD_BASEADDR )
#define GPIOE ( (GPIO_RegDef_t*) GPIOE_BASEADDR )

/*
 * interupt config -> CONFIG EXTI thông qua AFIO
 */
/* Base addresses (theo RM0008) */

#define AFIO_BASE         AFIO_BASEADDR
#define EXTI_BASE         EXTI_BASEADDR

/* Struct định nghĩa thanh ghi AFIO */
typedef struct {
	volatile uint32_t EVCR;
	volatile uint32_t MAPR;
	volatile uint32_t EXTICR[4];
	volatile uint32_t RESERVED0;
    volatile uint32_t MAPR2;
} AFIO_TypeDef;

/* Struct định nghĩa thanh ghi EXTI */
typedef struct {
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
} EXTI_TypeDef;

/* Macro trỏ tới địa chỉ base */
#define AFIO    ((AFIO_TypeDef *) AFIO_BASE)
#define EXTI    ((EXTI_TypeDef *) EXTI_BASE)

// GPIO interrupt mode (dùng cho EXTI)
#define GPIO_MODE_IT_FT 0x10 // Falling trigger
#define GPIO_MODE_IT_RT 0x11 // Rising trigger
#define GPIO_MODE_IT_RFT 0x12 // Rising + Falling
// Macro để map port sang code trong AFIO->EXTICR
#define GPIO_PORTCODE(x)       ((x == GPIOA) ? 0 : \
                                (x == GPIOB) ? 1 : \
                                (x == GPIOC) ? 2 : \
                                (x == GPIOD) ? 3 : \
                                (x == GPIOE) ? 4 : 0)
/*
 * CONFIG IRQ => NVIC
 * TRA BẲNG VECTOR NVIC TRONG REFERENCE PHẦN POSITION
 */
// VỊ TRÍ CỦA CÁC IRQ
#define EXTI0_IRQn      6
#define EXTI1_IRQn      7
#define EXTI2_IRQn      8
#define EXTI3_IRQn      9
#define EXTI4_IRQn      10
#define EXTI9_5_IRQn    23
#define EXTI15_10_IRQn  40

// spi
#define SPI1_IRQn 35
#define SPI2_IRQn 36
// I2C IRQ Numbers (RM0008, NVIC vector table)
#define I2C1_EV_IRQn   31   // I2C1 Event interrupt
#define I2C1_ER_IRQn   32   // I2C1 Error interrupt
#define I2C2_EV_IRQn   33   // I2C2 Event interrupt
#define I2C2_ER_IRQn   34   // I2C2 Error interrupt
// ======================= UART / USART IRQ Numbers =======================
// Theo RM0008, NVIC vector table
#define USART1_IRQn   37   // USART1 global interrupt
#define USART2_IRQn   38   // USART2 global interrupt
#define USART3_IRQn   39   // USART3 global interrupt
#define UART4_IRQn    52   // UART4 global interrupt
#define UART5_IRQn    53   // UART5 global interrupt
// PRIORITY IRQ
#define NVIC_IRQ_PRI0 		0
#define NVIC_IRQ_PRI1 		1
#define NVIC_IRQ_PRI2 		2
#define NVIC_IRQ_PRI3 		3
#define NVIC_IRQ_PRI4 		4
#define NVIC_IRQ_PRI5 		5
#define NVIC_IRQ_PRI6 		6
#define NVIC_IRQ_PRI7 		7
#define NVIC_IRQ_PRI8 		8
#define NVIC_IRQ_PRI9 		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14 		14
#define NVIC_IRQ_PRI15 		15
/*
 * Peripheral REGISTER definition structure for RCC
 */

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFGR;      // Clock configuration register
  volatile uint32_t CIR;       // Clock interrupt register
  volatile uint32_t APB2RSTR;  // APB2 peripheral reset register
  volatile uint32_t APB1RSTR;  // APB1 peripheral reset register
  volatile uint32_t AHBENR;    // AHB peripheral clock enable register
  volatile uint32_t APB2ENR;   // APB2 peripheral clock enable register
  volatile uint32_t APB1ENR;   // APB1 peripheral clock enable register
  volatile uint32_t BDCR;      // Backup domain control register
  volatile uint32_t CSR;       // Control/status register
} RCC_RegDef_t;
/*
 * peripheral definitions
 */
#define RCC   ((RCC_RegDef_t*) RCC_BASEADDR)


/*
 * CLOCK ENABLE MACROS FOR GPIOx PERIPHERALS
 */

/* Macros to enable peripheral clock for GPIO ports on APB2 */

#define GPIOA_PCLK_EN()   ( RCC->APB2ENR |= (1 << 2) )  // Bit 2: IOPAEN - Enable clock for GPIOA
#define GPIOB_PCLK_EN()   ( RCC->APB2ENR |= (1 << 3) )  // Bit 3: IOPBEN - Enable clock for GPIOB
#define GPIOC_PCLK_EN()   ( RCC->APB2ENR |= (1 << 4) )  // Bit 4: IOPCEN - Enable clock for GPIOC
#define GPIOD_PCLK_EN()   ( RCC->APB2ENR |= (1 << 5) )  // Bit 5: IOPDEN - Enable clock for GPIOD
#define GPIOE_PCLK_EN()   ( RCC->APB2ENR |= (1 << 6) )  // Bit 6: IOPEEN - Enable clock for GPIOE

// enable clock for AFIO
#define AFIO_PCLK_EN()   ( RCC->APB2ENR |= (1 << 0) )  // Bit 0: afio - Enable clock for afio
/*
 * CLOCK ENABLE MACROS FOR I2Cx PERIPHERALS
 */
/* I2C (at APB1 bus)
Bit 21 → I2C1EN
Bit 22 → I2C2EN  */

#define I2C1_PCLK_EN()   ( RCC->APB1ENR |= (1 << 21) )  // Enable clock for I2C1
#define I2C2_PCLK_EN()   ( RCC->APB1ENR |= (1 << 22) )  // Enable clock for I2C2



/*
 * CLOCK ENABLE MACROS FOR SPIx PERIPHERALS
 */
/*
SPI1 at APB2 bus → Bit 12 (SPI1EN)
SPI2 at APB1 bus → Bit 14 (SPI2EN)
 */
#define SPI1_PCLK_EN()   ( RCC->APB2ENR |= (1 << 12) )  // Enable clock for SPI1
#define SPI2_PCLK_EN()   ( RCC->APB1ENR |= (1 << 14) )  // Enable clock for SPI2

/*
 * CLOCK ENABLE MACROS FOR USARTx PERIPHERALS
 */
/*
 * USART1 at APB2 bus → Bit 14 (USART1EN)

USART2 at APB1 bus → Bit 17 (USART2EN)

USART3 at APB1 bus → Bit 18 (USART3EN)

UART4 at APB1 bus → Bit 19 (UART4EN)

UART5 at APB1 bus → Bit 20 (UART5EN)
 */

#define USART1_PCLK_EN()   ( RCC->APB2ENR |= (1 << 14) )  // Enable clock for USART1
#define USART2_PCLK_EN()   ( RCC->APB1ENR |= (1 << 17) )  // Enable clock for USART2
#define USART3_PCLK_EN()   ( RCC->APB1ENR |= (1 << 18) )  // Enable clock for USART3
#define UART4_PCLK_EN()    ( RCC->APB1ENR |= (1 << 19) )  // Enable clock for UART4
#define UART5_PCLK_EN()    ( RCC->APB1ENR |= (1 << 20) )  // Enable clock for UART5


/*
 * CLOCK ENABLE MACROS FOR AFIO PERIPHERAL (STM32F1 series)
 */
#define AFIO_PCLK_EN()   ( RCC->APB2ENR |= (1 << 0) )  // Bit 0: AFIOEN - Enable clock for AFIO

/* CLOCK DISABLE MACROS FOR GPIOx PERIPHERALS */

#define GPIOA_PCLK_DI()   ( RCC->APB2ENR &= ~(1 << 2) )  // Bit 2: IOPAEN
#define GPIOB_PCLK_DI()   ( RCC->APB2ENR &= ~(1 << 3) )  // Bit 3: IOPBEN
#define GPIOC_PCLK_DI()   ( RCC->APB2ENR &= ~(1 << 4) )  // Bit 4: IOPCEN
#define GPIOD_PCLK_DI()   ( RCC->APB2ENR &= ~(1 << 5) )  // Bit 5: IOPDEN
#define GPIOE_PCLK_DI()   ( RCC->APB2ENR &= ~(1 << 6) )  // Bit 6: IOPEEN

/* CLOCK DISABLE MACROS FOR I2Cx PERIPHERALS */

#define I2C1_PCLK_DI()    ( RCC->APB1ENR &= ~(1 << 21) ) // Bit 21: I2C1EN
#define I2C2_PCLK_DI()    ( RCC->APB1ENR &= ~(1 << 22) ) // Bit 22: I2C2EN

/* CLOCK DISABLE MACROS FOR SPIx PERIPHERALS */

#define SPI1_PCLK_DI()    ( RCC->APB2ENR &= ~(1 << 12) ) // Bit 12: SPI1EN
#define SPI2_PCLK_DI()    ( RCC->APB1ENR &= ~(1 << 14) ) // Bit 14: SPI2EN

/* CLOCK DISABLE MACROS FOR USARTx PERIPHERALS */

#define USART1_PCLK_DI()  ( RCC->APB2ENR &= ~(1 << 14) ) // Bit 14: USART1EN
#define USART2_PCLK_DI()  ( RCC->APB1ENR &= ~(1 << 17) ) // Bit 17: USART2EN
#define USART3_PCLK_DI()  ( RCC->APB1ENR &= ~(1 << 18) ) // Bit 18: USART3EN
#define UART4_PCLK_DI()   ( RCC->APB1ENR &= ~(1 << 19) ) // Bit 19: UART4EN
#define UART5_PCLK_DI()   ( RCC->APB1ENR &= ~(1 << 20) ) // Bit 20: UART5EN

/* CLOCK DISABLE MACROS FOR AFIO (STM32F1) */
#define AFIO_PCLK_DI()    ( RCC->APB2ENR &= ~(1 << 0) )  // Bit 0: AFIOEN

/* CLOCK DISABLE MACROS FOR SYSCFG (STM32F4/F7) */
#define SYSCFG_PCLK_DI()  ( RCC->APB2ENR &= ~(1 << 14) ) // Bit 14: SYSCFGEN


/*
 * MACRO TO RESET GPIOx PERIPHERALS
 *
 */
#define GPIOA_REG_RESET()   do{ (RCC->APB2RSTR |= (1 << 2)); \
                                (RCC->APB2RSTR &= ~(1 << 2)); }while(0)

#define GPIOB_REG_RESET()   do{ (RCC->APB2RSTR |= (1 << 3)); \
                                (RCC->APB2RSTR &= ~(1 << 3)); }while(0)

#define GPIOC_REG_RESET()   do{ (RCC->APB2RSTR |= (1 << 4)); \
                                (RCC->APB2RSTR &= ~(1 << 4)); }while(0)

#define GPIOD_REG_RESET()   do{ (RCC->APB2RSTR |= (1 << 5)); \
                                (RCC->APB2RSTR &= ~(1 << 5)); }while(0)

#define GPIOE_REG_RESET()   do{ (RCC->APB2RSTR |= (1 << 6)); \
                                (RCC->APB2RSTR &= ~(1 << 6)); }while(0)

/*
 * MACRO TO RESET SPIx PERIPHERALS
 *
 */

#define SPI1_REG_RESET()   do{ (RCC->APB2RSTR |= (1 << 12)); \
                               (RCC->APB2RSTR &= ~(1 << 12)); }while(0)

#define SPI2_REG_RESET()   do{ (RCC->APB1RSTR |= (1 << 14)); \
                               (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
// SOME GENERIC MACRO

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET
#define FLAG_RESET RESET
#define FLAG_SET SET
/*
 * Peripheral REGISTER definition structure for SPI
 */
typedef struct
{
    volatile uint32_t CR1;       // Control register 1
    volatile uint32_t CR2;       // Control register 2
    volatile uint32_t SR;        // Status register
    volatile uint32_t DR;        // Data register
    volatile uint32_t CRCPR;     // CRC polynomial register
    volatile uint32_t RXCRCR;    // RX CRC register
    volatile uint32_t TXCRCR;    // TX CRC register
    volatile uint32_t I2SCFGR;   // I2S configuration register
    volatile uint32_t I2SPR;     // I2S prescaler register
} SPI_RegDef_t;
/*
 * peripheral definitions for SPI, MACRO
 */
#define SPI1      ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2      ((SPI_RegDef_t*)SPI2_BASEADDR)
/*
 * BIT POSITION DEFINITIONS OF CR1 SPI PEROPHERAL
 */
#define SPI_CR1_CPHA 		0
#define SPI_CR1_CPOL 		1
#define SPI_CR1_MSTR 		2
#define SPI_CR1_BR 	 		3
#define SPI_CR1_SPE  		6
#define SPI_CR1_LSBFIRST 	7
#define SPI_CR1_SSI 		8
#define SPI_CR1_SSM 		9
#define SPI_CR1_RXONLY 		10
#define SPI_CR1_DFF 		11
#define SPI_CR1_CRCNEXT 	12
#define SPI_CR1_CRCEN 		13
#define SPI_CR1_BIDIOE 		14
#define SPI_CR1_BIDIMODE 	15
/*
 * BIT POSITION DEFINITIONS OF CR2 SPI PEROPHERAL
 */
#define SPI_CR2_RXDMAEN 	0
#define SPI_CR2_TXDMAEN 	1
#define SPI_CR2_SSOE 		2
#define SPI_CR2_ERRIE 		5
#define SPI_CR2_RXNEIE 		6
#define SPI_CR2_TXEIE 		7
/*
 * BIT POSITION DEFINITIONS OF SR SPI PEROPHERAL
 */
#define SPI_SR_RXNE 	0
#define SPI_SR_TXE 		1
#define SPI_SR_CHSIDE 	2
#define SPI_SR_UDR 		3
#define SPI_SR_CRCERR 	4
#define SPI_SR_MODF 	5
#define SPI_SR_OVR 		6
#define SPI_SR_BSY 		7
/*
 * peripheral definitions for I2C
 */
typedef struct
{
    volatile uint32_t CR1;     /*!< I2C Control register 1,        Address offset: 0x00 */
    volatile uint32_t CR2;     /*!< I2C Control register 2,        Address offset: 0x04 */
    volatile uint32_t OAR1;    /*!< I2C Own address register 1,    Address offset: 0x08 */
    volatile uint32_t OAR2;    /*!< I2C Own address register 2,    Address offset: 0x0C */
    volatile uint32_t DR;      /*!< I2C Data register,             Address offset: 0x10 */
    volatile uint32_t SR1;     /*!< I2C Status register 1,         Address offset: 0x14 */
    volatile uint32_t SR2;     /*!< I2C Status register 2,         Address offset: 0x18 */
    volatile uint32_t CCR;     /*!< I2C Clock control register,    Address offset: 0x1C */
    volatile uint32_t TRISE;   /*!< I2C TRISE register,            Address offset: 0x20 */
} I2C_RegDef_t;

#define I2C1      ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2      ((I2C_RegDef_t*)I2C2_BASEADDR)
/*
 * Bit position definitions of I2C peripheral
 */

/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE           0   // Peripheral enable
#define I2C_CR1_SMBUS        1   // SMBus mode
#define I2C_CR1_SMBTYPE      3   // SMBus type
#define I2C_CR1_ENARP        4   // ARP enable
#define I2C_CR1_ENPEC        5   // PEC enable
#define I2C_CR1_ENGC         6   // General call enable
#define I2C_CR1_NOSTRETCH    7   // Clock stretching disable (slave mode)
#define I2C_CR1_START        8   // Start generation
#define I2C_CR1_STOP         9   // Stop generation
#define I2C_CR1_ACK          10  // Acknowledge enable
#define I2C_CR1_POS          11  // ACK/PEC position (for data reception)
#define I2C_CR1_PEC          12  // Packet error checking
#define I2C_CR1_ALERT        13  // SMBus alert
#define I2C_CR1_SWRST        15  // Software reset

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ         0   // Bits [5:0] Peripheral clock frequency
#define I2C_CR2_ITERREN      8   // Error interrupt enable
#define I2C_CR2_ITEVTEN      9   // Event interrupt enable
#define I2C_CR2_ITBUFEN      10  // Buffer interrupt enable
#define I2C_CR2_DMAEN        11  // DMA requests enable
#define I2C_CR2_LAST         12  // DMA last transfer
/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0        0    // Address bit0 (for 10-bit addressing)
#define I2C_OAR1_ADD7_1      1    // Bits [7:1] of address (7-bit mode)
#define I2C_OAR1_ADD9_8      8    // Bits [9:8] of address (10-bit mode)
#define I2C_OAR1_ADDMODE     15   // Addressing mode (0: 7-bit, 1: 10-bit)

/*
 * Bit position definitions I2C_OAR2
 */
#define I2C_OAR2_ENDUAL      0    // Dual addressing mode enable
#define I2C_OAR2_ADD2        1    // Bits [7:1] second address

/*
 * Bit position definitions I2C_SR1
 */
#define I2C_SR1_SB           0    // Start bit (Master mode)
#define I2C_SR1_ADDR         1    // Address sent/received
#define I2C_SR1_BTF          2    // Byte transfer finished
#define I2C_SR1_ADD10        3    // 10-bit header sent
#define I2C_SR1_STOPF        4    // Stop detection (Slave mode)
#define I2C_SR1_RXNE         6    // Data register not empty (receive)
#define I2C_SR1_TXE          7    // Data register empty (transmit)
#define I2C_SR1_BERR         8    // Bus error
#define I2C_SR1_ARLO         9    // Arbitration lost
#define I2C_SR1_AF           10   // Acknowledge failure
#define I2C_SR1_OVR          11   // Overrun/Underrun
#define I2C_SR1_PECERR       12   // PEC error in reception
#define I2C_SR1_TIMEOUT      14   // Timeout/Tlow error
#define I2C_SR1_SMBALERT     15   // SMBus alert

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL          0    // Master/slave
#define I2C_SR2_BUSY         1    // Bus busy
#define I2C_SR2_TRA          2    // Transmitter/receiver
#define I2C_SR2_GENCALL      4    // General call address detected
#define I2C_SR2_SMBDEFAULT   5    // SMBus device default address
#define I2C_SR2_SMBHOST      6    // SMBus host header
#define I2C_SR2_DUALF        7    // Dual flag
#define I2C_SR2_PEC          8    // Packet error checking register [15:8]

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR          0    // Clock control register [11:0]
#define I2C_CCR_DUTY         14   // Fast mode duty cycle
#define I2C_CCR_FS           15   // I2C master mode selection (0: Standard, 1: Fast)
/*
 * MACRO TO RESET I2Cx PERIPHERALS
 *
 */

/*
 * MACRO TO RESET I2Cx PERIPHERALS
 */
#define I2C1_REG_RESET()     do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()     do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
/*
 * peripheral definitions for UART
 */
typedef struct
{
    volatile uint32_t SR;     /*!< Status register,        Address offset: 0x00 */
    volatile uint32_t DR;     /*!< Data register,          Address offset: 0x04 */
    volatile uint32_t BRR;    /*!< Baud rate register,     Address offset: 0x08 */
    volatile uint32_t CR1;    /*!< Control register 1,     Address offset: 0x0C */
    volatile uint32_t CR2;    /*!< Control register 2,     Address offset: 0x10 */
    volatile uint32_t CR3;    /*!< Control register 3,     Address offset: 0x14 */
    volatile uint32_t GTPR;   /*!< Guard time prescaler,   Address offset: 0x18 */
} USART_RegDef_t;

/* Base addresses for USART/UART peripherals on STM32F103C8T6 */
#define USART1_BASEADDR   (APB2PERIPH_BASE + 0x3800)
#define USART2_BASEADDR   (APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR   (APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR    (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR    (APB1PERIPH_BASE + 0x5000)

/* Peripheral definitions */
#define USART1   ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2   ((USART_RegDef_t*)USART2_BASEADDR)
#define USART3   ((USART_RegDef_t*)USART3_BASEADDR)
#define UART4    ((USART_RegDef_t*)UART4_BASEADDR)
#define UART5    ((USART_RegDef_t*)UART5_BASEADDR)
/*
 * Bit positions for USART_CR1 register
 */
#define USART_CR1_SBK      0   // Send Break
#define USART_CR1_RWU      1   // Receiver Wakeup
#define USART_CR1_RE       2   // Receiver Enable
#define USART_CR1_TE       3   // Transmitter Enable
#define USART_CR1_IDLEIE   4   // IDLE Interrupt Enable
#define USART_CR1_RXNEIE   5   // RXNE Interrupt Enable
#define USART_CR1_TCIE     6   // Transmission Complete Interrupt Enable
#define USART_CR1_TXEIE    7   // TXE Interrupt Enable
#define USART_CR1_PEIE     8   // Parity Error Interrupt Enable
#define USART_CR1_PS       9   // Parity Selection
#define USART_CR1_PCE      10  // Parity Control Enable
#define USART_CR1_WAKE     11  // Wakeup method
#define USART_CR1_M        12  // Word length
#define USART_CR1_UE       13  // USART Enable
#define USART_CR1_OVER8    15  // Oversampling mode (0=16, 1=8)


/*
 * BIT POSITION DEFINITIONS OF CR2 USART PERIPHERAL
 */
#define USART_CR2_ADD        0   // Address of the USART node (bit0..3)
#define USART_CR2_LBDL       5   // LIN break detection length
#define USART_CR2_LBDIE      6   // LIN break detection interrupt enable
#define USART_CR2_LBCL       8   // Last bit clock pulse
#define USART_CR2_CPHA       9   // Clock phase
#define USART_CR2_CPOL       10  // Clock polarity
#define USART_CR2_CLKEN      11  // Clock enable
#define USART_CR2_STOP       12  // STOP bits (bit12..13)
#define USART_CR2_LINEN      14  // LIN mode enable

/*
 * BIT POSITION DEFINITIONS OF CR3 USART PERIPHERAL
 */
#define USART_CR3_EIE        0   // Error interrupt enable
#define USART_CR3_IREN       1   // IrDA mode enable
#define USART_CR3_IRLP       2   // IrDA low-power
#define USART_CR3_HDSEL      3   // Half-duplex selection
#define USART_CR3_NACK       4   // Smartcard NACK enable
#define USART_CR3_SCEN       5   // Smartcard mode enable
#define USART_CR3_DMAR       6   // DMA enable receiver
#define USART_CR3_DMAT       7   // DMA enable transmitter
#define USART_CR3_RTSE       8   // RTS enable
#define USART_CR3_CTSE       9   // CTS enable
#define USART_CR3_CTSIE      10  // CTS interrupt enable

/*
 * BIT POSITION DEFINITIONS OF SR USART PERIPHERAL
 */
#define USART_SR_PE          0   // Parity error
#define USART_SR_FE          1   // Framing error
#define USART_SR_NE          2   // Noise error flag
#define USART_SR_ORE         3   // Overrun error
#define USART_SR_IDLE        4   // IDLE line detected
#define USART_SR_RXNE        5   // Read data register not empty
#define USART_SR_TC          6   // Transmission complete
#define USART_SR_TXE         7   // Transmit data register empty
#define USART_SR_LBD         8   // LIN break detection flag
#define USART_SR_CTS         9   // CTS flag
/*
 * Macros for resetting USART/UART peripherals
 */
#define USART1_REG_RESET()   do{ (RCC->APB2RSTR |= (1 << 14)); (RCC->APB2RSTR &= ~(1 << 14)); }while(0)
#define USART2_REG_RESET()   do{ (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); }while(0)
#define USART3_REG_RESET()   do{ (RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18)); }while(0)
#define UART4_REG_RESET()    do{ (RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19)); }while(0)
#define UART5_REG_RESET()    do{ (RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20)); }while(0)

#include "STM32F103C8T6_gpio_driver.h"
#include "STM32F103C8T6_SPI_driver.h"
#include <STM32F103C8T6_I2C_driver.h>
#include "STM32F103C8T6_UART_driver.h"
#endif /* INC_STM32F103C8T6_H_ */
