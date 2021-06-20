/*
 * stm32f4xx.h
 *
 *  Created on: Jun 18, 2021
 *      Author: Murali Krishna
 */

#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_

/*base address of the memory components*/
#define FLASH_BASEADDR 0x08000000U
#define SRAM_BASEADDR  0x20000000U
#define SRAM1_BASEADDR 0x20000000U
#define SRAM2_BASEADDR 0x2001C000U

/*base address of the Bus systems*/
#define AHB1_BASEADDR 0x40020000U
#define APB1_BASEADDR 0x40000000U
#define APB2_BASEADDR 0x40010000U


/*base address of the Peripherals hanging on to AHB1*/
#define GPIOA_BASEADDR (AHB1_BASEADDR + 0X0000)
#define GPIOB_BASEADDR (AHB1_BASEADDR + 0X0400)
#define GPIOC_BASEADDR (AHB1_BASEADDR + 0X0800)
#define GPIOD_BASEADDR (AHB1_BASEADDR + 0X0C00)
#define GPIOE_BASEADDR (AHB1_BASEADDR + 0X1000)
#define GPIOF_BASEADDR (AHB1_BASEADDR + 0X1400)
#define GPIOG_BASEADDR (AHB1_BASEADDR + 0X1800)
#define GPIOH_BASEADDR (AHB1_BASEADDR + 0X1C00)
#define GPIOI_BASEADDR (AHB1_BASEADDR + 0X2000)


/*base address of the Peripherals hanging on to APB1*/
#define I2C1_BASEADDR   (APB1_BASEADDR + 0X5400)
#define I2C2_BASEADDR   (APB1_BASEADDR + 0X5800)
#define I2C3_BASEADDR   (APB1_BASEADDR + 0X5C00)
#define SPI2_BASEADDR   (APB1_BASEADDR + 0X3800)
#define SPI3_BASEADDR   (APB1_BASEADDR + 0X3C00)
#define USART2_BASEADDR (APB1_BASEADDR + 0X4400)
#define USART3_BASEADDR (APB1_BASEADDR + 0X4800)
#define USART4_BASEADDR (APB1_BASEADDR + 0X4C00)
#define USART5_BASEADDR (APB1_BASEADDR + 0X5000)


/*base address of the Peripherals hanging on to APB2*/
#define SPI1_BASEADDR   (APB2_BASEADDR + 0X3000)
#define USART1_BASEADDR (APB2_BASEADDR + 0X1000)
#define USART6_BASEADDR (APB2_BASEADDR + 0X1400)
#define EXTI_BASEADDR   (APB2_BASEADDR + 0X3C00)
#define SYSCFG_BASEADDR (APB2_BASEADDR + 0X3800)

/*typedef for GPIO*/
typedef struct
{
	uint32_t MODER;
	uint32_t OTYPER;
	uint32_t OSPEEDR;
	uint32_t PUPDR;
	uint32_t IDR;
 	uint32_t ODR;
	uint32_t BSRR;
 	uint32_t LCKR;
	uint32_t AFR[2];
}GPIO_RegDef_t;

typedef struct
{
	uint32_t CR;
	uint32_t PLLCFGR;
	uint32_t CFGR;
	uint32_t CIR;
	uint32_t AHB1RSTR;
 	uint32_t AHB2RSTR;
	uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	uint32_t APB1RSTR;
	uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	uint32_t AHB1ENR;
	uint32_t AHB2ENR;
	uint32_t AHB3ENR;
	uint32_t RESERVED2;
	uint32_t APB1ENR;
	uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	uint32_t AHB1LPENR;
	uint32_t AHB2LPENR;
	uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	uint32_t APB1LPENR;
	uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	uint32_t BDCR;
	uint32_t CSR;
	uint32_t RESERVED[2];
	uint32_t SSCGR;
	uint32_t PLLI2SCFGR;
}RCC_Regdef_t;

/*
 * peripheral definitions
 */

#define GPIOA  ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB  (GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC  ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD  ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE  ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF  ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG  ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH  ((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI  ((GPIO_RegDef_t*) GPIOI_BASEADDR)
#define RCC    ((RCC_RegDef_t*) RCC_BASEADDR)


#define GPIOA_PCLK_EN()   (RCC->AHB1ENR |= (1<<0)) // set
#define GPIOB_PCLK_EN()   (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()   (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()   (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()   (RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()   (RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()   (RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()   (RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()   (RCC->AHB1ENR |= (1<<8))

#define GPIOA_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<0)) // reset
#define GPIOB_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<8))

typedef struct
{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t OAR1;
	uint32_t OAR2;
	uint32_t DR;
	uint32_t SR1;
	uint32_t SR2;
	uint32_t CCR;
	uint32_t TRISE;
	uint32_t FLTR;
}I2C_Regdef_t;

#define  I2C1  ((I2C_Regdef_t*) APB1_I2C1_BASEADDR)
#define I2C2  ((I2C_Regdef_t*) APB1_I2C2_BASEADDR)
#define  I2C3  ((I2C_Regdef_t*) APB1_I2C3_BASEADDR)


typedef struct
{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t SR;
    uint32_t DR;
    uint32_t CRCPR;
    uint32_t RXCRCR;
    uint32_t TXCRCR;
    uint32_t I2SCFGR;
    uint32_t I2SPR;
}SPI_Regdef_t;

#define SPI2  ((SPI_Regdef_t*) APB1_SPI2_BASEADDR)
#define SPI3  ((SPI_Regdef_t*) APB1_SPI3_BASEADDR)
#define SPI1  ((SPI_Regdef_t*) APB2_SPI1_BASEADDR)


#endif /* INC_STM32F4XX_H_ */
