/*
 * nRF24l01.h
 *
 *  Created on: 6. 4. 2018
 *      Author: ako_2
 */

#ifndef SOURCES_NRF24L01_H_
#define SOURCES_NRF24L01_H_


#include <stdint.h>

/* Memory Map - register address defines */
#define RF_CONFIG      0x00 /* CONFIG register */
#define RF_EN_AA       0x01 /* EN_AA register */
#define RF_EN_RXADDR   0x02 /* EN_RXADDR register */
#define RF_SETUP_AW    0x03 /* SETUP_AW register */
#define RF_SETUP_RETR  0x04
#define RF_RF_CH       0x05
#define RF_RF_SETUP    0x06 /* SETUP register */
#define RF_STATUS      0x07
#define RF_OBSERVE_TX  0x08
#define RF_RPD     0x09    /* Mnemonic for nRF24L01+ */
//#define CD          0x09   /* Mnemonic from nRF24L01, new is RPD */
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17

/* Bit Mnemonics */
/* CONFIG Register Bits */
#define MASK_RX_DR  (1<<6)  /* Mask interrupt caused by RX_DR: 1: interrupt masked. 0: interrupt enabled */
#define MASK_TX_DS  (1<<5)  /* Mask interrupt caused by TX_DS: 1: interrupt masked. 0: interrupt enabled */
#define MASK_MAX_RT (1<<4)  /* Mask interrupt caused by MAX_RT. 1: interrupt not reflected on IRQ pin. 0: reflect MAX_RT as active low interrupt on IRQ pin */
#define EN_CRC      (1<<3)  /* Enable CRC. Forced high if on of the bits in EN_AA is high */
#define CRCO        (1<<2)  /* CRC encoding scheme, 0: 1 byte, 1: 2 bytes */
#define PWR_UP      (1<<1)  /* 1: Power up, 0: Power down */
#define PRIM_RX     (1<<0)  /* 1: PRX, 0: PTX */
#define PRIM_TX     (0)     /* 0: PTX */

#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR_HIGH  3
#define RF_DR_LOW   5
#define RF_PWR      1
#define LNA_HCURR   0
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0

/* Command Name Mnemonics (Instructions) */
#define R_REGISTER     0x00
#define W_REGISTER     0x20
#define REGISTER_MASK  0x1F
#define R_RX_PAYLOAD   0x61
#define W_TX_PAYLOAD   0xA0
#define FLUSH_TX       0xE1
#define FLUSH_RX       0xE2
#define REUSE_TX_PL    0xE3
#define NOP            0xFF

#define CONFIG_DEFAULT_VAL         0x08
#define EN_AA_DEFAULT_VAL          0x3F
#define EN_RXADDR_DEFAULT_VAL      0x03
#define SETUP_AW_DEFAULT_VAL       0x03
#define SETUP_RETR_DEFAULT_VAL     0x03
#define RF_CH_DEFAULT_VAL          0x02
#define RF_SETUP_DEFAULT_VAL       0x0F
#define STATUS_DEFAULT_VAL         0x0E
#define OBSERVE_TX_DEFAULT_VAL     0x00
#define CD_DEFAULT_VAL             0x00
#define RX_ADDR_P0_B0_DEFAULT_VAL  0xE7
#define RX_ADDR_P0_B1_DEFAULT_VAL  0xE7
#define RX_ADDR_P0_B2_DEFAULT_VAL  0xE7
#define RX_ADDR_P0_B3_DEFAULT_VAL  0xE7
#define RX_ADDR_P0_B4_DEFAULT_VAL  0xE7
#define RX_ADDR_P1_B0_DEFAULT_VAL  0xC2
#define RX_ADDR_P1_B1_DEFAULT_VAL  0xC2
#define RX_ADDR_P1_B2_DEFAULT_VAL  0xC2
#define RX_ADDR_P1_B3_DEFAULT_VAL  0xC2
#define RX_ADDR_P1_B4_DEFAULT_VAL  0xC2
#define RX_ADDR_P2_DEFAULT_VAL     0xC3
#define RX_ADDR_P3_DEFAULT_VAL     0xC4
#define RX_ADDR_P4_DEFAULT_VAL     0xC5
#define RX_ADDR_P5_DEFAULT_VAL     0xC6
#define TX_ADDR_B0_DEFAULT_VAL     0xE7
#define TX_ADDR_B1_DEFAULT_VAL     0xE7
#define TX_ADDR_B2_DEFAULT_VAL     0xE7
#define TX_ADDR_B3_DEFAULT_VAL     0xE7
#define TX_ADDR_B4_DEFAULT_VAL     0xE7
#define RX_PW_P0_DEFAULT_VAL       0x00
#define RX_PW_P1_DEFAULT_VAL       0x00
#define RX_PW_P2_DEFAULT_VAL       0x00
#define RX_PW_P3_DEFAULT_VAL       0x00
#define RX_PW_P4_DEFAULT_VAL       0x00
#define RX_PW_P5_DEFAULT_VAL       0x00
#define FIFO_STATUS_DEFAULT_VAL    0x11

/* CONFIG register bitwise definitions */
#define CONFIG_RESERVED     0x80
#define CONFIG_MASK_RX_DR   0x40
#define CONFIG_MASK_TX_DS   0x20
#define CONFIG_MASK_MAX_RT  0x10
#define CONFIG_EN_CRC       0x08
#define CONFIG_CRCO         0x04
#define CONFIG_PWR_UP       0x02
#define CONFIG_PRIM_RX      0x01

/* EN_AA register bitwise definitions */
#define EN_AA_RESERVED      0xC0
#define EN_AA_ENAA_ALL      0x3F
#define EN_AA_ENAA_P5       0x20
#define EN_AA_ENAA_P4       0x10
#define EN_AA_ENAA_P3       0x08
#define EN_AA_ENAA_P2       0x04
#define EN_AA_ENAA_P1       0x02
#define EN_AA_ENAA_P0       0x01
#define EN_AA_ENAA_NONE     0x00

/* EN_RXADDR register bitwise definitions */
#define EN_RXADDR_RESERVED  0xC0
#define EN_RXADDR_ERX_ALL   0x3F
#define EN_RXADDR_ERX_P5    0x20
#define EN_RXADDR_ERX_P4    0x10
#define EN_RXADDR_ERX_P3    0x08
#define EN_RXADDR_ERX_P2    0x04
#define EN_RXADDR_ERX_P1    0x02
#define EN_RXADDR_ERX_P0    0x01
#define EN_RXADDR_ERX_NONE  0x00

/* SETUP_AW register bitwise definitions */
#define SETUP_AW_RESERVED 0xFC
#define SETUP_AW          0x03
#define SETUP_AW_5BYTES   0x03
#define SETUP_AW_4BYTES   0x02
#define SETUP_AW_3BYTES   0x01
#define SETUP_AW_ILLEGAL  0x00

/* SETUP_RETR register bitwise definitions */
#define SETUP_RETR_ARD        0xF0
#define SETUP_RETR_ARD_4000   0xF0 /* 4400 us retry delay */
#define SETUP_RETR_ARD_3750   0xE0 /* 3750 us retry delay */
#define SETUP_RETR_ARD_3500   0xD0 /* 3500 us retry delay */
#define SETUP_RETR_ARD_3250   0xC0 /* 3250 us retry delay */
#define SETUP_RETR_ARD_3000   0xB0 /* 3000 us retry delay */
#define SETUP_RETR_ARD_2750   0xA0 /* 2750 us retry delay */
#define SETUP_RETR_ARD_2500   0x90 /* 2500 us retry delay */
#define SETUP_RETR_ARD_2250   0x80 /* 2250 us retry delay */
#define SETUP_RETR_ARD_2000   0x70 /* 2000 us retry delay */
#define SETUP_RETR_ARD_1750   0x60 /* 1750 us retry delay */
#define SETUP_RETR_ARD_1500   0x50 /* 1500 us retry delay */
#define SETUP_RETR_ARD_1250   0x40 /* 1250 us retry delay */
#define SETUP_RETR_ARD_1000   0x30 /* 1000 us retry delay */
#define SETUP_RETR_ARD_750    0x20 /* 750 us retry delay */
#define SETUP_RETR_ARD_500    0x10 /* 500 us retry delay */
#define SETUP_RETR_ARD_250    0x00 /* 250 us retry delay */
#define SETUP_RETR_ARC        0x0F
#define SETUP_RETR_ARC_15     0x0F /* 15 retry count */
#define SETUP_RETR_ARC_14     0x0E /* 14 retry count */
#define SETUP_RETR_ARC_13     0x0D /* 13 retry count */
#define SETUP_RETR_ARC_12     0x0C /* 12 retry count */
#define SETUP_RETR_ARC_11     0x0B /* 11 retry count */
#define SETUP_RETR_ARC_10     0x0A /* 10 retry count */
#define SETUP_RETR_ARC_9      0x09 /* 9 retry count */
#define SETUP_RETR_ARC_8      0x08 /* 8 retry count */
#define SETUP_RETR_ARC_7      0x07 /* 7 retry count */
#define SETUP_RETR_ARC_6      0x06 /* 6 retry count */
#define SETUP_RETR_ARC_5      0x05 /* 5 retry count */
#define SETUP_RETR_ARC_4      0x04 /* 4 retry count */
#define SETUP_RETR_ARC_3      0x03 /* 3 retry count */
#define SETUP_RETR_ARC_2      0x02 /* 2 retry count */
#define SETUP_RETR_ARC_1      0x01 /* 1 retry count */
#define SETUP_RETR_ARC_0      0x00 /* 0 retry count, retry disabled */

/* RF_CH register bitwise definitions */
#define RF_CH_RESERVED    0x80

/* RF_SETUP register bitwise definitions */
#define SETUP_RESERVED    0xE0
#define SETUP_PLL_LOCK    0x10
#define SETUP_RF_DR       0x08
#define SETUP_RF_DR_250   0x20
#define SETUP_RF_DR_1000  0x00
#define SETUP_RF_DR_2000  0x08
#define SETUP_RF_PWR      0x06
#define SETUP_RF_PWR_0    0x06
#define SETUP_RF_PWR_6    0x04
#define SETUP_RF_PWR_12   0x02
#define SETUP_RF_PWR_18   0x00
#define SETUP_LNA_HCURR   0x01

/* STATUS register bit definitions */
#define STATUS_RESERVED                    0x80   /* bit 1xxx xxxx: This bit is reserved */
#define STATUS_RX_DR                       0x40   /* bit x1xx xxxx: Data ready RX FIFO interrupt. Asserted when new data arrives RX FIFO */
#define STATUS_TX_DS                       0x20   /* bit xx1x xxxx: Data sent TX FIFO interrupt. Asserted when packet transmitted on TX. */
#define STATUS_MAX_RT                      0x10   /* bit xxx1 xxxx: maximum number of TX retransmit interrupts */
#define STATUS_RX_P_NO                     0x0E
#define STATUS_RX_P_NO_RX_FIFO_NOT_EMPTY   0x0E
#define STATUS_RX_P_NO_UNUSED              0x0C
#define STATUS_RX_P_NO_5                   0x0A
#define STATUS_RX_P_NO_4                   0x08
#define STATUS_RX_P_NO_3                   0x06
#define STATUS_RX_P_NO_2                   0x04
#define STATUS_RX_P_NO_1                   0x02
#define STATUS_RX_P_NO_0                   0x00   /* bit xxxx 111x: pipe number for payload */
#define STATUS_TX_FULL                     0x01   /* bit xxxx xxx1: if bit set, then TX FIFO is full */

/* OBSERVE_TX register bitwise definitions */
#define OBSERVE_TX_PLOS_CNT   0xF0
#define OBSERVE_TX_ARC_CNT    0x0F

/* RPD register bitwise definitions for nRF24L01+ */
#define RPD_RESERVED    0xFE
#define RPD_RPD         0x01

/* RX_PW_P0 register bitwise definitions */
#define RX_PW_P0_RESERVED 0xC0

/* RX_PW_P0 register bitwise definitions */
#define RX_PW_P0_RESERVED 0xC0

/* RX_PW_P1 register bitwise definitions */
#define RX_PW_P1_RESERVED 0xC0

/* RX_PW_P2 register bitwise definitions */
#define RX_PW_P2_RESERVED 0xC0

/* RX_PW_P3 register bitwise definitions */
#define RX_PW_P3_RESERVED 0xC0

/* RX_PW_P4 register bitwise definitions */
#define RX_PW_P4_RESERVED 0xC0

/* RX_PW_P5 register bitwise definitions */
#define RX_PW_P5_RESERVED 0xC0

/* FIFO_STATUS register bitwise definitions */
#define FIFO_STATUS_RESERVED  0x8C
#define FIFO_STATUS_TX_REUSE  0x40
#define FIFO_STATUS_TX_FULL   0x20
#define FIFO_STATUS_TX_EMPTY  0x10
#define FIFO_STATUS_RX_FULL   0x02
#define FIFO_STATUS_RX_EMPTY  0x01

void WriteRegister(uint8_t reg, uint8_t val);

uint8_t ReadRegister(uint8_t reg);

void ReadRegisterData(uint8_t reg, uint8_t *buf, uint8_t bufSize);

void WriteRegisterData(uint8_t reg, uint8_t *buf, uint8_t bufSize);

uint8_t RF_GetStatus(void);

void Transmit_Payload(uint8_t *payload, uint8_t payloadSize);

void Receive_Payload(uint8_t *payload, uint8_t payloadSize);

void ResetStatus(uint8_t flags);

void SPI_Init(void);

void en_SPI(void);

void CSN_HIGH(void);

void CE_LOW(void);

void CSN_LOW(void);

void CE_HIGH(void);

void Systick_Init(void);

void delay_us(uint32_t micros);

void LEDG_ON(void);

void LEDG_OFF(void);

void LEDR_ON(void);

void LEDR_OFF(void);

void RGB_Init(void);

void Write(uint8_t val);



#endif /* SOURCES_NRF24L01_H_ */
