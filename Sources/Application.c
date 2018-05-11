#include "Application.h"
#include "nRF24L01.h"
#include <stdbool.h>
#include "meraky.h"
#include <stdlib.h>
#include "RTE_Device.h"
#include <stdio.h>
#include "drv_lcd.h"


#define IS_SENDER    0  /* 1 je vys�l��, 0 je p�ij�ma� */
#define PAYLOAD_SIZE 8  /* po�et payload bajt�, 0 to 32 bytes */
#define CHANNEL_NO   2  /* komunika�n� kan�l */


#define TX_POWERUP()   WriteRegister(RF_CONFIG, EN_CRC|CRCO|PWR_UP|PRIM_TX) /* nastaven� 2 bajtov�ho CRC, zapnut� a nastaven� jako PTX- prim�rn� vys�la� */
#define RX_POWERUP()   WriteRegister(RF_CONFIG, EN_CRC|CRCO|PWR_UP|PRIM_RX) /* nastaven� 2 bajtov�ho CRC, zapnut� a nastaven� jako PRX - prim�rn� p�ij�ma� */


static const uint8_t TADDR[5] = {0x11, 0x22, 0x33, 0x44, 0x55}; /* adresa za��zen� */
static uint8_t payload[PAYLOAD_SIZE]; /* buffer pro payload */


void APP_Run(void) {

	//toto je k lcd posilani teploty
	uint32_t temperature, humidity;
		ARM_I2C_STATUS status;
		char buffer[16];


	    // Inicializace a konfigurace ovladace I2C
		Driver_I2C1.Initialize(i2c0_event);
		Driver_I2C1.PowerControl(ARM_POWER_FULL);
		Driver_I2C1.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);

		// Inicializace ovladace displeje
		//LCD_initialize();
	//konec



	Systick_Init();
	SPI_Init();
	RGB_Init();
#if IS_SENDER
  int i;
#endif
  int cntr;
  uint8_t statusRF;
  WriteRegister(RF_CONFIG,PWR_UP&0xFD);
  Write(FLUSH_TX);
  Write(FLUSH_RX);

  delay_us(10000); /* �as k zapnut� za��zen� */

  /* Je pou�it RF_SETUP registr, kde nastav�me v�stupn� v�kon 0dBm a p�enos 250 kb/s*/
  WriteRegister(RF_RF_SETUP,SETUP_RF_PWR_0|SETUP_RF_DR_1000);
  WriteRegister(RX_PW_P0, PAYLOAD_SIZE); /* po�et payload bajt�, kter� bu� p�ij�m�me nebo odes�l�me */
  WriteRegister(RF_RF_CH, CHANNEL_NO); /* nastaven� kan�lu */

  /*  Nastaven� RADDR a TADDR jako vys�lac� adresy, RX_ADDR_P0 adresa mus� b�t stejn� jako TX_ADDR , ��m� se povol� ACK*/
  WriteRegisterData(RX_ADDR_P0, (uint8_t*)TADDR, sizeof(TADDR));
  WriteRegisterData(TX_ADDR, (uint8_t*)TADDR, sizeof(TADDR));


  WriteRegister(RF_EN_RXADDR, EN_RXADDR_ERX_P0); /* zajist�, aby se adresy rovnaly, povol� datovou trubici 0 */


  ResetStatus(STATUS_RX_DR|STATUS_TX_DS|STATUS_MAX_RT); /* smazan� p��znak� p�eru�en� */

#if IS_SENDER
  WriteRegister(RF_EN_AA, EN_AA_ENAA_P0); /* Povolen� automatick�ho potvrzen� ACK.*/
  WriteRegister(RF_SETUP_RETR, SETUP_RETR_ARD_750|SETUP_RETR_ARC_15); /* delay 750 mikrosekund mezi opakovan�m */
  TX_POWERUP();  /* Zapnut� vys�lac�ho m�du */
  CE_LOW();   /* CE-low */
#else
  RX_POWERUP();  /* Zapnut� p�ij�mac�ho m�du */
  CE_HIGH();   /* CE-high */
#endif

  cntr = 0;
  for(;;) {
#if IS_SENDER

	  LEDR_ON();
      delay_us(20000);
      //temperature = MeasureTemperature();
      //humidity = MeasureHumidity();

      //delay_us(100000);
      //for(i=0;i<PAYLOAD_SIZE;i++) {
        payload[0] =0; //temperature>>24;
        payload[1] = 0;//temperature>>16;
        payload[2] = 1; //temperature>>8;
        payload[3] = rand()%10;//temperature;

        payload[4] = 0;//humidity>>24;
        payload[5] = 0;//humidity>>16;
        payload[6] = 0;//humidity>>8;
        payload[7] = rand()%24;//humidity;







      //}
      //uint32_t vysledek_teplota = payload[0]<<24 | payload[1]<<16 | payload[2]<<8 | payload[3];
      //uint32_t vysledek_vlhkost = payload[4]<<24 | payload[5]<<16 | payload[6]<<8 | payload[7];
      /*LCD_clear();
      sprintf(buffer, "H: %d.%d %%", vysledek_vlhkost,humidity);
      LCD_puts(buffer);
      sprintf(buffer, "T: %d.%d C", vysledek_teplota/10, temperature/10);
      LCD_set_cursor(2,1);
      LCD_puts(buffer);*/
      Transmit_Payload(payload,sizeof(payload)); /* poslat data */
      LEDR_OFF();
      delay_us(5000);



      statusRF = RF_GetStatus();

      if (statusRF&(STATUS_MAX_RT)) {
        ResetStatus(STATUS_MAX_RT);
        Transmit_Payload(payload,sizeof(payload)); /* poslat data */
      }



    delay_us(1);
#else




      delay_us(20000);
      statusRF = RF_GetStatus();
      if (statusRF&STATUS_RX_DR) {
    	  Receive_Payload(payload,sizeof(payload));
        uint32_t vysledek_teplota = payload[0]<<24 | payload[1]<<16 | payload[2]<<8 | payload[3];
        uint32_t vysledek_vlhkost = payload[4]<<24 | payload[5]<<16 | payload[6]<<8 | payload[7];
             LCD_clear();
             sprintf(buffer, "H: %d %%", vysledek_vlhkost);
             LCD_puts(buffer);
             sprintf(buffer, "T: %d.%d C", vysledek_teplota/10, vysledek_teplota%10);
             LCD_set_cursor(2,1);
             LCD_puts(buffer);

             LEDR_OFF();
             delay_us(5000);

        ResetStatus(STATUS_RX_DR|STATUS_TX_DS|STATUS_MAX_RT); /* Reset v�ech p��znak� */
        LEDG_ON();
        delay_us(30000);
        LEDG_OFF();
        delay_us(20000);
      }

      else {

        LEDR_ON();
        delay_us(30000);
        LEDR_OFF();
        delay_us(20000);
        ResetStatus(STATUS_RX_DR|STATUS_TX_DS|STATUS_MAX_RT);


    }
    delay_us(1);
#endif
  }
}
