#include "Application.h"
#include "nRF24L01.h"
#include <stdbool.h>
#include "meraky.h"
#include <stdlib.h>
#include "RTE_Device.h"
#include <stdio.h>
#include "drv_lcd.h"


#define IS_SENDER    0  /* 1 je vysíláè, 0 je pøijímaè */
#define PAYLOAD_SIZE 8  /* poèet payload bajtù, 0 to 32 bytes */
#define CHANNEL_NO   2  /* komunikaèní kanál */


#define TX_POWERUP()   WriteRegister(RF_CONFIG, EN_CRC|CRCO|PWR_UP|PRIM_TX) /* nastavení 2 bajtového CRC, zapnutí a nastavení jako PTX- primární vysílaè */
#define RX_POWERUP()   WriteRegister(RF_CONFIG, EN_CRC|CRCO|PWR_UP|PRIM_RX) /* nastavení 2 bajtového CRC, zapnutí a nastavení jako PRX - primární pøijímaè */


static const uint8_t TADDR[5] = {0x11, 0x22, 0x33, 0x44, 0x55}; /* adresa zaøízení */
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

  delay_us(10000); /* èas k zapnutí zaøízení */

  /* Je použit RF_SETUP registr, kde nastavíme výstupní výkon 0dBm a pøenos 250 kb/s*/
  WriteRegister(RF_RF_SETUP,SETUP_RF_PWR_0|SETUP_RF_DR_1000);
  WriteRegister(RX_PW_P0, PAYLOAD_SIZE); /* poèet payload bajtù, které buï pøijímáme nebo odesíláme */
  WriteRegister(RF_RF_CH, CHANNEL_NO); /* nastavení kanálu */

  /*  Nastavení RADDR a TADDR jako vysílací adresy, RX_ADDR_P0 adresa musí být stejná jako TX_ADDR , èímž se povolí ACK*/
  WriteRegisterData(RX_ADDR_P0, (uint8_t*)TADDR, sizeof(TADDR));
  WriteRegisterData(TX_ADDR, (uint8_t*)TADDR, sizeof(TADDR));


  WriteRegister(RF_EN_RXADDR, EN_RXADDR_ERX_P0); /* zajistí, aby se adresy rovnaly, povolí datovou trubici 0 */


  ResetStatus(STATUS_RX_DR|STATUS_TX_DS|STATUS_MAX_RT); /* smazaní pøíznakù pøerušení */

#if IS_SENDER
  WriteRegister(RF_EN_AA, EN_AA_ENAA_P0); /* Povolení automatického potvrzení ACK.*/
  WriteRegister(RF_SETUP_RETR, SETUP_RETR_ARD_750|SETUP_RETR_ARC_15); /* delay 750 mikrosekund mezi opakovaním */
  TX_POWERUP();  /* Zapnutí vysílacího módu */
  CE_LOW();   /* CE-low */
#else
  RX_POWERUP();  /* Zapnutí pøijímacího módu */
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

        ResetStatus(STATUS_RX_DR|STATUS_TX_DS|STATUS_MAX_RT); /* Reset všech pøíznakù */
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
