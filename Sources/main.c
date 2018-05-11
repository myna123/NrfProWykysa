/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "MKL25Z4.h"
#include "nRF24l01.h"
#include "meraky.h"
#include <stdbool.h>
#include <stdlib.h>
#include "RTE_Device.h"
#include <stdio.h>
#include "drv_lcd.h"



#define SENDER 0 /* 1 je vysílaè, 0 je pøijímaè*/
#define CHANNEL_NUMBER 2 /* komunikaèní kanál*/
#define PAYLOAD_SIZE 8 /* poèet bajtù, 0 až 32 bajtù */



static const uint8_t address[5] = {0x11, 0x22, 0x33, 0x44, 0x55 }; /*adresa zaøízení*/
static uint8_t payload[PAYLOAD_SIZE];

char buffer[16];




int main(void)
{

	uint32_t temperature, humidity;
	ARM_I2C_STATUS status;

	char buffer[16];

	Systick_Init();
	SPI_Init();
	RGB_Init();
	//I2C_Init();
	//LCD_initialize();


//toto RF init
	uint8_t statusRF;
	WriteRegister(RF_CONFIG,PWR_UP&0xFD);
	Write(FLUSH_TX);
	Write(FLUSH_RX);

	delay_us(10000);

	WriteRegister(RF_RF_SETUP,SETUP_RF_PWR_0|SETUP_RF_DR_1000);
	WriteRegister(RX_PW_P0, PAYLOAD_SIZE);
	WriteRegister(RF_RF_CH, CHANNEL_NUMBER);

	WriteRegisterData(RX_ADDR_P0, (uint8_t*)address, sizeof(address));
	WriteRegisterData(TX_ADDR, (uint8_t*)address, sizeof(address));
	WriteRegister(RF_EN_RXADDR, EN_RXADDR_ERX_P0);
// konec

#if SENDER
	WriteRegister(RF_EN_AA, EN_AA_ENAA_P0);
	WriteRegister(RF_SETUP_RETR, SETUP_RETR_ARD_750|SETUP_RETR_ARC_15);
	WriteRegister(RF_CONFIG, EN_CRC|CRCO|PWR_UP|PRIM_TX);
	CE_LOW();

	for(;;) {

		LEDR_ON();
		delay_us(20000);

		//temperature = MeasureTemperature();
		//humidity = MeasureHumidity();

		payload[0] = 0;//temperature>>24;
		payload[1] = 0; //temperature>>16;
		payload[2] = 1;//temperature>>8;
		payload[3] =rand()%10; //temperature;

		payload[4] =0; //humidity>>24;
		payload[5] =0;//humidity>>16;
		payload[6] =0; //humidity>>8;
		payload[7] =rand()%20; //humidity;

		Transmit_Payload(payload,sizeof(payload));

		LEDR_OFF();
		delay_us(5000);


		//vysledek_teplota = payload[0]<<24 | payload[1]<<16 | payload[2]<<8 | payload[3];
		//vysledek_vlhkost = payload[4]<<24 | payload[5]<<16 | payload[6]<<8 | payload[7];

		/*delay_us(100000);
		LCD_clear();
	    sprintf(buffer, "H: %d %%", vysledek_vlhkost);
		LCD_puts(buffer);
		sprintf(buffer, "T: %d.%d C", vysledek_teplota/10, vysledek_teplota%10);
		LCD_set_cursor(2,1);
		LCD_puts(buffer);*/


		statusRF = RF_GetStatus();

		if (statusRF&(STATUS_MAX_RT)) {
			ResetStatus(STATUS_MAX_RT);
		    Transmit_Payload(payload,sizeof(payload));
		}

	}
	delay_us(1);
#else
	WriteRegister(RF_CONFIG, EN_CRC|CRCO|PWR_UP|PRIM_RX);
	CE_HIGH();
	LCD_initialize();
	for(;;){
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
	}


#endif









	return 0;
}



////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////





