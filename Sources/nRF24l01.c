/*
 * nRF24l01.c

 *
 *  Created on: 6. 4. 2018
 *      Author: ako_2
 */



#include "MKL25Z4.h"
#include "nRF24l01.h"
#include "Application.h"


#define		CSN		(0)
#define		CE		(16)
#define		MISO	(7)
#define		MOSI	(6)
#define		SCK		(5)
#define		IRQ     (3)
#define		LEDG     (19)
#define		LEDR     (18)


volatile uint32_t g_delaycnt;


void Systick_Init(void) {
	//Nastaveni preruseni od casovace SysTick na 10 us periodu
	// vyuzito v delay_ms
	SysTick_Config(SystemCoreClock / 100000u );
}
void en_SPI(void) {


	SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK; // povol� hodinov� sign�l pro SPI0

}

void SPI_Init(void) {

	en_SPI();

	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK|SIM_SCGC5_PORTD_MASK;


	PORTC->PCR[CSN] = PORT_PCR_MUX(1); // PTC0 jako CSN
	PORTC->PCR[CE] = PORT_PCR_MUX(1); // PTC16 jako CE
	PORTC->PCR[SCK] = PORT_PCR_MUX(2); // PTC5 jako SPI0_SCK
	PORTC->PCR[MOSI] = PORT_PCR_MUX(2); // PTC6 jako SPI0_MOSI
	PORTC->PCR[MISO] = PORT_PCR_MUX(2); // PTC7 jako SPI0_MISO



	PTC->PDDR |= (1 << CSN); // nastaven� pinu CSN jako v�stupu
	PTC->PDDR |= (1 << CE); // nastaven� pinu CE jako v�stupu

	SPI0_C1 &= ~ SPI_C1_CPHA_MASK; // nulovan� bitu CPHA
	SPI0_C1 = SPI_C1_MSTR_MASK ; //CPOL=0,CPHA=0 use polling
	SPI0_BR = (SPI_BR_SPPR(0x02) | SPI_BR_SPR(0x02)); /* Nastaven� baud rate na 1Mbps*/
	SPI0_C1 |= SPI_C1_SPE_MASK; // povolen� SPI

	CSN_HIGH(); //Inicializace CSN
	CE_LOW();   // Inicializace CE
}

void CSN_LOW(void) {

	PTC->PCOR |= (1 << CSN);

}

void CSN_HIGH(void) {

	PTC->PSOR |= (1 << CSN);

}

void CE_HIGH(void) {

	PTC->PSOR |= (1 << CE);

}

void CE_LOW(void) {

	PTC->PCOR |= (1 << CE);

}

void delay_us(uint32_t micros)
{
	g_delaycnt = micros;

    /* Cekame az SysTick dekrementuje pocitadlo na 0 */
    while (g_delaycnt != 0u)
       ;
}

/* Obsluha pro SysTick interrupt.
   Jmeno obsluzne funkce je preddefinovano.
   Staci v nasem programu vytvorit funkci tohoto jmena a bude
   volana ona misto vychozi prazdne funckce.
*/

void SysTick_Handler(void) {
    /* Dekrementujeme pocitadlo pouzivane funkci delay_ms */
    if (g_delaycnt != 0u)
    {
        g_delaycnt--;
    }
}

static uint8_t SPI_send(uint8_t val) {


	uint8_t ch;
	while(!(SPI0_S & SPI_S_SPTEF_MASK ) ); // dokud nebude prazdn� transmit buffer
	SPI0_D = val;

	while(!(SPI0_S & SPI_S_SPRF_MASK ) ); // dokud nebudou data dostupn� v recieve buffer
	ch = SPI0_D;

	return ch;


}


/*!
 * \brief P�istupuje k dan�mu registru a nastavuje pot�ebn� funkce
 * \param reg Registr, kter� chceme vyu��t
 * \param val Hodnota registru, kterou chceme nastavit
 */
void WriteRegister(uint8_t reg, uint8_t val) {
  CSN_LOW();
  SPI_send(W_REGISTER|reg); /* zapisuje dan� p��kaz v registru */
  SPI_send(val); /* zap�e hodnotu */
  CSN_HIGH();
  delay_us(1);
}

/*!
 * \brief Z�pis v�ce bajt� do sb�rnice.
 * \param reg Adresa registru
 * \param buf Buffer toho co chceme zapsat
 * \param bufSize Velikost bufferu v bajtech
 */
void WriteRegisterData(uint8_t reg, uint8_t *buf, uint8_t bufSize) {
  CSN_LOW();
  SPI_send(W_REGISTER|reg);
  for(int i=0;i<bufSize;i++) {
	  SPI_send(buf[i]);
   }
  CSN_HIGH();
  delay_us(1);
}

/*!
 * \brief Na�ten� v�ce bajt� ze sb�rnice
 * \param reg Adresa registru
 * \param buf Buffer kam zapsat data
 * \param bufSize Velikost bufferu v bajtech
 */
void ReadRegisterData(uint8_t reg, uint8_t *buf, uint8_t bufSize) {
  CSN_LOW();
  SPI_send(R_REGISTER|reg);

  for(int i=0;i<bufSize;i++) {
	  buf[i] = SPI_send(buf[i]);
    }

  CSN_HIGH();
  delay_us(1);
}
/*!
 * \brief Z�p�e bajt do sb�rnice, ale nena�te ho zp�t
 * \param val Bajt k z�pisu.
 */
void Write(uint8_t val) {
  CSN_LOW();
  SPI_send(val);
  CSN_HIGH();
  delay_us(1);
}
/*!
 * \brief Z�pis bajtu a jeho na�ten�
 * \param val Bajt k zapsan� do posuvn�ho SPI registru
 * \return Bajt na�ten� z posuvn�ho registru
 */
uint8_t WriteRead(uint8_t val) {
  CSN_LOW();
  val = SPI_send(val);
  CSN_HIGH();
  delay_us(1);
  return val;
}

/*!
 * \brief Na�te a vrat� STATUS
 * \return Status
 */
uint8_t RF_GetStatus(void) {
  return WriteRead(NOP);
}

/*!
 * \brief Resetuje p��znaky
 * \param flags P��znak, �i v�ce p��znak� RF24_STATUS_RX_DR, RF24_STATUS_TX_DS, RF24_STATUS_MAX_RT
 */
void ResetStatus(uint8_t flags) {
  delay_us(1);
  CSN_LOW();
  delay_us(1);
  WriteRegister(RF_STATUS, flags);
  delay_us(1);
  CSN_HIGH();
  delay_us(1);
}

/*!
 * \brief  Po�le payload do Tx FIFO fronty
 * \param payload Buffer s payload k posl�n�
 * \param payloadSize Velikost payload v bufferu
 */
void Transmit_Payload(uint8_t *payload, uint8_t payloadSize) {
  Write(FLUSH_TX); /* vyhod� star� data */
  WriteRegisterData(W_TX_PAYLOAD, payload, payloadSize); /* zap�e payload */
  CE_HIGH(); /* za��tek p�enosu */
  delay_us(2);
  CE_LOW();  /* konec p�enosu */
}

/*!
 * \brief P�ijme Rx payload z FIFO a ulo�� do bufferu.
 * \param payload Ukazatel na payload v bufferu
 * \param payloadSize Velikost payload v bufferu
 */
void Receive_Payload(uint8_t *payload, uint8_t payloadSize) {
  CE_LOW();
  ReadRegisterData(R_RX_PAYLOAD, payload, payloadSize);
  CE_HIGH();
}



/*void PORTD_ISR(void) {

	if (PORTD_ISFR & (1 << IRQ)) //pokud je p�eru�en� identifikov�no
	{
		APP_OnInterrupt();
	}

	PORTD_ISFR=0xFFFFFFFF; // ma�eme p��znaky ISR na portu D

}*/

void RGB_Init(void){

	SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK );


	PORTB->PCR[LEDR] = PORT_PCR_MUX(1);  //Red led pin
	PORTB->PCR[LEDG] = PORT_PCR_MUX(1);//Green led pin


	PTB->PDDR |= (1 << LEDR );
	PTB->PDDR |= (1 << LEDG );

	LEDG_OFF();
	LEDR_OFF();



}

void LEDG_ON(void) {

	PTB->PCOR |= (1 << LEDG);

}

void LEDG_OFF(void) {

	PTB->PSOR |= (1 << LEDG);

}


void LEDR_ON(void) {

	PTB->PCOR |= (1 << LEDR);

}

void LEDR_OFF(void) {

	PTB->PSOR |= (1 << LEDR);

}






