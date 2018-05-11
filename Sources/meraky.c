/*
 * meraky.c
 *
 *  Created on: 8. 5. 2018
 *      Author: ako_2
 */

#include "MKL25Z4.h"
#include "RTE_Device.h"
#include <stdio.h>
#include "drv_lcd.h"		// ovladac displeje
//#include "drv_systick.h"	// pro delay_ms

#include "MKL25Z4.h"
#include "Application.h"
#include "nRF24l01.h"
#include "meraky.h"


// Adresy obvodu na I2C sbernici
#define I2C_ADR_RTC (0b1010000)
#define I2C_ADR_TEMP_SENSOR (0b1001000)
#define I2C_ADR_HMDT_SENSOR (0b0100111)

// Registry teplotniho snimace LM75
#define LM75_REG_TEMP  (0)
uint32_t MeasureHumidity(void)
{
	uint8_t hih_measure_status;
	uint8_t dataSend[2] = { 0 };	// data to send
	uint8_t sendSize = 0;		// number of bytes to send
	uint8_t dataReceive[4];
	uint32_t humidity;
	ARM_I2C_STATUS status;

		// Zahajime mereni poslanim slave adresy a R/W bitu 0 (prikaz write bez dat)
		Driver_I2C1.MasterTransmit(I2C_ADR_HMDT_SENSOR, dataSend, 0, false);
		status = Driver_I2C1.GetStatus();
		while (status.busy) {
			status = Driver_I2C1.GetStatus();
		}
		// Senzoru trva 37 ms nez dokonci prevod.
		// Muzeme take zjistovat status bity v prijatych datech, ale
		// to je komplikovanejsi.
		delay_us(4000);


		Driver_I2C1.MasterReceive(I2C_ADR_HMDT_SENSOR, dataReceive, 4, false);
		// Cekame na prijem dat
		status = Driver_I2C1.GetStatus();
		while (status.busy) {
			status = Driver_I2C1.GetStatus();
		}

		humidity = 256 * (dataReceive[0] & 0b00111111) + dataReceive[1];

		// Vypocet vlhkosti
		humidity = ((int) humidity * 100) / 16383;
		return humidity;

}

/*
 Cteni teploty z LM75A
 Vraci teplotu ve stupnich x 10, napr. 205 znamena 20.5 C

 Poznamky:
 Obvod ma "pointer" ktery urcuje, co chceme cist nebo zapisovat.
 adresa 0 = teplota (2 B)
 adresa 1 = config (1B)
 a dalsi nepodstatne adresy :)
 V config registru je defaultne hodnota 0 a ta vyhovuje pro mereni teploty.
 Presto se zda nutne do nej zapsat nejprve tuto 0.
*/
uint32_t MeasureTemperature(void)
{
	// Inicializacni prikaz pro teplotni senzor LM75
	const uint8_t cmd_lm75_init[] = {0b00000001, 0b00000000};		// zapis defaultnich hodnot do config registru

	uint8_t dataSend[2] = { 0 };	// data to send
	uint8_t sendSize = 0;		// number of bytes to send
	uint8_t dataReceive[2];
	uint32_t temperature;
	ARM_I2C_STATUS status;

	// Inicializace teplotniho snimace
	Driver_I2C1.MasterTransmit(I2C_ADR_TEMP_SENSOR, cmd_lm75_init, 2, false);
	// Cekame na odeslani dat
	status = Driver_I2C1.GetStatus();
	while ( status.busy ) {
		status = Driver_I2C1.GetStatus();
	}


#if 1
	// Varianta a) - cteni teploty s poslanim pointer hodnoty
	// tj. posilam prikaz "precti registr s teplotou":
	// pointer = 0 (teplota;
	// cti(pointer)

	dataSend[0] = LM75_REG_TEMP;		// hodnota = 0 = pointer
	sendSize = 1;
	Driver_I2C1.MasterTransmit(I2C_ADR_TEMP_SENSOR, dataSend, sendSize, true);
	// Cekame na odeslani dat
	status = Driver_I2C1.GetStatus();
	while (status.busy)
		status = Driver_I2C1.GetStatus();

	Driver_I2C1.MasterReceive(I2C_ADR_TEMP_SENSOR, dataReceive, 2, false);
	// Cekame na odeslani dat
	status = Driver_I2C1.GetStatus();
	while (status.busy)
		status = Driver_I2C1.GetStatus();

	temperature = (256 * dataReceive[0] + dataReceive[1]) >> 5;
	temperature = (temperature * 1270) / 0x3f8;		// prepocet na st.C
	return temperature;
#endif


#if 0
	//
	// Varianta b) - cteni teploty with preset pointer - funguje taky
	// tj. posilam prikaz precti posledni registr (a to je registr teploty, pokud
	// jsem na nej predtim nastavil pointer:
	// Stav:
	// nefunguje; arbitration lost po prvnim cteni...
	dataSend[0] = LM75_REG_TEMP;		// hodnota = 0 = pointer
	sendSize = 1;
	Driver_I2C1.MasterTransmit(I2C_ADR_TEMP_SENSOR, dataSend, sendSize, true);
	status = Driver_I2C1.GetStatus();
	while (status.busy)
		status = Driver_I2C1.GetStatus();
	Driver_I2C1.MasterReceive(I2C_ADR_TEMP_SENSOR, dataReceive, 2, false);
	status = Driver_I2C1.GetStatus();
	while (status.busy)
	status = Driver_I2C1.GetStatus();

	temperature = (256 * dataReceive[0] + dataReceive[1]) >> 5;
	temperature = (temperature * 1270) / 0x3f8;// prepocet na st.C
	return temperature;

#endif



}

// funkce volana ovladacem I2C pokud povolime signalizaci udalosti
void i2c0_event(uint32_t event)
{
	;
}

void I2C_Init(void){

	uint32_t temperature, humidity;
	ARM_I2C_STATUS status;
	char buffer[16];

	// Inicializace a konfigurace ovladace I2C
	Driver_I2C1.Initialize(i2c0_event);
	Driver_I2C1.PowerControl(ARM_POWER_FULL);
	Driver_I2C1.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);

}
