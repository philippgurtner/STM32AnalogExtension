/*	Diese Funktionen Beziehen sich auf das ADC-Board. Pins müssen über HAL nach fixen Namen definiert werden. --> Programmierguinde
*
*/

#include "ADCBoard.h"

// Standard Values, could be overwritten

// Define PT100 depends on the used hardware
t_measurement_constants pt100 = {
		3.9083e-3,
		-5.775e-7,
		-4.183e-12,
		 3.85e-3,
		 100,
		 2200			//todo widerstand ist 2 ohm zu hoch --> Nullpunktabgleichen automatisieren :)
};





/* Diese Funktion wird bei einem DRDY Interrupt automatisch ausgeführt
 * Daten sind bereit und können ausgelesen werden.
 */
void ADC_DRDY_InputHandler(void)
{
	// HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	 ADCmeasuremenReady=1;
}


uint32_t ADCinit(void)
{
	HAL_Delay(3);												// minimum delay of 2.2 ms to allow power supplies to settle and power-up reset to complete
	HAL_GPIO_WritePin(ADC_RESET_GPIO_Port,ADC_RESET_Pin,1); 	// Set reset
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,1);			//
	HAL_GPIO_WritePin(ADC_START_GPIO_Port, ADC_START_Pin,0);	// Start by command (Start pin LOW)
	HAL_Delay(10);

	while(ADCregread(STATUS_REG_ADDR) & ADS_RDY)				// Ready bit überprüfen  //todo ist bisschen hart wenn immer in dieser while
	{
		HAL_Delay(5);
	}
	ADCregwrite(STATUS_REG_ADDR, ADCregread(STATUS_REG_ADDR)&~ADS_FL_POR); // Clear FL POR Flag (Redback,...)


	ADCsendCommand(WAKE_COMMAND);		// Sollte nicht sein, jedoch zur sicherheit

	//todo

	//Read all Registers to simplify config
	ADCreadRegs(ID_REG_ADDR,REGISTER_COUNT, au8_registers);


	//Set Mode to single conversation opearation (Always use ADCstart() )
	au8_registers[DATARATE_REG_ADDR] &= ADS_CONVMODE_SS;

	au8_registers[INPMUX_REG_ADDR]	|= (ADS_P_AIN4 | ADS_N_AINCOM); // // Select INPUT (INPMUX- Register): Setzen der Eingänge
	au8_registers[REF_REG_ADDR]		= (ADS_REFINT_ON_ALWAYS+ADS_REFSEL_INT); // Set Internal voltage Reference (2.5V Max inputvoltage = 2.5V)

	ADCwriteRegs(ID_REG_ADDR,REGISTER_COUNT, au8_registers); 	// write all registers



	//ADCregwrite(PGA_REG_ADDR, ADS_PGA_ENABLED);
	//ADCsendCommand(STOP_COMMAND);
	//ADCsendCommand(START_COMMAND);

	ADCstart();
	//HAL_GPIO_WritePin(ADC_START_GPIO_Port, ADC_START_Pin,1);	// Start by Pin

	return 0;
}

/* Schreibt ein Register des ADCs
 * @param	startadress 	-> Zu schreibendes Register
 * @param	data			-> Daten für in das Register
 */
void ADCregwrite(uint8_t startadress, uint8_t data)
{
	uint8_t txdata[1] = {data};
	ADCwriteRegs(startadress, 1, txdata);
}

/* Liest ein Register des ADCs
 * @param	startadress	-> zu lesendes Register
 * @resp	Gelesenes Register
 */
uint8_t ADCregread(uint8_t startadress)
{
	uint8_t rxdata[1] = {0};
	ADCreadRegs(startadress, 1, rxdata);
	return rxdata[0];
}

/* Sendet ADC Komando Start (steuerung über Command, Start-PIN = LOW)
 */
void ADCstart(void)
{
	ADCmeasuremenReady=0;
	ADCsendCommand(START_COMMAND);
}

/* Schreibt mehrere, aufeinanderfolgende Register des ADCs
 * @param	u8_startadress	-> Erstes zu schreibendes Register
 * @param	u8_length		-> Anzahl zu schreibende 8Bit Register
 * @param 	pu8_data		-> Pointer auf Array mit den Daten
 * @resp	erfolgreich =1, sonst 0
 */
uint8_t ADCwriteRegs(uint8_t u8_startadress, uint8_t u8_length, uint8_t *pu8_data)
{
	/*	Protokollaufbau:
	 * 	0b010r'rrrr		Startadress | 0x40 for WriteRegister
	 * 	length -1		0 = 1Byte, 1= 2Byte
	 * 	Data(...)
	 */

	// checks maximal length
	if(u8_length==0 && u8_length > REGISTER_COUNT)	// sonst werden keine Daten gesendet, Max register, 0 ist auch ein register! deshalb >=
	{	
		return 0;
	}
	
	// erstellen eines Array für das senden der Daten
	uint8_t txdata[u8_length+2];
	txdata[0]= u8_startadress | 0x40; 	// set write command
	txdata[1]= u8_length -1; 	// 0x00 is 1 byte
	
	for(uint8_t i=0; i<u8_length; i++)	// Dateiinhalt in txdata plazieren
	{
		txdata[i+2]= *(pu8_data+i);
	}
	
	

	HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin,0);
	HAL_Delay(CHIP_SELECT_WAIT_TIME);

	HAL_SPI_Transmit(&hspi1,txdata,u8_length+2,10);			// +2 for startadress and length, length is only datacount
	
	HAL_Delay(CHIP_SELECT_WAIT_TIME);
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,1);

	return 1;
}

/* Liest mehrere, aufeinanderfolgende Register des ADCs
 * @param	u8_startadress	-> Erstes zu lesendes Register
 * @param	u8_length		-> Anzahl zu lesende 8Bit Register
 * @param 	pu8_rxdata		-> Pointer auf Array mit den Daten
 * @resp	erfolgreich =1, sonst 0	//todo
 */
uint8_t ADCreadRegs(uint8_t u8_startadress, uint8_t u8_length, uint8_t *pu8_rxdata) 	// Speichert rückgabe in rxdata
{
	/*	Protokollaufbau:
	 * 	Senden des Befehles:
	 * 		0b001r'rrrr		Startadress | 0x20 for ReadRegister
	 * 		length -1		0 = 1Byte, 1= 2Byte
	 * 	Lesen der Daten
	 * 	Data(...)
	 */

	// checks maximal length
	if(u8_length==0 && u8_length > REGISTER_COUNT)	// sonst werden keine Daten gesendet, Max register, 0 ist auch ein register! deshalb >=
	{
		return 0;
	}

	uint8_t txdata[2] = {u8_startadress |0x20 , u8_length-1};	// create array for spi transmit (to get a response from ADC)
	
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin,0);
	HAL_Delay(CHIP_SELECT_WAIT_TIME);

	HAL_SPI_Transmit(&hspi1,txdata, 2, 10);			//Transmit TX Data (2 Bytes)
	HAL_SPI_Receive(&hspi1, pu8_rxdata, u8_length, 50);		//todo realy +1?

	HAL_Delay(CHIP_SELECT_WAIT_TIME);
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,1);
	return 1;

}

/*	Senden eines Befehls für den ADC
 * 	@param	u8_opcode	-> befehl aus ADS1204S0x.h
 */
void ADCsendCommand(uint8_t u8_opcode)
{
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin,0);
	HAL_Delay(CHIP_SELECT_WAIT_TIME);

	uint8_t txbuf[1]= {u8_opcode};				//Start Convertion via Command
	HAL_SPI_Transmit(&hspi1,txbuf,1,10);

	HAL_Delay(CHIP_SELECT_WAIT_TIME);
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,1);
	
}

/*	Read positive ADC Input, as negative is AINCOM set
 * 	Die Funktion kann in der ISR von DRDY ausgeführt werden (mode=0), oder immer (mode=1)
 * 	@param mode =1 -> Read with command , mode = 0 -> DRDY is high !
 * 	@return u32_result
 */
uint32_t ADRreadInput(uint8_t mode) 	// 0=DRDY high, 1=Command
{
		uint8_t rxbuf[3]={1,1,1};
		uint32_t u32_result=0;
		uint8_t dStatus[1];
		uint8_t dCRC[1];


		if (mode == 1) // Read by Command
		{
			ADCsendCommand(RDATA_COMMAND);	//RDATA => Read Data
		}

		HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin,0);
		HAL_Delay(CHIP_SELECT_WAIT_TIME);

		//todo

		// if the Status byte is set - grab it
		if( (au8_registers[SYS_REG_ADDR] & ADS_SENDSTATUS_ENABLE) )
		{
			HAL_SPI_Receive(&hspi1, dStatus,1,10);
		}


		HAL_SPI_Receive(&hspi1,rxbuf,3,10); //Read Datarate Register --> Ohne CRC und Status

		// is CRC enabled?
		if( (au8_registers[SYS_REG_ADDR] & ADS_CRC_ENABLE) )
		{
			HAL_SPI_Receive(&hspi1, dCRC,1,10);

			//todo calc CRC
		}


		HAL_Delay(CHIP_SELECT_WAIT_TIME);
		HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,1);

		u32_result = rxbuf[0];
		u32_result = (u32_result<<8) + rxbuf[1];
		u32_result = (u32_result<<8) + rxbuf[2];

		return u32_result;
	
}

/*	Setzen eines Inputchannels des ADC (AIN Neg = AINCOM)
 * 	Kanal wird zuerst gesetzt, dann wird gewartet !!
 * 	@param u8_channel 	-> AIN0... AIN5 (ADS124S06), AIN0...AIN11 (ASD124S08)
 * 	@ return  -> success =1
 */
uint8_t setADCsigleChannel(uint8_t u8_channel)
{
	if (au8_registers[ID_REG_ADDR] & ADS_124S06 && u8_channel > (ADS_P_AIN5 >> 4) )	// DS124S06 has only 6 channels (AIN5 is max)
	{
		return 0;
	}
	else if(u8_channel > (ADS_P_AIN11 >> 4) )						// DS124S08 has 12 channels (AIN11 is max)
	{
		return 0;
	}

	else
	{
		ADCregwrite(INPMUX_REG_ADDR,(u8_channel<<4 | ADS_N_AINCOM));
		HAL_Delay(7); 		// Delay until Input is stable (Datasheet p.36, Table 10. Internal Reference Settling Time)
		return 1;
	}
}

/*	Liest einzelnen ADCKanal aus
 * 	@param 	u8_channel -> zu lesender Kanal
 * 	@resp 	-> ADC output
 */
uint32_t readsingleADCChannel(uint8_t u8_channel)
{
	if(setADCsigleChannel(u8_channel))
	{

		ADCstart();
		while(ADCmeasuremenReady!=1);
		return ADRreadInput(1);


	}
	else
	{
		return 0;
	}

}



float read_temperature(void)		//need be called in a while. Flag newtemperatureavaliable is set if set and must be cleared manualy !
{
	static uint32_t au32_rawdata[3];
	static uint8_t u8_rawcounter;
	static float temperature;

	switch(tempmeasurementstate)
	{
		case MEASURE: 		// ISR CALL
			setADCsigleChannel(u8_rawcounter);
			ADCstart();
			while(ADCmeasuremenReady!=1);
			au32_rawdata[u8_rawcounter]= ADRreadInput(1); 	// Read (DRDY is high)
			u8_rawcounter ++;
			if(u8_rawcounter >= 3)
			{
				tempmeasurementstate = NEWMEASURE;
			}

			return u8_rawcounter;
			break;

		case READY:
			u8_rawcounter=0;
			tempmeasurementstate=MEASURE;

			break;

		// Both response are temperature
		case NEWMEASURE:
			u8_rawcounter=0;
			tempmeasurementstate = NEWTEMP;
			temperature = calctemperature( calc_R_temperature(au32_rawdata, pt100), pt100 );
		case NEWTEMP:
			return temperature;

		default:
			return 0;
	}
	return 0;
}

float calc_R_temperature(uint32_t *pu32_rawdata, t_measurement_constants tempresistor)		// Berechnung R_temp
{
	// Berechnung: Upt100= ADC0-2xADC1 + ADC2
	uint32_t u32_result_ADCsteps=  	pu32_rawdata[0] + pu32_rawdata[2] - 2*(pu32_rawdata[1]);

	// Steps to voltage
	float u32_result_ADCvoltage = 	(float) (fullscale_voltage * u32_result_ADCsteps / bitresulution2 );		// bitresolution2 for the positive spectrum

	//  R= U/I = U / ( UR_I / R_I ) =  U * R_I / UR_I
	float Voltage_R_11 =  (float) (fullscale_voltage * pu32_rawdata[2] / bitresulution2);

	return (u32_result_ADCvoltage * tempresistor.R_11 / Voltage_R_11 );
}

float calctemperature(float Rtemp, t_measurement_constants tempresistor)
{
	if(Rtemp>=100)		// ToDo Temperatur >= 0°C
	{
		/* For Calculate the temperature from the Resistance in positve the Callendar-Van Dusen equation is used.
		 * https://en.wikipedia.org/wiki/Resistance_thermometer
		 * 	T= -A + sqrt(A^2 - 4*B *(1- (Rtemp/R0) )) / 2*B
		 */

		return (-tempresistor.A + sqrt((tempresistor.A * tempresistor.A) - 4*tempresistor.B*(1-(Rtemp/tempresistor.R0))))	/(2*tempresistor.B);
	}
	else
	{
		/*
		 * Negative Temperature
		 * For easy calculating, the same formula is used in negative temperature.
		 * He isn't correct because C isn't 0 as in the positive temperature
		 * In this case we get a measurement error
		 * todo for future improuvement a new calculation
		 */

		return (-tempresistor.A + sqrt((tempresistor.A * tempresistor.A) - 4*tempresistor.B*(1-(Rtemp/tempresistor.R0))))	/(2*tempresistor.B);
	}

}



float calcAnalogvoltage(uint32_t u32_measurement)
{
	return fullscale_voltage * u32_measurement / bitresulution2;
}
