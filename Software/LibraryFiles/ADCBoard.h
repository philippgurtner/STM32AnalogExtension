#ifndef ADCBoard_H_
#define ADCBoard_H_

// Includes
#include <math.h>
#include "ADS1204S0x.h"
#include "stm32f0xx_hal.h"
#include "spi.h"

// HAL Hardware Definitions 
//(With generation out of STM32cubeMX should be deleted in main.h)
#define ADC_CS_Pin GPIO_PIN_5
#define ADC_CS_GPIO_Port GPIOF
#define ADC_START_Pin GPIO_PIN_4
#define ADC_START_GPIO_Port GPIOC
#define ADC_RESET_Pin GPIO_PIN_5
#define ADC_RESET_GPIO_Port GPIOC
#define ADC_DRDY_Pin GPIO_PIN_0
#define ADC_DRDY_GPIO_Port GPIOB
#define ADC_DRDY_EXTI_IRQn EXTI0_1_IRQn
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOC
#define LD3_Pin GPIO_PIN_9
#define LD3_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA


// Definitions
#define fullscale_voltage 2.5		// fullscale_voltage in Volt
#define bitresulution2 8388608		// 2^24 /2 --> Fullscale positiv

// Global variables
float f_measuredtemperature;		// Global variable for temperature
uint8_t ADCmeasuremenReady;			// Flag for measurement
float f_Channel3Measure;			// Global variable for extern input 3
uint8_t au8_registers[REGISTER_COUNT];	// Storrage for all registers (for later uses)


// Enum for statemachine temperaturemeasure
typedef enum {
	DISABLED,
	READY,			// Ready to measure
	NEWMEASURE,		// Data are completly read
	NEWTEMP,		// Data are calculated to temp
	MEASURE			// busy
}t_temperaturstate;

extern t_temperaturstate tempmeasurementstate;

// Struct for measurement constants (depending of hardware)
typedef struct {
	float A 	;
	float B 	;
	float C 	;
	float alpha ;
	float R0 	;
	float R_11  ;		// Resistor 11 on ADCBoard (for current calculations )
}t_measurement_constants;



// Functions
extern uint32_t ADCinit(void);

// ADC read registers
extern uint8_t ADCreadRegs(uint8_t u8_startadress, uint8_t u8_length, uint8_t *pu8_rxdata); 	// Read multiple registers
extern uint8_t ADCregread(uint8_t startadress);									// Read single register

// ADC write registers
extern uint8_t ADCwriteRegs(uint8_t startadress, uint8_t length, uint8_t *data);	// Write multiple registers
extern void ADCregwrite(uint8_t startadress, uint8_t data);						// Write single register


// ADC Commands
extern void ADCsendCommand(uint8_t op_code);
extern void ADCstart(void);						// Write START command


extern uint8_t setADCsigleChannel(uint8_t u8_channel);

// Calculations for temperature
extern float calctemperature(float Rtemp, t_measurement_constants tempresistor);				// Calculate Tempeature out of Rtemp
extern float calc_R_temperature(uint32_t *pu32_rawdata, t_measurement_constants tempresistor);	// Calculates Rtemp
extern float calcAnalogvoltage(uint32_t u32_measurement);										// Calculates analog voltage from input

// Data read funcions
extern float read_temperature(void);
extern uint32_t readsingleADCChannel(uint8_t u8_channel);			// Set Channel to required channel, delay until channel is stable, measure...
extern uint32_t ADRreadInput(uint8_t mode);					// Read data from ADC, mode 0= Command, 1= ISR (DRDY must be high)


/* Interrupt
* This Function should be called in the EXTI0_1_IRQHandler()
* in stm32f0xx_it.c
*/
extern void ADC_DRDY_InputHandler(void);

/*
void EXTI0_1_IRQHandler(void)
{
	ADC_DRDY_InputHandler();
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
*/




#endif /* ADCBoard_H_ */
