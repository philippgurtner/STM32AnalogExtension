#ifndef ADS124S0x_H_
#define ADS124S0x_H_

#define REGISTER_COUNT 18

/*
 * Define all Register addresses
 * Used for readable adressing
*/

#define ID_REG_ADDR			0x00	// Device ID Register
#define STATUS_REG_ADDR		0x01	// Device Status Register
#define INPMUX_REG_ADDR		0x02	// Input Multiplexer Register
#define PGA_REG_ADDR		0x03	// Gain Setting Register
#define DATARATE_REG_ADDR	0x04	// Data Rate Register
#define REF_REG_ADDR		0x05	// Reference Control Register
#define IDACMAG_REG_ADDR	0x06	// Excitation Current Register 1
#define IDACMUX_REG_ADDR	0x07	// Excitation Current Register 2
#define VBIAS_REG_ADDR		0x08	// Sensor Biasing Register
#define SYS_REG_ADDR		0x09	// System Control Register
#define OFCAL0_REG_ADDR		0x0A	// Offset Calibration Register 1
#define OFCAL1_REG_ADDR		0x0B	// Offset Calibration Register 2
#define OFCAL2_REG_ADDR		0x0C	// Offset Calibration Register 3
#define FSCAL0_REG_ADDR		0x0D	// Gain Calibration Register 1
#define FSCAL1_REG_ADDR		0x0E	// Gain Calibration Register 2
#define FSCAL2_REG_ADDR		0x0F	// Gain Calibration Register 3
#define GPIODAT_REG_ADDR	0x10	// GPIO Data Register
#define GPIOCON_REG_ADDR	0x11	// GPIO Configuration Register




/* 	Commands
*	
*/
#define NOP_COMMAND			0x00	
#define WAKE_COMMAND		0x02
#define SLEEP_COMMAND		0x04
#define RESET_COMMAND		0x06
#define START_COMMAND		0x08
#define STOP_COMMAND		0x0A
#define SFOCAL_COMMAND		0x19
#define SYOCAL_COMMAND		0x16
#define SYGCAL_COMMAND		0x17
#define RDATA_COMMAND		0x12
#define REGRD_COMMAND		0x20
#define REGWR_COMMAND		0x40



/* Register sub masks */
/* 	Device ID (ID) Register
 *	adress=0x00 , reset 0xxx
 *
 *   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *        			RESERVED[4:0]      			 			 |  	      DEV_ID[2:0]
 *
 */
/* Define VER (device version) */
#define ADS_124S08				0x00
#define ADS_124S06				0x01


/*	Device Status (STATUS) Register
 *	address = 0x01 , reset = 0x80
 *
 *   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *   FL_POR  |    nRDY   | FL_P_RAILP| FL_P_RAILN| FL_N_RAILP| FL_N_RAILN| FL_REF_L1 | FL_REF_L0
 *
 */
#define ADS_FL_POR				0x80	// power-on reset flag
#define ADS_RDY					0x40	// Device ready flag
#define ADS_FL_P_RAILP			0x20	// Positive PGA output at positive rail flag
#define ADS_FL_P_RAILN			0x10	// Positive PGA output at negative rail flag
#define ADS_FL_N_RAILP			0x08	// Negative PGA output at positive rail flag(
#define ADS_FL_N_RAILN			0x04	// Negative PGA output at negative rail flag
#define ADS_FL_REF_L1			0x02	// Reference voltage monitor flag, level 1
#define ADS_FL_REF_L0			0x01	// Reference voltage monitor flag, level 0


/*	Input Multiplexer (INPMUX) Register
 *	address = 0x02 , reset = 0x01
 *
 *   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *         			MUXP[3:0]   				 |       			MUXN[3:0]
 *
 */
/* Define the ADC positive input channels (MUXP) */
#define ADS_P_AIN0				0x00 	//(Default)
#define ADS_P_AIN1				0x10	
#define ADS_P_AIN2				0x20
#define ADS_P_AIN3				0x30
#define ADS_P_AIN4				0x40
#define ADS_P_AIN5				0x50
#define ADS_P_AIN6				0x60	// ADS124S08 only
#define ADS_P_AIN7				0x70	// ADS124S08 only
#define ADS_P_AIN8				0x80	// ADS124S08 only
#define ADS_P_AIN9				0x90	// ADS124S08 only
#define ADS_P_AIN10				0xA0	// ADS124S08 only
#define ADS_P_AIN11				0xB0	// ADS124S08 only
#define ADS_P_AINCOM			0xC0
/* Define the ADC negative input channels (MUXN)*/
#define ADS_N_AIN0				0x00
#define ADS_N_AIN1				0x01
#define ADS_N_AIN2				0x02
#define ADS_N_AIN3				0x03
#define ADS_N_AIN4				0x04
#define ADS_N_AIN5				0x05
#define ADS_N_AIN6				0x06	// ADS124S08 only
#define ADS_N_AIN7				0x07	// ADS124S08 only
#define ADS_N_AIN8				0x08	// ADS124S08 only
#define ADS_N_AIN9				0x09	// ADS124S08 only
#define ADS_N_AIN10				0x0A	// ADS124S08 only
#define ADS_N_AIN11				0x0B	// ADS124S08 only
#define ADS_N_AINCOM			0x0C


/* 	Gain Setting (PGA) Register
 *	address = 0x03 , reset 0x00
 *
 *   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *   		DELAY[2:0]		     	 |      PGA_EN[1:0]      |              GAIN[2:0]
 *
 */
/* Define conversion delay in tmod * clock periods */
#define ADS_DELAY_14			0x00	// default
#define ADS_DELAY_25			0x20
#define ADS_DELAY_64			0x40
#define ADS_DELAY_256			0x60
#define ADS_DELAY_1024			0x80
#define ADS_DELAY_2048			0xA0
#define ADS_DELAY_4096			0xC0
#define ADS_DELAY_1				0xE0
/* Define PGA control */
#define ADS_PGA_BYPASS			0x00	// default
#define ADS_PGA_ENABLED			0x08
/* Define Gain */
#define ADS_GAIN_1				0x00	// default
#define ADS_GAIN_2				0x01
#define ADS_GAIN_4				0x02
#define ADS_GAIN_8				0x03
#define ADS_GAIN_16				0x04
#define ADS_GAIN_32				0x05
#define ADS_GAIN_64				0x06
#define ADS_GAIN_128			0x07

/* Data Rate (DATARATE) Register
 *	address = 0x04 , reset = 0x14
 *
 *   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *   G_CHOP  |    CLK    |    MODE   |   FILTER  | 				  DR[3:0]
 *
 */
#define ADS_GLOBALCHOP			0x80	// 0= Disabled ,1= Enabled
#define ADS_CLKSEL_EXT			0x40	// 0= Internal 4.096MHZ Clock ,1= External Clock
#define ADS_CONVMODE_SS			0x20	// 0= Continius Concersion , 1= Singleshot
#define ADS_FILTERTYPE_LL		0x10	// 1= Low-latency filter , 0= Sinc filter
/* Define the data rate */
#define ADS_DR_2_5				0x00
#define ADS_DR_5				0x01
#define ADS_DR_10				0x02
#define ADS_DR_16				0x03
#define ADS_DR_20				0x04	// default
#define ADS_DR_50				0x05
#define ADS_DR_60				0x06
#define ADS_DR_100				0x07
#define ADS_DR_200				0x08
#define ADS_DR_400				0x09
#define ADS_DR_800				0x0A
#define ADS_DR_1000				0x0B
#define ADS_DR_2000				0x0C
#define ADS_DR_4000				0x0D

/*	Reference Control (REF) Register
 * 	address = 0x05 , reset = 0x10
 *
 *   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *	  FL_REF_EN[1:0]	 | nREFP_BUF | nREFN_BUF | 		REFSEL[1:0]		 | 		REFCON[1:0]
 *
 */
 /* Reference monitor configuration*/
#define ADS_FLAG_REF_DISABLE	0x00	// default
#define ADS_FLAG_REF_EN_L0		0x40
#define ADS_FLAG_REF_EN_BOTH	0x80
#define ADS_FLAG_REF_EN_10M		0xC0
/*Positive reference buffer bypass */
#define ADS_REFP_BYP_DISABLE	0x20	 
#define ADS_REFP_BYP_ENABLE		0x00	// default
/* Negative reference buffer bypass*/
#define ADS_REFN_BYP_DISABLE	0x10	// default
#define ADS_REFN_BYP_ENABLE		0x00
/*Reference input selection */
#define ADS_REFSEL_P0			0x00	// default REFP0, REFN0 
#define ADS_REFSEL_P1			0x04	// REFP1, REFN1
#define ADS_REFSEL_INT			0x08	// Internal 2.5-V reference
/* Internal voltage reference configuration */
#define ADS_REFINT_OFF			0x00	// default
#define ADS_REFINT_ON_PDWN		0x01	// but powers down in power-down mode
#define ADS_REFINT_ON_ALWAYS	0x02	// always on, even in power-down mode

/*	Excitation Current Register 1 (IDACMAG)
 *	address = 0x06 , reset = 0x00
 *
 *   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 * FL_RAIL_EN|	  PSW	 |     0     | 		0	 | 			    	IMAG[3:0]
 *
 */
#define ADS_FLAG_RAIL_ENABLE	0x80	
#define ADS_FLAG_RAIL_DISABLE	0x00	// default
#define ADS_PSW_OPEN			0x00	// default
#define ADS_PSW_CLOSED			0x40	
/* IDAC magnitude selection */
#define ADS_IDACMAG_OFF			0x00	// default
#define ADS_IDACMAG_10			0x01
#define ADS_IDACMAG_50			0x02
#define ADS_IDACMAG_100			0x03
#define ADS_IDACMAG_250			0x04
#define ADS_IDACMAG_500			0x05
#define ADS_IDACMAG_750			0x06
#define ADS_IDACMAG_1000		0x07
#define ADS_IDACMAG_1500		0x08
#define ADS_IDACMAG_2000		0x09

/* 	Excitation Current Register 2 (IDACMUX)
 *	 address = 0x07 , reset = 0xFF
 *	
 *   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *                    I2MUX[3:0]                 |                   I1MUX[3:0]
 *
 */
/* Define IDAC2 Output */
#define ADS_IDAC2_AIN0			0x00
#define ADS_IDAC2_AIN1			0x10
#define ADS_IDAC2_AIN2			0x20
#define ADS_IDAC2_AIN3			0x30
#define ADS_IDAC2_AIN4			0x40
#define ADS_IDAC2_AIN5			0x50
#define ADS_IDAC2_AIN6			0x60	// ADS124s08 , REFP1 ADS124S06
#define ADS_IDAC2_AIN7			0x70	// ADS124s08 , REFN1 ADS124S06
#define ADS_IDAC2_AIN8			0x80	// ADS124s08 only
#define ADS_IDAC2_AIN9			0x90	// ADS124s08 only
#define ADS_IDAC2_AIN10			0xA0	// ADS124s08 only
#define ADS_IDAC2_AIN11			0xB0	// ADS124s08 only
#define ADS_IDAC2_AINCOM		0xC0
#define ADS_IDAC2_OFF			0xF0	// default
/* Define IDAC1 Output */
#define ADS_IDAC1_AIN0			0x00
#define ADS_IDAC1_AIN1			0x01
#define ADS_IDAC1_AIN2			0x02
#define ADS_IDAC1_AIN3			0x03
#define ADS_IDAC1_AIN4			0x04
#define ADS_IDAC1_AIN5			0x05
#define ADS_IDAC1_AIN6			0x06	// ADS124s08 , REFP1 ADS124S06
#define ADS_IDAC1_AIN7			0x07	// ADS124s08 , REFN1 ADS124S06
#define ADS_IDAC1_AIN8			0x08	// ADS124s08 only
#define ADS_IDAC1_AIN9			0x09	// ADS124s08 only
#define ADS_IDAC1_AIN10			0x0A	// ADS124s08 only
#define ADS_IDAC1_AIN11			0x0B	// ADS124s08 only
#define ADS_IDAC1_AINCOM		0x0C
#define ADS_IDAC1_OFF			0x0F	// default

/* Sensor Biasing (VBIAS) Register
 *	address = 0x08 , reset = 0x00
 *
 *   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *  VB_LEVEL | 	VB_AINC  |  VB_AIN5  |  VB_AIN4  |  VB_AIN3  |  VB_AIN2  |  VB_AIN1  |  VB_AIN0
 *
 */
#define ADS_VBIAS_LVL_DIV2		0x00	// default
#define ADS_VBIAS_LVL_DIV12		0x80
/* Define VBIAS here */
#define ADS_VB_AINC				0x40	// 0 = disconected default
#define ADS_VB_AIN5				0x20	// 0 = disconected default
#define ADS_VB_AIN4				0x10	// 0 = disconected default
#define ADS_VB_AIN3				0x08	// 0 = disconected default
#define ADS_VB_AIN2				0x04	// 0 = disconected default
#define ADS_VB_AIN1				0x02	// 0 = disconected default
#define ADS_VB_AIN0				0x01	// 0 = disconected default

/* System Control (SYS) Register
 *	address = 0x09 , reset = 0x10
 *
 *   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *			   SYS_MON[2:0]			 |	   CAL_SAMP[1:0]     |  TIMEOUT  | 	  CRC	 | SENDSTAT
 *
 */
 /* System monitor configuration*/
#define ADS_SYS_MON_OFF			0x00	// default
#define ADS_SYS_MON_SHORT		0x20
#define ADS_SYS_MON_TEMP		0x40
#define ADS_SYS_MON_ADIV4		0x60
#define ADS_SYS_MON_DDIV4		0x80
#define ADS_SYS_MON_BCS_2		0xA0
#define ADS_SYS_MON_BCS_1		0xC0
#define ADS_SYS_MON_BCS_10		0xE0
/* Calibration sample size selection */
#define ADS_CALSAMPLE_1			0x00
#define ADS_CALSAMPLE_4			0x08
#define ADS_CALSAMPLE_8			0x10
#define ADS_CALSAMPLE_16		0x18
/* SPI timeout enable */
#define ADS_TIMEOUT_DISABLE		0x00	// default
#define ADS_TIMEOUT_ENABLE		0x04
/* CRC enable */
#define ADS_CRC_DISABLE			0x00	// default
#define ADS_CRC_ENABLE			0x02
/* STATUS byte enable */
#define ADS_SENDSTATUS_DISABLE	0x00	// default
#define ADS_SENDSTATUS_ENABLE	0x01

/* 	Offset Calibration Register 1 (OFCAL0)
 *	address = 0x0A , reset = 0x00
 *
 *   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *                                         OFC[7:0]
 */
 
 
/* 	Offset Calibration Register 2 (OFCAL1)
 *	address = 0x0B , reset = 0x00
 *   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *                                         OFC[15:8]
 */
 
/* 	Offset Calibration Register 3 (OFCAL2)
 *	address = 0x0C , reset = 0x00
 *   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *                                         OFC[23:16]
 */
 
/* 	Gain Calibration Register 1 (FSCAL0)
 *	address = 0x0D , reset = 0x00
 *   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *                                         FSC[7:0]
 */

/* 	Gain Calibration Register 2 (FSCAL1)
 *	address = 0x0E , reset = 0x00
 *   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *                                         FSC[15:8]
 */
 
/* 	Gain Calibration Register 3 (FSCAL2)
 *	address = 0x0F , reset = 0x10
 *   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *                                         FSC[23:16]
 */

/* 	GPIO Data (GPIODAT) Register
 *	address = 0x10 , reset = 0x00
 *   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *                      DIR[3:0]    			 | 					DAT[3:0]
 *
 * Define GPIO configuration (0-Output; 1- Input) here */
#define ADS_GPIO0_DIR_INPUT		0x10
#define ADS_GPIO1_DIR_INPUT		0x20
#define ADS_GPIO2_DIR_INPUT		0x40
#define ADS_GPIO3_DIR_INPUT		0x80


 
 
/*	GGPIO Configuration (GPIOCON)
 *	address = 0x10 , reset = 0x00
 *	 Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
 *--------------------------------------------------------------------------------------------
 *     0	 |	   0	 |	   0	 |	   0     |                    CON[3:0]
 *
 * 
 * Define GPIO configuration (0-Analog Input; 1-GPIO) here */
#define ADS_GPIO0_GPIO		0x10
#define ADS_GPIO1_GPIO		0x20
#define ADS_GPIO2_GPIO		0x40
#define ADS_GPIO3_GPIO		0x80

/*
 *
 */
/* Lengths of conversion data components */
#define DATA_LENGTH			3
#define STATUS_LENGTH		1
#define CRC_LENGTH			1
/*
 * The time we have to wait for the CS GPIO to actually
 * pull down before we start sending SCLKs
 *
 */
#define CHIP_SELECT_WAIT_TIME	0

//?????????????????????????????????????????????????????????????????????????????????????????????????
//?????????????????????????????????????????????????????????????????????????????????????????????????
/* Flag to signal that we are in the process of collecting data */
#define DATA_MODE_NORMAL	0x00
#define DATA_MODE_STATUS	0x01
#define DATA_MODE_CRC		0x02

 /* The clock rate of the internal XTAL... */
#define DEVICE_ICLK 16384000


#define SPI_HAS_TRANSACTION 0

#endif /* ADS124S0x_H_ */
