/*Use of AI / Cognitive Assistance Software is not allowed in any evaluation, assessment or exercise.*/
/*=============================================================================
	File Name:	ELNC6011KPLab5S.c  
	Author:		KPerera
	Date:		19/03/2025
	Modified:	None
	© Fanshawe College, 2025

	Description: This program creates a data structure for 3 ADC sensor channels
				to sample 10 samples in every sencond. Then, calculates the
				the average and displays the high limit, low limit and average
				of every single channel. This program runs on MPLAB IDE and 
				PIC18F45K22 microcontroller is being used. Lab 1 used to build 
				hardware circuit and PIC18F45K22 datasheet has been used.
=============================================================================*/

/* Preprocessor ===============================================================
   Hardware Configuration Bits ==============================================*/
#pragma config FOSC		= INTIO67
#pragma config PLLCFG	= OFF
#pragma config PRICLKEN = ON
#pragma config FCMEN	= OFF
#pragma config IESO		= OFF
#pragma config PWRTEN	= OFF 
#pragma config BOREN	= ON
#pragma config BORV		= 285 
#pragma config WDTEN	= OFF
#pragma config PBADEN	= OFF
#pragma config LVP		= OFF
#pragma config MCLRE	= EXTMCLR

// Libraries ==================================================================
#include <p18f45k22.h>
#include <stdio.h>
#include <stdlib.h>
#include <usart.h>

// Constants  =================================================================

#define TRUE	1	
#define FALSE	0
#define CNTRL	1
#define	MYADDY	375
#define STEPRES	0.0049	
#define	SAMPLESIZE	10
#define TXBUFSIZE	35
#define BUFSIZE	35
#define TOKENSIZE	10
#define SENSORCOUNT	3
#define ONESEC	10
#define	FIVESEC	5
#define T0FLAG	INTCONbits.TMR0IF
#define RC1FLAG	PIR1bits.RC1IF
#define	PBMASK	0xF0
#define	STEPPERMASK	0x0F
#define STEPPERPORT	LATB
#define	LEDLIGHTING	LATCbits.LATC0
#define	LEDCOOLER	LATCbits.LATC1
#define	LEDHEATER	LATCbits.LATC2
#define	LEDFAN		LATCbits.LATC3
#define LEDSPKLR	LATCbits.LATC4
#define	PATTERNCOUNT	4
#define	NOPRESS	0xF0
#define MODE	0xE0
#define	CHANSEL	0xD0
#define INC		0xB0
#define	DEC		0X70
#define STEP	3
//sensor m and c values (y = mx + c) 
#define TEMPM	0.01
#define	TEMPC	0.5

#define HUMDM	0.05

#define	CO2M	0.00036
#define	CO2C	0.0017	



// Prototypes  =================================================================
void isr();

#pragma code interruptVector = 0x0008

void interruptVector()
{
	_asm
		GOTO isr
	_endasm
} // eo isr::

#pragma code

// Global Variables  ==========================================================
typedef struct
{
	int samples[SAMPLESIZE];
	int avg;
	int insert;
	int highL;
	int lowL;
	
	char avgReady;
	
}sensorCh_t;

sensorCh_t sensors[SENSORCOUNT];

char patterns[PATTERNCOUNT] = {0x01, 0x02, 0x04, 0x08};

char txBuf[TXBUFSIZE];

char *tokens[TOKENSIZE];

char conlim[ ] = {"CONLIM"};

typedef struct
{
	char currentPat;
	char patCount;
	int currentPos;
	int setPos;
	char mtrFlag;
	
}stepper_t;

typedef struct
{
	char chanSel;
	char mode;
	char pbState;
	char lastState;
	
}pbs_t;

stepper_t vent;

pbs_t pbs;

typedef char flag_t;

typedef struct
{
	flag_t chngeLFlag;
	flag_t baseSenRdy; 
	
}events_t;

events_t modifyLim; 

typedef struct
{
	char buf[BUFSIZE];
	int insertCount;
	char senRdy;
	
}serial_t;

serial_t rcvr;
serial_t trmt;

// Functions  =================================================================

/*>>> setOSC4MHz: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		17/05/2024
Modified:	KPerera
Desc:		This function will select a frequecy via input arguemnt and it will 
			run until the oscillator stablizes at that frequency.
Input: 		char freq, char, input arguement brings frequency value into the 
			function and make a selection according to it.
Returns:	None 
 ============================================================================*/
void setOSC4MHz( char freq)
{
	switch (freq)
	{
		case 16:	
			OSCCON = 0x72;
			break;
		case 8:	
			OSCCON = 0x62;
			break;
		case 4:	
			OSCCON = 0x52;
			break; 
		case 2:	
			OSCCON = 0x42;
			break;
		case 1:	
			OSCCON = 0x32;
			break;
	}
	while (!OSCCONbits.HFIOFS);
} // eo setOSC4MHz::

/*>>> configPorts: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		24/05/2024
Modified:	KPerera
Desc:		This function configures all 5 ports. Unused pins have been set to 
			digital inputs. RA4 to RA7 have been used as pushbutton digital 
			inputs, and RD0 to RD7 have been used as digital outputs.
Input: 		None 
Returns:	None 
 ============================================================================*/
void configPorts()
{
	// Port A configuration
	ANSELA = 0x07;
	LATA = 0x00;
	TRISA = 0xFF;
	
	// Port B configuration
	ANSELB = 0x00;
	LATB = 0x00;
	TRISB = 0xF0;
	
	// Port C configuration
	ANSELC = 0x00;
	LATC = 0x00;
	TRISC = 0xE0;
	
	// Port D configuration
	ANSELD = 0x00;
	LATD = 0x00;
	TRISD = 0xFF;
	
	// Port E configuration
	ANSELE = 0x00;
	LATE = 0x00;
	TRISE = 0xFF;
} // eo configPorts::

/*>>> resetTMR0: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		14/06/2024
Modified:	None
Desc:		This function resets the timer0 when it overflows(when event happens)
			by resetting the flag to 0, and assign values to timer0 high byte and 
			the lower byte according the false start. 
Input: 		None
Returns:	None 
 ============================================================================*/
void resetTMR0()
{
	T0FLAG = FALSE;	// Reset the timer0 flag bit to 0
	TMR0H = 0x3C;	// Set the timer0 high byte for the false start
	TMR0L = 0xB0;	// Set the timer0 low byte for the false start
	
} // eo resetTMR0::

/*>>> configTMR0: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		14/06/2024
Modified:	KPerera
Desc:		This function configures timer0 control register by selecting 16-bit
			mode, internal oscillator clock, increment starts on low to high, and
			assigning the prescaler value of 16. Also, it runs the resetTMR0 fuction
			to reset the timer.
Input: 		None
Returns:	None 
 ============================================================================*/
void configTMR0()
{
	T0CON = 0x90;
	resetTMR0();
} // eo configTMR0::


/*>>> configADC: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		07/06/2024
Modified:	KPerera
Desc:		This function configures ADCON0 register by selecting the analog
			channel and enabling ADC, ADCON1 register by slecting correct (+)
			and (-) reference voltages, and ADCON2 register by right justifing 
			the reading, number of TADs and selecting ADC clock selection.
Input: 		None
Returns:	None 
 ============================================================================*/
void configADC()
{
	ADCON0 = 0x01;
	ADCON1 = 0x00;
	ADCON2 = 0xA9;
	
} // eo configADC::

/*>>> sp2Config: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		21/06/2024
Modified:	KPerera
Desc:		This function configures serial port 1 by assigning values to the 
			BAUDCON1 register, RCSTA1, TXSTA1 by enabling TX1 and RX1, and assinging
			a value to SPBRG1 according to the 9600 Baud Rate. 
Input: 		None
Returns:	None 
 ============================================================================*/
void sp2Config()
{
	SPBRG2 = 12;
	BAUDCON2 = 0x40;
	RCSTA2 = 0x90;
	TXSTA2 = 0x26;
	
} // eo sp2Config::

/*>>> sp1Config: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		21/06/2024
Modified:	KPerera
Desc:		This function configures serial port 1 by assigning values to the 
			BAUDCON1 register, RCSTA1, TXSTA1 by enabling TX1 and RX1, and assinging
			a value to SPBRG1 according to the 9600 Baud Rate. 
Input: 		None
Returns:	None 
 ============================================================================*/
void sp1Config()
{
	SPBRG1 = 25;
	BAUDCON1 = 0x40;
	RCSTA1 = 0x90;
	TXSTA1 = 0x26;
	
} // eo sp1Config::

/*>>> getADCSample: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		07/06/2024
Modified:	None
Desc:		This function will run until the oscillator stablizes at 4 MHZ.
Input: 		char channel, This input arguement brings frequency value into the 
			function and make a selection according to it.
Returns:	int / 10-bit ADC value 
 ============================================================================*/
int getADCSample(char channel)
{
	ADCON0bits.CHS = channel;
	ADCON0bits.GO = TRUE;
	
	while (ADCON0bits.GO);
	
	return ADRES;
	
} // eo getADCSample::

/*>>> configInts: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		19/07/2024
Modified:	KPerera
Desc:		This function enables global interrupts, peripheral interrupts and 
			receiver 1 in UART.
Input: 		None
Returns:	None 
 ============================================================================*/
void configInts()
{
	INTCON = 0xE0;
	RCONbits.IPEN = FALSE;	
	PIE1bits.RC1IE = TRUE;	
} // eo configInts::

/*>>> systemInitialization: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		07/06/2024
Modified:	KPerera
Desc:		This function calls and run setOSC4MHz(4), configPorts() and configADC(), 
			configTMR0() function and sp1Config.
Input: 		None
Returns:	None 
 ============================================================================*/
void systemInitialization()
{
	setOSC4MHz(4);
	configPorts();
	configADC();
	sp1Config();
	sp2Config();
	configTMR0();
		
} // eo systemInitialization::

/*>>> initSensorCh: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		22/01/2025
Modified:	KPerera
Desc:		Initialize the data structure sensorCh_t to 0.
Input: 		sensorCh_t *senPtr, this input arguement brings data data structure
			pointer into the function to access every member in it.
Returns:	None 
 ============================================================================*/
void initSensorCh(sensorCh_t *senPtr, char chID)
{
	int index = 0;
	for(index = 0; index < SAMPLESIZE; index++)
	{
		senPtr -> samples[index] = 0;
	} //eo for initializing the samples array
	
	senPtr -> avg = 0;
	senPtr -> insert = 0;
	senPtr -> avgReady = 0;

	switch (chID)
	{
		case 0:
			sensors[0].highL = 35;
			sensors[0].lowL = 15;
			break;
	
		case 1: 
			sensors[1].highL = 65;
			sensors[1].lowL = 35;
			break;
		case 2:
			sensors[2].highL = 1400; 
			sensors[2].lowL = 650;
			break;
		
		default:
			break;		
	}
	
} // eo initSensorCh::

/*>>> initStepper: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		29/01/2025
Modified:	KPerera
Desc:		Initialize the data structure sensorCh_t to 0.
Input: 		sensorCh_t *senPtr, this input arguement brings data data structure
			pointer into the function to access every member in it.
Returns:	None 
 ============================================================================*/
void initStepper(stepper_t *vent)
{
	vent -> currentPat = patterns[0];
	vent -> patCount = 0;
	vent -> currentPos = 0;
	vent -> setPos = 0;
	vent -> mtrFlag = FALSE;
	
} // eo initStepper::

/*>>> initStepper: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		29/01/2025
Modified:	KPerera
Desc:		Initialize the data structure sensorCh_t to 0.
Input: 		sensorCh_t *senPtr, this input arguement brings data data structure
			pointer into the function to access every member in it.
Returns:	None 
 ============================================================================*/
void initPBs(pbs_t *pbs)
{
	pbs -> chanSel = 0;
	pbs -> mode = FALSE;
	pbs -> pbState = 0;
	pbs -> lastState = PBMASK;

} // eo initStepper::

/*>>> initModifyLim: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		29/01/2025
Modified:	KPerera
Desc:		Initialize the data structure sensorCh_t to 0.
Input: 		sensorCh_t *senPtr, this input arguement brings data data structure
			pointer into the function to access every member in it.
Returns:	None 
 ============================================================================*/
void initModifyLim(events_t *limits)
{
	limits -> chngeLFlag = FALSE;
	limits -> baseSenRdy = FALSE;
		
} // eo initModifyLim::


/*>>> initReceiver: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		29/01/2025
Modified:	KPerera
Desc:		Initialize the data structure sensorCh_t to 0.
Input: 		sensorCh_t *senPtr, this input arguement brings data data structure
			pointer into the function to access every member in it.
Returns:	None 
 ============================================================================*/
void initReceiver(serial_t *rcPtr)
{
	int index = 0;
	
	for (index = 0; index < BUFSIZE; index++)
	{
		rcPtr -> buf[index] = 0;
	}
	rcPtr -> insertCount = 0;
	rcPtr -> senRdy = FALSE;	
	
} // eo initReceiver1::

/*>>> initReceiver: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		29/01/2025
Modified:	KPerera
Desc:		Initialize the data structure sensorCh_t to 0.
Input: 		sensorCh_t *senPtr, this input arguement brings data data structure
			pointer into the function to access every member in it.
Returns:	None 
 ============================================================================*/
void initTYransmitter(serial_t *txPtr)
{
	int index = 0;
	
	for (index = 0; index < BUFSIZE; index++)
	{
		txPtr -> buf[index] = 0;
	}
	txPtr -> insertCount = 0;
	txPtr -> senRdy = FALSE;	
	
} // eo initReceiver1::

/*>>> motorControl: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		11/03/2025
Modified:	KPerera
Desc:		Initialize the data structure sensorCh_t to 0.
Input: 		sensorCh_t *senPtr, this input arguement brings data data structure
			pointer into the function to access every member in it.
Returns:	None 
 ============================================================================*/
void motorControl(int newSetPos)
{
	vent.setPos = newSetPos;
	if (vent.setPos > vent.currentPos)
	{
		vent.patCount++;
		if (vent.patCount >= PATTERNCOUNT)
		{
			vent.patCount = 0;
		}
		vent.currentPat = patterns[ vent.patCount];
		vent.currentPos += STEP;

		STEPPERPORT = STEPPERMASK & vent.currentPat;
	}
						
	if (vent.setPos < vent.currentPos)
	{
		vent.patCount--;
		if (vent.patCount <= 0)
		{
			vent.patCount = PATTERNCOUNT - 1;
		}
		vent.currentPat = patterns[ vent.patCount];
		vent.currentPos -= STEP;
				
		STEPPERPORT = STEPPERMASK & vent.currentPat;
	}	

} // eo motorControl::

/*>>> calcCheckSum: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		11/03/2025
Modified:	KPerera
Desc:		Initialize the data structure sensorCh_t to 0.
Input: 		sensorCh_t *senPtr, this input arguement brings data data structure
			pointer into the function to access every member in it.
Returns:	None 
 ============================================================================*/
char calcCheckSum(char *ptr)
{
	char checkSum = 0;
	
	while (*ptr)
	{
		checkSum ^= *ptr;
		ptr++;	
	}
	
	return checkSum;
} // eo calcCheckSum::

/*>>> validateSen: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		11/03/2025
Modified:	KPerera
Desc:		Initialize the data structure sensorCh_t to 0.
Input: 		sensorCh_t *senPtr, this input arguement brings data data structure
			pointer into the function to access every member in it.
Returns:	None 
 ============================================================================*/
char validateSen(char *ptr)
{
	char newCS = 0, receivedCS = 0;
	int count = strlen( ptr );
	char csRdy = FALSE;
	
	while (csRdy != TRUE)
	{
		if ( *(ptr + count) == ',')
		{
			*(ptr + count) = 0;
			receivedCS = atoi( ptr + count + 1);
			csRdy = TRUE; 
		}
		count--;	
	}
	newCS = calcCheckSum(ptr);
	
	if ( newCS == receivedCS)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
} // eo validateSen::

/*>>> parseStr: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		11/03/2025
Modified:	KPerera
Desc:		Initialize the data structure sensorCh_t to 0.
Input: 		sensorCh_t *senPtr, this input arguement brings data data structure
			pointer into the function to access every member in it.
Returns:	None 
 ============================================================================*/
void parseStr(char *ptr)
{
	char tokenCounter = 0;
	
	while (*ptr)
	{
		if ( *ptr == '$' || *ptr == ',')
		{
			*ptr = 0;
			tokens[tokenCounter] = ptr + 1;
			tokenCounter++;
		}
		ptr++;
	}	
}// eo parseStr::

/*>>> executeStr: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		11/03/2025
Modified:	KPerera
Desc:		Initialize the data structure sensorCh_t to 0.
Input: 		sensorCh_t *senPtr, this input arguement brings data data structure
			pointer into the function to access every member in it.
Returns:	None 
 ============================================================================*/
void executeStr(char *ptr)
{
	if ( strcmp ( conlim, tokens[ 0 ] == 0 ))
	{
		if ( atoi( tokens[ 4 ] ))
		{
			sensors[ atoi( tokens[ 3 ])].highL = atoi( tokens[ 5 ]);
		}
		else
		{
			sensors[ atoi( tokens[ 3 ])].lowL = atoi( tokens[ 5 ]);
		}
	}
}// eo executeStr::

/*>>> displayWindow: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		07/06/2024
Modified:	KPerera
Desc:		This function calls and run setOSC4MHz(4), configPorts() and configADC(), 
			configTMR0() function and sp1Config.
Input: 		None
Returns:	None 
 ============================================================================*/
int displayWindow()
{
	int tempVal, humdVal, co2Val = 0; 
	
	if (sensors[ 0 ].avgReady)
	{
		tempVal = (((sensors[ 0 ].avg)* STEPRES) - TEMPC) / TEMPM;
	}
	else
	{
		tempVal = 0;
	}
	if (sensors[ 1 ].avgReady)
	{
		humdVal = ((sensors[ 1 ].avg)* STEPRES) / HUMDM;
	}
	else
	{
		humdVal = 0;
		
	}
	if (sensors[ 2 ].avgReady)
	{
		co2Val = (((sensors[ 2 ].avg)* STEPRES) - CO2C) / CO2M;
	}
	else
	{
		co2Val = 0;
	}
	
	printf("Sensor System: 375\n\n\r");
	printf("Channel: %d\t\t Mode: %S\n\n\r", pbs.chanSel, (pbs.mode? "High": "Low") );
	printf("Temp: \t%d %cC\t\t Humd: \t%d %%\t\t CO2: \t%d ppm\n\r", tempVal, 248, humdVal, co2Val);
	printf("HL: \t%d %cC\t\t HL: \t%d %%\t\t HL: \t%d ppm\n\r", sensors[ 0 ].highL,248, sensors[ 1 ].highL, sensors[ 2 ].highL);
	printf("LL: \t%d %cC\t\t LL: \t%d %%\t\t LL: \t%d ppm\n\n\r", sensors[ 0 ].lowL, 248, sensors[ 1 ].lowL, sensors[ 2 ].lowL);
	printf("Heat: %S\t Cool: %S\t Fan: %S\n\r", (LEDHEATER? "ON": "OFF"), (LEDCOOLER? "ON": "OFF"), (LEDFAN? "ON": "OFF") );
	printf("Lights: %S\t SPKLR: %S\n\n\r", (LEDLIGHTING? "ON": "OFF"), (LEDSPKLR? "ON": "OFF") );
	printf("Vent\n\r");
	printf("Set Position: %d%c\t Current Position: %d%c\n\rData Pattern: 0x0%x\n\r", vent.setPos, 248, vent.currentPos, 248, patterns[vent.patCount] );
	
	if (tempVal < sensors[ 0 ].highL && tempVal > sensors[ 0 ].lowL)
	{
		LEDCOOLER = FALSE;
		LEDHEATER = FALSE;
		LEDFAN = FALSE;
	}
	else if (tempVal > sensors[ 0 ].highL)
	{
		LEDCOOLER = TRUE;
		LEDHEATER = FALSE;
		LEDFAN = TRUE;
		vent.setPos = 90;
	}
	else if (tempVal < sensors[ 0 ].lowL)
	{
		LEDCOOLER = FALSE;
		LEDHEATER = TRUE;
		LEDFAN = TRUE;
		vent.setPos = 6;
	}
	else if (humdVal < sensors[ 1 ].highL && humdVal > sensors[ 1 ].lowL)
	{
		LEDSPKLR = FALSE;
	}
	else if (humdVal > sensors[ 1 ].highL)
	{
		LEDSPKLR = FALSE;
		vent.setPos = 66;
	}
	else if (humdVal < sensors[ 1 ].lowL)
	{
		LEDSPKLR = TRUE;
		vent.setPos = 12;
	}
	else if (co2Val > sensors[ 2 ].highL)
	{
		LEDFAN = TRUE;
		vent.setPos = 9;
	}
	else if (co2Val < sensors[ 2 ].lowL)
	{
		LEDFAN = TRUE;
		vent.setPos = 90;
	}
	
	return vent.setPos;
} // eo displayWindow::

/*>>> displayCommSen: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		07/06/2024
Modified:	KPerera
Desc:		This function calls and run setOSC4MHz(4), configPorts() and configADC(), 
			configTMR0() function and sp1Config.
Input: 		None
Returns:	None 
 ============================================================================*/
void displayCommSen()
{
	printf("\n\r%s\n\r", trmt.buf);

} // eo displayCommSen::

/*>>> changeMode: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		29/02/2025
Modified:	KPerera
Desc:		This function calls and run setOSC4MHz(4), configPorts() and configADC(), 
			configTMR0() function and sp1Config.
Input: 		None
Returns:	None 
 ============================================================================*/
void changeMode()
{
	pbs.mode  = !pbs.mode;
	
} // eo changeMode::

/*>>> changeChannel: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		29/02/2025
Modified:	KPerera
Desc:		This function calls and run setOSC4MHz(4), configPorts() and configADC(), 
			configTMR0() function and sp1Config.
Input: 		None
Returns:	None 
 ============================================================================*/
void changeChannel()
{
	pbs.chanSel++;
	
	if (pbs.chanSel >= SENSORCOUNT)
	{
		pbs.chanSel = 0;
	}
		
} // eo changeChannel::

/*>>> incrementLimit: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		29/02/2025
Modified:	KPerera
Desc:		This function calls and run setOSC4MHz(4), configPorts() and configADC(), 
			configTMR0() function and sp1Config.
Input: 		None
Returns:	None 
 ============================================================================*/
void incrementLimit()
{
	if (pbs.mode)
	{
		sensors[ pbs.chanSel].highL++;
	}
	else
	{
		sensors[ pbs.chanSel].lowL++;
	}
		
} // eo incrementLimit::

/*>>> decrementLimit: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		29/02/2025
Modified:	KPerera
Desc:		This function calls and run setOSC4MHz(4), configPorts() and configADC(), 
			configTMR0() function and sp1Config.
Input: 		None
Returns:	None 
 ============================================================================*/
void decrementLimit()
{
	if (pbs.mode)
	{
		sensors[ pbs.chanSel].highL--;
	}
	else
	{
		sensors[ pbs.chanSel].lowL--;
	}
		
} // eo decrementLimit::

/*>>> isr: ===========================================================
Author:		KPerera                                                                                                                                      
Date:		07/06/2024
Modified:	KPerera
Desc:		This function interrpt service routine when receiver 1 flag is up.
Input: 		None
Returns:	None 
 ============================================================================*/
void isr()
{
	int hold = 0;
	
	if (RC1FLAG)
	{
		hold = RCREG2;
		
		if ( hold  == '$')
		{
			rcvr.insertCount = 0;
		}
		if ( hold == '\r')
		{
			hold = 0;
			rcvr.senRdy = TRUE;
		}
		rcvr.buf[ rcvr.insertCount] = hold;
		rcvr.insertCount++;
	}
	INTCON |= 0xC0;	
}// eo isr::


/*=== MAIN: FUNCTION ==========================================================
 ============================================================================*/
void main( void )
{
	char chID = 0;
	char index1 = 0;
	char secCount = 0;
	long sum = 0;
	
	systemInitialization();

	for (chID = 0; chID < SENSORCOUNT; chID++)
	{
		initSensorCh( &sensors[chID], chID);
	} //eo for
	
	initPBs( &pbs );
	initStepper( &vent );
	initModifyLim( &modifyLim );
	initReceiver( &rcvr );
	initTYransmitter( &trmt );
	

	while(TRUE)
	{
		pbs.pbState = (PORTA & PBMASK);
		
		if (pbs.pbState != NOPRESS && pbs.pbState != pbs.lastState)
		{
			pbs.lastState = pbs.pbState;
			
			switch (pbs.pbState)
			{	
				case MODE:
					changeMode();
					break;
				case CHANSEL:
					changeChannel();
					break;
				case INC:
					incrementLimit();
					break;
				case DEC:
					decrementLimit();
					break;
				default:
					break;
			}//eo switch
		}
		else if (pbs.pbState == NOPRESS)
		{
			pbs.lastState = 0;
		}
		
		if (pbs.pbState == INC || pbs.pbState == DEC)
		{
			modifyLim.chngeLFlag = TRUE;
		}
		
		if ( modifyLim.chngeLFlag )
		{
			modifyLim.chngeLFlag  = FALSE;
			
			if (pbs.mode)
			{
				sprintf( trmt.buf, "$%s,%d,%d,%d,%d,%d", conlim, CNTRL, MYADDY, pbs.chanSel, pbs.mode, sensors[ pbs.chanSel].highL);
			}
			else
			{
				sprintf( trmt.buf, "$%s,%d,%d,%d,%d,%d", conlim, CNTRL, MYADDY, pbs.chanSel, pbs.mode, sensors[ pbs.chanSel].lowL);
			}
			
			sprintf( trmt.buf, "%s,%d\r", trmt.buf, calcCheckSum( trmt.buf ));
			modifyLim.baseSenRdy = TRUE;
			puts2USART( trmt.buf );
			trmt.insertCount = 0;
		}
		
		if (T0FLAG)
		{
			resetTMR0();
			secCount++;
			motorControl(vent.setPos);
			
			if ( secCount == ONESEC )
			{
				secCount = 0;
				
				for ( chID = 0; chID < SENSORCOUNT; chID++ )
				{
					sensors[ chID ].samples[ sensors[ chID ].insert] = getADCSample(chID);
					sensors[ chID ].insert++;
					
					if (sensors[ chID ].insert >= SAMPLESIZE)
					{
						sensors[ chID ].insert = 0;
						sensors[ chID ].avgReady = TRUE;
					} //eo if

					if (sensors[ chID ].avgReady)
					{
						sum = 0;
						for (index1 = 0; index1 < SAMPLESIZE; index1++)
						{
							sum += sensors[chID].samples[ index1 ];

						} //eo for
						sensors[ chID ].avg = sum / SAMPLESIZE;
						
					} //eo if
				} //eo for chID
				
				printf("\033[H\033[2J");
				displayWindow();
				if (modifyLim.baseSenRdy)
				{	
					if (trmt.insertCount < FIVESEC)
					{
						displayCommSen();
						trmt.insertCount++;
					}
					else
					{	
						
						modifyLim.baseSenRdy = FALSE;
						
					}
				}
			} //eo if
		} //eo if
		
	} //eo while

} // eo main::
