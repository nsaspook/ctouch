#pragma	config	OSC = HSPLL
#pragma config	WDT = ON, WDTPS = 128
/*
 * This program converts the rs-232 output from a ELO Carroll-Touch touch-screen controller
 * to a format that can be used with the Varian E220/E500 Implanter
 * The Carroll controller must be first programmed 
 * USART1 is the host comm port
 * USART2 is the touch-screen comm port
 * A jumper between PORTJ pin6 and +vcc enables debug mode
 * 2x16 LCD status panel and 8 led status lights.
 * 
 * Fred Brooks, Microchip Inc , May 2009
 * Gresham, Oregon
 * 
 * 
 * This application is designed for use with the
 * ET-BASE PIC8722 board and  device.  
 * It was written using example2.c as the starting point.
 * 
 * 1.  Creating large data objects
 * 2.  Reading from and writing to the USART. 
 * 3.  Interrupt handling (#pragma interrupt, interrupt vectors, and
 *     interrupt service routines)
 * 4.  System header files
 * 5.  Processor-specific header files
 * 6.  #pragma sectiontype
 * 7.  Inline assembly
 */
#include <p18F8722.h>
#include <usart.h>
#include <xlcd.h>
#include <delays.h>
#include <string.h>
#include <stdlib.h>

void rx_handler (void);

#define BUF_SIZE 128
#define	CMD_SIZE 2
#define	CMD_OVERFLOW	CMD_SIZE*12
#define ELO_SIZE 12
#define ELO_SEQ 10
#define FALSE	0
#define TRUE	1
#define	BLINK_RATE	10000

volatile int	CATCH=FALSE, i=0 ,y=0, LED_UP=TRUE, z=0, TOUCH=FALSE, UNTOUCH=FALSE,
				CATCH46=FALSE, CATCH37=FALSE, TSTATUS=TRUE, cdelay=900, NEEDSETUP=FALSE,
				CRTFIX_DEBUG=1, LCD_GO=TRUE;
volatile long	j=0,alive_led=0, touch_count=0, resync_count=0,rawint_count=0, status_count=0;

/*
 * Step #1  The data is allocated into its own section.
 */
#pragma idata bigdata
char			lcdstr[18]="CRTFIX E0.92   |",lcdstatus[18]="D",lcdstatus_touched[18]="Touched",
				tmp_str[18]="   ",debugstr[18]="DEBUG jmpr#6   |";
unsigned char elobuf[BUF_SIZE], spinchr, commchr=' ';
unsigned char elocodes[ELO_SIZE]={
	0x0d,0x0d,0x0d,0x0d,0x0d,0x3c,0x2b,0x44,0x27,0x44,0x3d,0x2a}; 	// initial touch config codes
volatile unsigned int tchar, uchar, debug_port=0;
#pragma idata

#pragma code rx_interrupt = 0x8
void rx_int (void)
{
  _asm goto rx_handler _endasm
}
#pragma code

#pragma interrupt rx_handler
void rx_handler (void)
{
  unsigned char c;
  unsigned int dcount;

	rawint_count++;							// debug counters
	LCD_GO=FALSE;							// shutdown LCD display processing for now
  	PORTJ = PORTJ|0xf0;						// all leds off before checking serial ports
  	commchr='*';
	ClrWdt();								// reset the WDT timer
	
  /* Get the character received from the USARTs */

  if (DataRdy2USART()) {					// is data from touch
		c = Read2USART();					// read data from touch
		if (TOUCH) {
			elobuf[i++]=c;
			if (c == 0xFF) {				// end of report
				CATCH=TRUE;
				TOUCH=FALSE;				// stop buffering touch data.
			};	
		};	
		if (c == 0xFE) {					// looks like a touch report
			TOUCH=TRUE;
			CATCH=FALSE;
			i=0;
		};
		if (c == 0xF5) {					// looks like a status report
			TSTATUS=TRUE;
		};
		if (i>(BUF_SIZE-1)){
			 i=0;							// stop buffer-overflow	
			 TOUCH=FALSE;
			 CATCH=FALSE;
		};	 
	};


  if (DataRdy1USART()) {					// is data from host
	  	tchar = Read1USART();				// read from host

	    if ((tchar == 0x46))				// send one report to host
    {
	    	CATCH46 = TRUE;

	}

	    if ((tchar == 0x37))				// start of touch scan read
    {
	    	CATCH37 = TRUE;

	}
	
		    if ((tchar == 0x3C))			// touch reset from host
    {
	    	NEEDSETUP = FALSE;

	}
	};

	/* clear interrupt flag */
    PIR1bits.RCIF = 0;
}



void wdtdelay(unsigned int delay)
{
unsigned int	dcount;

		for (dcount=0; dcount<=delay; dcount++) {	// delay a bit
		PORTJ = PORTJ|0xf0;							// all leds on
		ClrWdt();									// reset the WDT timer
	};
}

void elocmdout(unsigned char *elostr)
{
		PORTJ = PORTJ|0x20;					// led 2 on
		while (Busy2USART()) {};			// wait until the usart is clear
      	putc2USART (elostr[0]);
		wdtdelay(30000);
		while (Busy2USART()) {};			// wait until the usart is clear
		PORTJ = PORTJ|0x40;
}

void setup_lcd(void)
{
		elocmdout(&elocodes[0]);		
		elocmdout(&elocodes[1]);
		elocmdout(&elocodes[2]);
		elocmdout(&elocodes[3]);		
		elocmdout(&elocodes[4]);
		elocmdout(&elocodes[5]);
		elocmdout(&elocodes[6]);		
		elocmdout(&elocodes[7]);
		elocmdout(&elocodes[8]);
		elocmdout(&elocodes[9]);		
		elocmdout(&elocodes[10]);
		elocmdout(&elocodes[11]);
		NEEDSETUP=FALSE;
}	

void	DelayFor18TCY(void)
{
Delay10TCYx(60);
}

//------------------------------------------
void DelayPORXLCD (void) {
Delay1KTCYx(150);// Delay of 15ms
		// Cycles = (TimeDelay * Fosc) / 4
		// Cycles = (15ms * 4MHz) / 4
		// Cycles = 15,000
return;
}

//------------------------------------------
void DelayXLCD (void) {
Delay1KTCYx(50); // Delay of 5ms
		// Cycles = (TimeDelay * Fosc) / 4
		// Cycles = (5ms * 4MHz) / 4
		// Cycles = 5,000
return;
}


void main (void)
{
	unsigned int	delayc, spinner=0;

  /* Configure all PORT B,E,H,J pins for output */
  TRISJ = 0;
  TRISH = 0;
  TRISE = 0;
  TRISB = 0;
  PORTH = 0;
  TRISD = 0xff;
  
  DelayPORXLCD();
  OpenXLCD( FOUR_BIT & LINES_5X7 );
  while( BusyXLCD() );
  putsXLCD(lcdstr);
  while( BusyXLCD() );
 
  /* 
   * Open the USART configured as
   * 8N1, 9600 baud, in receive INT mode
   */
  Open1USART (USART_TX_INT_OFF &
             USART_RX_INT_ON &
             USART_ASYNCH_MODE &
             USART_EIGHT_BIT &
             USART_CONT_RX &
             USART_BRGH_LOW, 64);				// 40mhz osc HS		9600 baud, USART_BRGH_LOW and 64, for 9600 baud @ 40MHz   (0.16% ERROR IN RATE)
												// USART_BRGH_HIGH and 64, for 9600 baud @ 10MHz  (0.16% ERROR IN RATE)
             
  Open2USART (USART_TX_INT_OFF &
             USART_RX_INT_ON &
             USART_ASYNCH_MODE &
             USART_EIGHT_BIT &
             USART_CONT_RX &
             USART_BRGH_LOW, 64);				// 40mhz osc HS		9600 baud, USART_BRGH_LOW and 64, for 9600 baud @ 40MHz   (0.16% ERROR IN RATE)
												// USART_BRGH_HIGH and 64, for 9600 baud @ 10MHz  (0.16% ERROR IN RATE)

  /* Enable interrupt priority */
  RCONbits.IPEN = 1;

  /* Make receive interrupt high priority */
  IPR1bits.RCIP = 1;

  /* Enable all high priority interrupts */
  INTCONbits.GIEH = 1;


 setup_lcd();							// send lcd touch controller setup codes
		
 while (DataRdy1USART()) {		// dump 1 rx data
	z = Read1USART();
 };
 while (DataRdy2USART()) {		// dump 2 rx data
	z = Read2USART();
 };

  /* Enable interrupt priority */
  RCONbits.IPEN = 1;

  /* Make receive interrupt high priority */
  IPR1bits.RCIP = 1;

  /* Enable all high priority interrupts */
  INTCONbits.GIEH = 1;

  /* Loop forever */
  PORTJ=0x00;									// set leds to on at powerup/reset
  PORTE=0x0f;
  												// leds from outputs to ground via resistor.
  while (TRUE)
 {
	if (j++ >= BLINK_RATE) 							// delay a bit ok
	{
		if (spinner++ > 2) {
			spinchr='|';
			spinner=0;
		} else {
			spinchr='-';
		}		
		PORTJ=(alive_led & 0xfe);								// roll leds cylon style
		if (LED_UP && (alive_led!=0)) {
			alive_led=alive_led*2;
		} else {
			if (alive_led!=0) alive_led=alive_led/2;
		} 
		if (alive_led<2) {
			alive_led=2;
			LED_UP=TRUE;
		} else {
			if (alive_led>8) {
				alive_led=8;
				LED_UP=FALSE;
			}
		}
		j=0;
		ClrWdt();								// reset the WDT timer
/*	send data to lcd status panel */
		if (LCD_GO) {
			if (PORTD>63) {						// debug jumper 6
				while( BusyXLCD() );
				SetDDRamAddr(0xc0);
				while( BusyXLCD() );
				putsXLCD(lcdstatus);
  				while( BusyXLCD() );
				itoa(touch_count,tmp_str+1);
				putsXLCD(tmp_str);
  				while( BusyXLCD() );
				itoa(status_count,tmp_str+1);
				putsXLCD(tmp_str);
  				while( BusyXLCD() );
				itoa(resync_count,tmp_str+1);
				putsXLCD(tmp_str);
  				while( BusyXLCD() );
 			}; 		
			lcdstr[14]=commchr;
			lcdstr[15]=spinchr;
			SetDDRamAddr(0);
			if (PORTD>63) {						// debug jumper 6
				debugstr[14]=commchr;
				debugstr[15]=spinchr;
				while( BusyXLCD() );
  				putsXLCD(debugstr);
			} else {	
				while( BusyXLCD() );
  				putsXLCD(lcdstr);
  			};		
  			commchr=' ';
  		};		
	}
	
 	if (CATCH46)								// allow for touch processing 
	 	{
		 	// try to dump buffers
		while (DataRdy1USART()) {		// dump 1 rx data
		 z = Read1USART();
		};
		while (DataRdy2USART()) {		// dump 2 rx data
		 z = Read2USART();
		}; 

		 	if (CATCH) {						// send the buffered touch report
			 while (Busy1USART()) {};			// wait until the usart is clear
			 putc1USART (0xFE);
			 while (Busy1USART()) {};			// wait until the usart is clear
			 putc1USART (elobuf[0]);			// turn reporting back on
			 while (Busy1USART()) {};			// wait until the usart is clear
			 putc1USART (elobuf[1]);
			 while (Busy1USART()) {};			// wait until the usart is clear
			 putc1USART (0xFF);					// turn reporting back on
			 i=0;
			 touch_count++;
			 CATCH=FALSE;
			 CATCH46=FALSE;
			} else {	
		 	 while (Busy1USART()) {};			// wait until the usart is clear
			 putc1USART (0xF5);					// send status report
			 while (Busy1USART()) {};			// wait until the usart is clear
			 putc1USART (0xFF);					// end of report
			 status_count++;
			 CATCH46 = FALSE;
			};
		};
 	if (CATCH37)								// send screen size codes 
	 	{
		 	wdtdelay(cdelay);
		 	while (Busy2USART()) {};			// wait until the usart is clear
			putc2USART (0x3D);					// send clear buffer to touch
		 	while (Busy1USART()) {};			// wait until the usart is clear
			putc1USART (0xF4);					// send status report
			while (Busy1USART()) {};			// wait until the usart is clear
			putc1USART (0x77);					// touch parm
		 	while (Busy1USART()) {};			// wait until the usart is clear
			putc1USART (0x5F);					// touch parm
			while (Busy1USART()) {};			// wait until the usart is clear
			putc1USART (0xFF);					// end of report
			resync_count++;
			CATCH37 = FALSE;
		};
	if (!TSTATUS) {
												// code to restart touch-screen
	};
	
	if (TOUCH) {
												// do nothing now.
	};

	if (NEEDSETUP) setup_lcd();					// send lcdsetup codes to screen	
	
	/*	check for port errors	*/
	if (RCSTA1bits.OERR == 1)
	{
		PORTE != 0x02;						// overrun clear error and reenable receiver 1
		RCSTA1bits.CREN = 0;
		RCSTA1bits.CREN = 1;
	}
	if (RCSTA2bits.OERR == 1)
	{
		PORTE != 0x08;						// overrun clear error and reenable receiver 2
		RCSTA2bits.CREN = 0;
		RCSTA2bits.CREN = 1;
	}
	LCD_GO=TRUE;							// processing is done so restart LCD displays.
 };
}
