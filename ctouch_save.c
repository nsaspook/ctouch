/* #pragma	config	OSC = HS */
#pragma config	WDT = ON, WDTPS = 128
/*
 * This program converts the rs-232 output from a ELO Carroll-Touch touch-screen controller
 * to a format that can be used with the Varian E220/E500 Implanter
 * The Carroll controller must be first programmed 
 * USART1 is the host comm port
 * USART2 is the touch-screen comm port
 * 
 * Fred Brooks, Microchip Inc , April 2009
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
#include <delays.h>

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
				CATCH46=FALSE, CATCH37=FALSE, TSTATUS=TRUE, cdelay=500, NEEDSETUP=FALSE;
volatile long	j=0,alive_led=0;

/*
 * Step #1  The data is allocated into its own section.
 */
#pragma idata bigdata
unsigned char elobuf[BUF_SIZE];
unsigned char elocodes[ELO_SIZE]={
	0x0d,0x0d,0x0d,0x0d,0x0d,0x3c,0x2b,0x44,0x27,0x44,0x3d,0x2a}; 	// initial touch config codes
volatile unsigned int tchar, uchar;
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

  	PORTJ = 0xff;							// all leds off before checking serial ports
	ClrWdt();								// reset the WDT timer
	
  /* Get the character received from the USARTs */

  if (DataRdy2USART()) {					// is data from touch

		c = Read2USART();					// read data from touch
		if (TOUCH) {
			elobuf[i++]=c;
			if (c == 0xFF) {				// end of report
				CATCH=TRUE;
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
			 i=0;			// stop buffer-overflow	
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
	
		    if ((tchar == 0x3C))				// touch reset from host
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
		PORTJ = 0xff;								// all leds on
		ClrWdt();									// reset the WDT timer
	};
}

void elocmdout(unsigned char *elostr)
{
		PORTJ = 0xff;					// led 2 on
		while (Busy2USART()) {};			// wait until the usart is clear
      	putc2USART (elostr[0]);
		wdtdelay(30000);
		while (Busy2USART()) {};			// wait until the usart is clear
		PORTJ = 0xf2;
}

void setuplcd(void)
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
	Delay10TCYx(2);
}

void	DelayPORXLCD(void)
{

}
	
void	DelayXLCD(void)
{

}	


void main (void)
{
	unsigned int	delayc;

  /* Configure all PORT B,E,H,J pins for output */
  TRISJ = 0;
  TRISH = 0;
  TRISE = 0;
  TRISB = 0;
  
  
//  OpenXLCD( FOUR_BIT & LINE_5X10 );
//  while( BusyXLCD() );
//  putrsXLCD( (const far rom char *) "Fred Brooks");
 
  /* 
   * Open the USART configured as
   * 8N1, 9600 baud, in receive INT mode
   */
  Open1USART (USART_TX_INT_OFF &
             USART_RX_INT_ON &
             USART_ASYNCH_MODE &
             USART_EIGHT_BIT &
             USART_CONT_RX &
             USART_BRGH_HIGH, 254);				// 40mhz osc HS		9600 baud
             
  Open2USART (USART_TX_INT_OFF &
             USART_RX_INT_ON &
             USART_ASYNCH_MODE &
             USART_EIGHT_BIT &
             USART_CONT_RX &
             USART_BRGH_HIGH, 254);				// 40mhz osc HS		9600 baud

  /* Enable interrupt priority */
  RCONbits.IPEN = 1;

  /* Make receive interrupt high priority */
  IPR1bits.RCIP = 1;

  /* Enable all high priority interrupts */
  INTCONbits.GIEH = 1;


  /* Display a prompt to the USART ,,, for USART testing only will lockup screen */
//  putrs1USART (
//    (const far rom char *)"\n\rVarian e220 touch-screen converter. Port 1 Fred Brooks, Microchip Inc. April 2009 v0.6 \n\r");
//  putrs2USART (
//    (const far rom char *)"\n\rVarian e220 touch-screen converter. Port 2 Fred Brooks, Microchip Inc. April 2009 v0.6 \n\r");

	setuplcd();							// send lcd touch controller setup codes

//		elocmdout(&elocodes[0]);		
//		elocmdout(&elocodes[1]);
//		elocmdout(&elocodes[2]);
//		elocmdout(&elocodes[3]);		
//		elocmdout(&elocodes[4]);
//		elocmdout(&elocodes[5]);
//		elocmdout(&elocodes[6]);		
//		elocmdout(&elocodes[7]);
//		elocmdout(&elocodes[8]);
//		elocmdout(&elocodes[9]);		
//		elocmdout(&elocodes[10]);
//		elocmdout(&elocodes[11]);

		
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
  PORTJ=0xff;									// set leds to on at powerup/reset
  PORTE=0x0f;
  												// leds from outputs to ground via resistor.
  while (TRUE)
 {
	if (j++ >= BLINK_RATE) 							// delay a bit ok
	{
		PORTH = !PORTH;								// flash H0
		PORTJ=alive_led & 0xfe;								// roll leds cylon style
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
	}
	
 	if (CATCH46)								// allow for touch processing 
	 	{
		 	wdtdelay(cdelay);
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
			 CATCH=FALSE;
			} else {	
		 	 while (Busy1USART()) {};			// wait until the usart is clear
			 putc1USART (0xF5);
			 while (Busy1USART()) {};			// wait until the usart is clear
			 putc1USART (0xFF);					// turn reporting back on
			 CATCH46 = FALSE;
			};
		};
 	if (CATCH37)								// send screen size codes 
	 	{
		 	wdtdelay(cdelay);
		 	while (Busy1USART()) {};			// wait until the usart is clear
			putc1USART (0xF4);
			while (Busy1USART()) {};			// wait until the usart is clear
			putc1USART (0x77);					// turn reporting back on
		 	while (Busy1USART()) {};			// wait until the usart is clear
			putc1USART (0x5F);
			while (Busy1USART()) {};			// wait until the usart is clear
			putc1USART (0xFF);				// turn reporting back on
			CATCH37 = FALSE;
		};
	if (!TSTATUS) {
												// code to restart touch-screen
	};
	
	if (NEEDSETUP) setuplcd();					// send lcdsetup codes to screen	
	
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
 };
}
