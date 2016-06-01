// ---------------------------------------------------------------------------------
// Objective:
//	Read the temperature from the sensor and send it via UART everytime the 'enter' button
//	is pressed
//
// ------------------------------------------------------------------------------------

//#include "driverlib.h"
//#include "../msp430_driverlib_2_60_00_02/driverlib/MSP430FR5xx_6xx/driverlib.h"
#include "stdint.h"
//#include <string.h>
//#include <stdlib.h>
#include <msp430.h>



// ------------------------------------------------------------------------------
// UART interface
// -------------------------------------------------------------------------------
#define UART_TX_READY		(UCA0IFG & UCTXIFG)
#define UART_RX_READY		(UCA0IFG & UCRXIFG)
#define UART_TX_DONE		(UCA0IFG & UCTXCPTIFG)
#define UART_RESET_TX_DONE	(UCA0IFG &= ~UCTXCPTIFG)

// sensor addresses and registers
#define LIGHT_SENSOR_ADDRESS 0b1000100u
#define CMD_GET_LIGHT_LSB 0x02u
#define CMD_GET_LIGHT_MSB 0x03u

#define HUMIDITY_SENSOR_ADDRESS 0b1000000u
#define CMD_GET_HUMIDITY 0b11100101u
#define CMD_GET_TEMP_SENSIRION 0b11100011u

#define PRESSURE_SENSOR_ADDRESS 0b1110111u
#define CMD_GET_PRESSURE 0x74u
#define CMD_READ_PRESSURE_MSB 0xF6u
#define CMD_READ_PRESSURE_LSB 0xF7u

//Function Prototypes
static void send_char(char);
static void send_str(char*);
static void send_int(int, int);
static void new_line();
void update();

uint8_t displayState = 0;


//because leds are not linear when dimmed with pwm
uint8_t pwm[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
		0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
		0x02, 0x02, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x04, 0x04, 0x04, 0x04, 0x04, 0x05, 0x05, 0x05,
		0x05, 0x06, 0x06, 0x06, 0x07, 0x07, 0x07, 0x08, 0x08, 0x08, 0x09, 0x09, 0x0A, 0x0A, 0x0B, 0x0B,
		0x0C, 0x0C, 0x0D, 0x0D, 0x0E, 0x0F, 0x0F, 0x10, 0x11, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
		0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1F, 0x20, 0x21, 0x23, 0x24, 0x26, 0x27, 0x29, 0x2B, 0x2C,
		0x2E, 0x30, 0x32, 0x34, 0x36, 0x38, 0x3A, 0x3C, 0x3E, 0x40, 0x43, 0x45, 0x47, 0x4A, 0x4C, 0x4F,
		0x51, 0x54, 0x57, 0x59, 0x5C, 0x5F, 0x62, 0x64, 0x67, 0x6A, 0x6D, 0x70, 0x73, 0x76, 0x79, 0x7C,
		0x7F, 0x82, 0x85, 0x88, 0x8B, 0x8E, 0x91, 0x94, 0x97, 0x9A, 0x9C, 0x9F, 0xA2, 0xA5, 0xA7, 0xAA,
		0xAD, 0xAF, 0xB2, 0xB4, 0xB7, 0xB9, 0xBB, 0xBE, 0xC0, 0xC2, 0xC4, 0xC6, 0xC8, 0xCA, 0xCC, 0xCE,
		0xD0, 0xD2, 0xD3, 0xD5, 0xD7, 0xD8, 0xDA, 0xDB, 0xDD, 0xDE, 0xDF, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5,
		0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xED, 0xEE, 0xEF, 0xEF, 0xF0, 0xF1, 0xF1, 0xF2,
		0xF2, 0xF3, 0xF3, 0xF4, 0xF4, 0xF5, 0xF5, 0xF6, 0xF6, 0xF6, 0xF7, 0xF7, 0xF7, 0xF8, 0xF8, 0xF8,
		0xF9, 0xF9, 0xF9, 0xF9, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFC,
		0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD,
		0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF};

// -----------------------------------------------------------------------------
// initialization functions
// -----------------------------------------------------------------------------
void init_gpio(void)
{

    // ---------------------------------------------------------------------
    // Initializing GPIOs
    // ---------------------------------------------------------------------
    // UART1 port
    // 2.0 : TX
    // 2.1 : RX
    P2SEL1 |= (BIT0 | BIT1);
    P2SEL0 &= ~(BIT0 | BIT1);
    //GPIO for LED (for pwm )
    //1.4: R
    //1.5: G
    //3.4: B
    P1DIR |= BIT4;
    P1DIR |= BIT5;
    P3DIR |= BIT4;
    P1SEL0 |= BIT4 | BIT5;
    P3SEL0 |= BIT4;
    //s2 on P1.1
    P1DIR &= ~BIT1;		// Input direction
    P1REN |= BIT1;		// Enabling pull-up/down
    P1OUT |= BIT1;		// Pull-up
    P1IES &= ~BIT1;		// Interrupt edge select: high-to-low
    P1IE |= BIT1;		// Enabling interrupt for this port
    P1IFG &= ~BIT1;      // Clean P1.1 Interrupt Flag
}

void init_clock(void)
{

	CSCTL0_H = CSKEY >> 8;	// Unlock CS registers
  	CSCTL1 = DCOFSEL_3 | DCORSEL;		// Set DCO to 8MHz
  	CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;	// Set SMCLK = MCLK = DCO
  	CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;	// Set all dividers to 1
	/*
    // -------------------------------------------------------------------
    // Setting the clock
    // We use the low frequency VLO oscillator, that works in LPM3 mode, to
    //	source the timer.
    // -------------------------------------------------------------------
	CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_0);	// DCO set to 1 MHz
	CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);	//MCLK set to 1MHz
	CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_32);	//SMCLK set to 8MHz
	CS_turnOnSMCLK();
	*/
}

void init_UART(void)
{
    // ---------------------------------------------------------------------
    // Initialization of the UART
    // - No parity
    // - LSB first
    // - One stop bit
    // - 8-bit data
    // - UART mode (asynchronous)
    // - ACLK clock is used
    // - Erroneous characters rejected
    // - No break characters interrupts
    // - Not dormant
    // - Software reset disabled
    // - Baudrate: 9600 baud
    // ---------------------------------------------------------------------
    UCA0CTLW0 = UCSWRST;	// Reset
    UCA0CTLW0 = UCSSEL__SMCLK;
    // Setting the baudrate
    UCA0BR0 = 52;
    UCA0BR1 = 0x00;
    UCA0MCTLW |= UCOS16 | UCBRF_1;
    UCA0CTLW0 &= ~UCSWRST;	// Unreset
    // Enabling interrupt on character reception
    UCA0IE |= UCRXIE;
}

void init_timer() {
	 // Configure Timer0_B
	  TB0CCR0 = (255)-1;                         // PWM Period
	  TB0CCTL1 = OUTMOD_7;                      // CCR1 reset/set
	  //TB0CCTL1 |= CCIE; 	//Enable Interrupts on CCR1
	  TB0CCR1 = 0;                            // CCR1 PWM duty cycle
	  TB0CCTL2 = OUTMOD_7;                      // CCR2 reset/set
	  //TB0CCTL2 |= CCIE; 	//Enable Interrupts on CCR1
	  TB0CCR2 = 0;                            // CCR2 PWM duty cycle
	  TB0CCTL3 = OUTMOD_7;
	  //TB0CCTL3 |= CCIE; 	//Enable Interrupts on CCR1
	  TB0CCR3 = 0;
	  TB0CTL = TBSSEL__ACLK | MC__UP | TBCLR;  // SMCLK, up mode, clear TAR
}

void i2c_init(void)
{
	  // Configure USCI_B0 for I2C mode

	  P1SEL1 |= BIT6 | BIT7;                    // I2C pins

	  UCB0CTLW0 |= UCSWRST;                     // Software reset enabled
	  UCB0CTLW0 |= UCMODE_3 | UCMST | UCSYNC;   // I2C mode, Master mode, sync

	  UCB0BRW = 24;                         // baudrate = SMCLK / 8
	  //UCB0I2CSA = TEMP_SENSOR_ADDRESS;               // Slave address
	  UCB0TBCNT = 0x0002;                       // number of bytes to be received
	  UCB0CTL1 &= ~UCSWRST;

	  UCB0IE |= UCNACKIE;             // transmit and NACK interrupt enable

}

void init_RTC() {

	RTCCTL01 = RTCTEVIE | RTCRDYIE | RTCBCD | RTCHOLD;
	                                            // RTC enable, BCD mode, RTC hold
	                                            // enable RTC read ready interrupt
	                                            // enable RTC time event interrupt

	RTCYEAR = 0x2010;                       // Year = 0x2010
	RTCMON = 0x4;                           // Month = 0x04 = April
	RTCDAY = 0x05;                          // Day = 0x05 = 5th
	RTCDOW = 0x01;                          // Day of week = 0x01 = Monday
	RTCHOUR = 0x10;                         // Hour = 0x10
	RTCMIN = 0x32;                          // Minute = 0x32
	RTCSEC = 0x45;                          // Seconds = 0x45
    RTCADOWDAY = 0x2;                       // RTC Day of week alarm = 0x2
    RTCADAY = 0x20;                         // RTC Day Alarm = 0x20
    RTCAHOUR = 0x10;                        // RTC Hour Alarm
    RTCAMIN = 0x23;                         // RTC Minute Alarm

    RTCCTL01 &= ~(RTCHOLD);                 // Start RTC
}


// -------------------------------------------------------------------------
// UART interrupt handler
// -------------------------------------------------------------------------

// Interrupt handler for input characters
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
	if (UART_RX_READY) {
		char c = UCA0RXBUF;
		if (c == '\n') {
			LPM0_EXIT;
		}
	}
}


// -------------------------------------------------------------------------
// UART SEND/PARSE STRING FUNCTIONS
// -------------------------------------------------------------------------

// Send a single character using UART
static void send_char(char c)
{
	 // Sending data through UART test
	 while (!UART_TX_READY);
	 UCA0TXBUF = c;
	 while (!UART_TX_DONE);
	 UART_RESET_TX_DONE;
}

// Send a string using UART
static void send_str(char str[])
{
	while (*str != '\0') {
		send_char(*str);
		str++;
	}
}

static void send_int(int n, int b)
{
	static char digits[] = "0123456789ABCDEF";
	char buffer[5];
	int i = 0, sign;

	if ((sign = n) < 0) {
		n = -n;
	}

	do {
		buffer[i++] = digits[n % b];
	} while ((n /= b) > 0);

	if (sign < 0) {
		buffer[i++] = '-';
	}

	while(--i >= 0) {
		send_char(buffer[i]);
	}
}

static void new_line()
{
	send_char('\n');
	send_char(0xD);
}

// interrupt handler for i2c
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) USCI_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
  {
    case USCI_NONE:          break;         // Vector 0: No interrupts
    case USCI_I2C_UCALIFG:   break;         // Vector 2: ALIFG
    case USCI_I2C_UCNACKIFG:                // Vector 4: NACKIFG
      UCB0CTLW0 |= UCTXSTP;                 // resend stop if NACK
      break;
    case USCI_I2C_UCSTTIFG:

    	__no_operation();
    	break;         // Vector 6: STTIFG
    case USCI_I2C_UCSTPIFG:  break;         // Vector 8: STPIFG
    case USCI_I2C_UCRXIFG3:  break;         // Vector 10: RXIFG3
    case USCI_I2C_UCTXIFG3:  break;         // Vector 12: TXIFG3
    case USCI_I2C_UCRXIFG2:  break;         // Vector 14: RXIFG2
    case USCI_I2C_UCTXIFG2:  break;         // Vector 16: TXIFG2
    case USCI_I2C_UCRXIFG1:  break;         // Vector 18: RXIFG1
    case USCI_I2C_UCTXIFG1:  break;         // Vector 20: TXIFG1
    case USCI_I2C_UCRXIFG0:  break;         // Vector 22: RXIFG0
    case USCI_I2C_UCTXIFG0:                 // Vector 24: TXIFG0
      break;
    default: break;
  }
}


// -------------------------------------------------------------------------
// RTC interrupt handler
// -------------------------------------------------------------------------

//Interrupt handler for the RTC
#pragma vector=RTC_VECTOR
__interrupt void RTC_B_ISR(void)
{
    switch(__even_in_range(RTCIV,16))
    {
    case 0: break;      //No interrupts
    case 2:             //RTCRDYIFG (This interrupt happens every second.)
        update();
        break;
    default: break;
    }
}

/*
 * interrupt handler for switch
 */

#pragma vector=PORT1_VECTOR
__interrupt void sw2_interrupt_handler (void) {
	displayState++;
	displayState %= 3;
	send_int(displayState, 10);
	new_line();
	update();
	__delay_cycles(64000);			//debouncing
   while (! (P1IN & BIT1));       //Wait for the release of the button
   __delay_cycles(64000);			//debouncing


   // Reseting the interrupt flag
   P1IFG &= ~BIT1;                //Clean P1.1 Interrupt Flag
}


void setRGB(int r, int g, int b){
	TB0CCR1 = (int)pwm[r];
	TB0CCR2 = (int)pwm[g];
	TB0CCR3 = (int)pwm[b];
}

void i2c_writeByte(uint8_t data)
{
	// Ensure previous stop condition got sent
	while (UCB0CTL1 & UCTXSTP);

	UCB0CTLW0 |= UCTR + UCTXSTT;	// Transmit mode + Start condition

	// Poll transmit interrupt flag
	while (!(UCB0IFG & UCTXIFG0));

	// Write data byte
	UCB0TXBUF = data;

	// Poll transmit interrupt flag
	while (!(UCB0IFG & UCTXIFG0));

	// Send stop condition
	UCB0CTLW0 |= UCTXSTP;

	// Reset transmit interrupt flag
	UCB0IFG &= ~UCTXIFG0;
}

unsigned int readData() {

	unsigned int t = 0;

	UCB0CTLW0 &= ~UCTR;			// Receive mode
	UCB0CTL1 |= UCTXSTT;  // I2C start condition
	while(!(UCB0IFG & UCRXIFG));
	t |= UCB0RXBUF;				//read low byte

	while(!(UCB0IFG & UCRXIFG));
	t |= UCB0RXBUF<<8;			//read high byte
	// Send stop condition
	UCB0CTLW0 |= UCTXSTP;

	return t;
}

int getTemperature() {

	/*UCB0I2CSA = TEMP_SENSOR_ADDRESS;

	i2c_writeByte(CMD_GET_TEMPERATURE);

	int t = readData();
	t = (t >> 2)/32;
	//t = -46.85 + 175.72 * (t/65536);
	return t;*/
	UCB0I2CSA = HUMIDITY_SENSOR_ADDRESS;
	i2c_writeByte(CMD_GET_TEMP_SENSIRION);
	unsigned int t = readData();
	t &= ~0b11; //last 2 bits are status bits, not data
	int temp = 0;
	temp = 100*(-46.85+175.72*((double)t/65536.0));
	return temp;
}

unsigned int getLight(){
	//send_str("LIGHT START: ");
	UCB0I2CSA = LIGHT_SENSOR_ADDRESS;

	//i2c_writeByte(0x0);
	//i2c_writeByte(0b10100001);

	i2c_writeByte(0x03);
	int l = readData();

	return l;

	/*i2c_writeByte(CMD_GET_LIGHT_MSB);
	unsigned int l = 0;
	UCB0CTLW0 &= ~UCTR;			// Receive mode
	UCB0CTL1 |= UCTXSTT;  // I2C start condition
	while(!(UCB0IFG & UCRXIFG));
	l |= UCB0RXBUF;				//read low byte
	while(!(UCB0IFG & UCRXIFG));
	l |= UCB0RXBUF<<8;
	UCB0CTLW0 |= UCTXSTP;

	unsigned int l2 = 0;
	i2c_writeByte(CMD_GET_LIGHT_LSB);
	UCB0CTLW0 &= ~UCTR;			// Receive mode
	UCB0CTL1 |= UCTXSTT;  // I2C start condition
	while(!(UCB0IFG & UCRXIFG));
	l2 |= UCB0RXBUF;				//read low byte
	while(!(UCB0IFG & UCRXIFG));
	l2 |= UCB0RXBUF<<8;
	UCB0CTLW0 |= UCTXSTP;

	//send_int(l, 10);
	//new_line();
	//send_int(l2, 10);
	//new_line();
	l = l | (l2<<8);
	return l;*/
}

int getHumidity() {
	UCB0I2CSA = HUMIDITY_SENSOR_ADDRESS;
	i2c_writeByte(CMD_GET_HUMIDITY);
	unsigned int rh = readData();
	rh &= ~0b11;  //because last 2 bits are status bits, not data
	rh = 100*(-6 + 125 * ((double)rh/65536.0));
	return rh;
}

int getPressure() {
	UCB0I2CSA = PRESSURE_SENSOR_ADDRESS;
	//i2c_writeByte(CMD_GET_PRESSURE);
	i2c_writeByte(0xF4);
	i2c_writeByte(0x34 + (1 << 6));
	__delay_cycles(64000);
	i2c_writeByte(CMD_READ_PRESSURE_MSB);
	unsigned int msb = readData();
	i2c_writeByte(CMD_READ_PRESSURE_LSB);
	unsigned int lsb = readData();
	i2c_writeByte(0xF8);
	unsigned int xlsb = readData();
	return (msb << 16 + lsb << 8 + xlsb) >> (7);
}

void update(){
	unsigned int rh = (int)getHumidity();
	int temp = (int)getTemperature();
	//int pressure = (int)getPressure();
	//unsigned int light = (int)getLight();
	int intTemp = temp / 100;
	int intRh = rh / 100;
	if(displayState == 0){
		switch(intTemp) {
			case -20 ... 0:
				setRGB(0, (intTemp + 20)*12.75, 255);
				break;
			case 1 ... 15:
				setRGB(intTemp * 17, 255, 255);
				break;
			case 16 ... 25:
				setRGB(255, 255, 255 - (intTemp - 15) * 25.5);
				break;
			case 26 ... 40:
				setRGB(255, 255 - (intTemp - 25) * 17, 0);
				break;
		}
	}
	else if(displayState == 1){
		switch(intRh) {
			case 0 ... 50:
				setRGB(255, intRh * 5.1, 0);
				break;
			case 51 ... 100:
				setRGB(255 - (intRh - 50) * 5.1, 255, 0);
		}
	}
	else if(displayState == 2) {
		setRGB(255, 255, 255);
	}

	/*send_str("temp: ");
	send_int(temp/100, 10);
	send_str(".");
	send_int(temp%100, 10);
	send_str(" Celsius");
	new_line();
	send_str("humidity: ");
	send_int(rh/100, 10);
	send_char('.');
	send_int(rh%100, 10);
	send_char('%');
	new_line();
	send_str("light: ");
	send_int(light, 10);
	new_line();
	//send_str("Pressusre: ");
	//send_int(pressure,10);
	//new_line();*/
	//csv data format for plotting
	send_int(temp/100, 10);
	send_char('.');
	send_int(temp%100, 10);
	send_char(',');
	send_int(rh/100, 10);
	send_char('.');
	send_int(rh%100, 10);
	/*send_char(',');
	send_int(light, 10);
	send_char(',');
	send_int(pressure, 10);*/
	new_line();
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;                   // Disable the GPIO power-on default high-impedance mode


    init_gpio();							// Initialize GPIO

                                              // to activate previously configured port settings
    init_clock();							// Initialize clocks
    init_UART();							// Initialize serial port
    i2c_init();								// Init I2C for communicating with the sensor board
    init_timer();							// init timer for PWM control of the RGB LED
    init_RTC();
    //int i = 0;
    /*for (i = 0; i < 256; i ++){
       	setRGB(i, i, i);
       	__delay_cycles(400000);
    }*/
    __enable_interrupt();
    send_str("INITIALIZED");
    new_line();

    for ( ;; ) {

    	__bis_SR_register(LPM3_bits | GIE);             // Enter LPM3, interrupts enabled
    	 __no_operation();                         // For debugger
    }
}
