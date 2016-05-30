// ---------------------------------------------------------------------------------
// Objective:
//	Read the temperature from the sensor and send it via UART everytime the 'enter' button
//	is pressed
//
// ------------------------------------------------------------------------------------

//#include "driverlib.h"
#include "../msp430_driverlib_2_60_00_02/driverlib/MSP430FR5xx_6xx/driverlib.h"
#include "stdint.h"


// ------------------------------------------------------------------------------
// UART interface
// -------------------------------------------------------------------------------
#define UART_TX_READY		(UCA0IFG & UCTXIFG)
#define UART_RX_READY		(UCA0IFG & UCRXIFG)
#define UART_TX_DONE		(UCA0IFG & UCTXCPTIFG)
#define UART_RESET_TX_DONE	(UCA0IFG &= ~UCTXCPTIFG)

//Function Prototypes
static void send_char(char);
static void send_str(char*);
static void send_int(int, int);
static void new_line();

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

void i2c_init(void)
{
	  // Configure USCI_B0 for I2C mode

	  P1SEL1 |= BIT6 | BIT7;                    // I2C pins

	  UCB0CTLW0 |= UCSWRST;                     // Software reset enabled
	  UCB0CTLW0 |= UCMODE_3 | UCMST | UCSYNC;   // I2C mode, Master mode, sync

	  UCB0BRW = 24;                         // baudrate = SMCLK / 8
	  //UCB0I2CSA = TEMP_SENSOR_ADDRESS;               // Slave address
	  UCB0TBCNT = 0x0003;                       // number of bytes to be received	//TODO
	  UCB0CTL1 &= ~UCSWRST;

	  UCB0IE |= UCNACKIE;             // transmit and NACK interrupt enable

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
	t = 100*(-46.85+175.72*((double)t/65536.0));
	return t;
}

unsigned int getLight(){
	send_str("LIGHT START: ");
	UCB0I2CSA = LIGHT_SENSOR_ADDRESS;

	i2c_writeByte(CMD_GET_LIGHT_LSB);
	unsigned int l = 0;
	UCB0CTLW0 &= ~UCTR;			// Receive mode
	UCB0CTL1 |= UCTXSTT;  // I2C start condition
	while(!(UCB0IFG & UCRXIFG));
	l |= UCB0RXBUF;				//read low byte
	while(!(UCB0IFG & UCRXIFG));
		l |= UCB0RXBUF<<8;
	UCB0CTLW0 |= UCTXSTP;

	unsigned int l2 = 0;
	i2c_writeByte(CMD_GET_LIGHT_MSB);
	UCB0CTLW0 &= ~UCTR;			// Receive mode
	UCB0CTL1 |= UCTXSTT;  // I2C start condition
	while(!(UCB0IFG & UCRXIFG));
	l2 |= UCB0RXBUF;				//read low byte
	while(!(UCB0IFG & UCRXIFG));
		l2 |= UCB0RXBUF<<8;
	UCB0CTLW0 |= UCTXSTP;

	send_int(l, 10);
	new_line();
	send_int(l2, 10);
	new_line();
	l = l | (l2<<8);
	return l;
}

int getHumidity() {
	UCB0I2CSA = HUMIDITY_SENSOR_ADDRESS;
	i2c_writeByte(CMD_GET_HUMIDITY);
	unsigned int rh = readData();
	rh &= ~0b11;  //because last 2 bits are status bits, not data
	rh = -6 + 125 * ((double)rh/65536.0);
	return rh;
}

int getPressure() {
	UCB0I2CSA = PRESSURE_SENSOR_ADDRESS;
	i2c_writeByte(CMD_GET_PRESSURE);
	__delay_cycles(64000);
	i2c_writeByte(CMD_READ_PRESSURE_MSB);
	unsigned int msb = readData();
	i2c_writeByte(CMD_READ_PRESSURE_LSB);
	unsigned int lsb = readData();
	return lsb | (msb << 8);
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


    __enable_interrupt();
    send_str("INITIALIZED");
    new_line();

    for ( ;; ) {

    	//LPM0;
    	unsigned int rh = (int)getHumidity();
    	int temp = (int)getTemperature();
    	int pressure = (int)getPressure();
    	unsigned int light = (int)getLight();
    	send_str("temp: ");
    	send_int(temp/100, 10);
    	send_str(".");
    	send_int(temp%100, 10);
    	send_str(" Celsius");
    	new_line();
    	send_str("humidity: ");
    	send_int(rh, 10);
    	new_line();
    	send_str("light: ");
    	send_int(light, 10);
    	new_line();
    	send_str("Pressure: ");
    	send_int(pressure, 10);
    	new_line();

    	//csv data format for plotting
    	/*send_int(temp/100, 10);
    	send_char('.');
    	send_int(temp%100, 10);
    	send_char(',');
    	send_int(rh, 10);
    	send_char(',');
    	send_int(light, 10);
    	send_char(',');
    	send_int(pressure, 10);
    	new_line();*/

    	   __delay_cycles(4000000);
    }
}
