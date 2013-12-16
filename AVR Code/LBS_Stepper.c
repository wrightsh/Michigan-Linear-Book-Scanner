//Shawn Wright
//University of Michigan
//Linear Book Scanner
//Stepper Controller
//December 12, 2013

/*
 * The purpose of this program is to control a stepper motor
 * for the Linear Book Scanner.  Essentially we want a real 
 * time opperating system commanding the steps so that we 
 * know that the velocity is consistent.  
 *
 * Using SPI you can set a position (in steps) and a % of max
 * velocity for the stepper motor to go to.  Pull the enable
 * pin low, send the send position flag (currently 240 but can
 * be changed to any 8 bit number) then send two bits of position
 * data, followed by one bit of % max velocity data.  When the enable
 * pin is pulled high the microcontroller will make the motor move
 * to that position at that speed.  You can also set what position
 * it currently is at, this is useful for setting the home position of
 * the Book Scanner.  Pull the enable pin low, send 241 followed by two
 * bytes of position data (16 bit signed integer).  When the enable pin
 * is made high the controller will attempt to go to its set position.
 *
 * There may be a bug in this code in that the micro seems to need to be
 * reset after the raspberry pi is booted but I'm not sure what would
 * be causing that at the moment, to do this briefly pull the reset pin
 * low.
 */

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>


#define ENABLE (PINB & _BV(PB0))

#define STEPS_ON (PORTC |= _BV(PC4))
#define STEPS_OFF (PORTC &= ~_BV(PC4))
#define STEPS (PINC & _BV(PC4))

#define DIRECTION_ON (PORTC |= _BV(PC3))
#define DIRECTION_OFF (PORTC &= ~_BV(PC3))

#define MOTOR_ENABLE_ON (PORTC |= _BV(PC2))
#define MOTOR_ENABLE_OFF (PORTC &= ~_BV(PC2))


volatile uint8_t MOVING = 0; //flag set to indicate motion
volatile int8_t DIRECTION = 0; //1 or -1 depending on movement direction
volatile int16_t CURRENT_POSITION = 0; //Current absolute position in microsteps

volatile char PERCENT_VELOCITY = 0;
volatile int16_t  SET_POSITION = 0; //Desired absolute position in microsteps


volatile double MAX_RPM = 500; //steps per second
volatile double MAX_SPEED_BY_TIMESTEP = 0;

volatile uint8_t SPI_POSITION = 0; //used to tell what to send on the spi next

volatile char TEMP1 = 0;
volatile char TEMP2 = 0;
volatile char TEMP3 = 0;
volatile char TEMP4 = 0;

void initialize(void)
{

	TCCR1B |= _BV(WGM12);
	TCNT1 = 0; //set timer to zero
	TIMSK1 |= _BV(OCIE1A); //enable OCR1A interrupt

	DDRC |= _BV(PC4); //set PC4 output for Setps
	DDRC |= _BV(PC3); //set PC3 output for direction
	DDRC |= _BV(PC2); //motor enable
	
	MAX_SPEED_BY_TIMESTEP = 300000/MAX_RPM ;// 8000000*60/(200*16*MAX_RPM);
	
	
	//spi
	// Set MISO as output
	DDRB |= _BV(PB4) |_BV(PB5); //MOSI output
	/* Enable SPI, slave, set clock rate fck/16 */

	SPCR |= _BV(SPE);
	SPCR |= _BV(SPIE);
	
	//ENABLE PIN CHANGE interrupt
	DDRB &= ~_BV(PB0);     // Clear the PB0 pin
	// PB0 (PCINT0 pin) is now an input
	PORTB |= _BV(PB0);    // turn On the Pull-up
	// PB0 is now an input with pull-up enabled

	PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
	PCMSK0 |= (1 << PCINT0);  // set PCINT0 to trigger an interrupt on state change
	
	sei();
}


void CW(void)
{
	DIRECTION_ON;
	DIRECTION = 1; //added to position
}


void CCW(void)
{
	DIRECTION_OFF;
	DIRECTION = -1; //added to position
}


int position()
{
	cli();
	
	OCR1A = (unsigned)(MAX_SPEED_BY_TIMESTEP*100/(double)PERCENT_VELOCITY);

	if(CURRENT_POSITION < SET_POSITION){
		CW();
		} else if(CURRENT_POSITION > SET_POSITION){
		CCW();
		} else if(CURRENT_POSITION == SET_POSITION){
		MOVING = 0;
		return 2;
	}


	TCNT1 = 0;
	MOVING = 1;  //moving warning flag set
	
	TCCR1B |= _BV(CS10); //turn clk on 1:1
	TIMSK1 |= _BV(OCIE1A); //enable OCR1A interrupt
	MOTOR_ENABLE_ON;
	sei(); //enable global interrupts

	return 1;
}


void stop_motor(void)
{
	TIMSK1 &= ~_BV(OCIE1A); //enable OCR1A interrupt
	TCCR1B &= ~_BV(CS10); //shut down clock
	MOTOR_ENABLE_OFF;
	MOVING = 0;
}


ISR(TIMER1_COMPA_vect)
{
	
	if(ENABLE){
		if(STEPS){
			STEPS_OFF;
			} else{
			STEPS_ON;
			CURRENT_POSITION += DIRECTION; //update position on rising edge

			if(CURRENT_POSITION == SET_POSITION){
				stop_motor();
			}
		}
	}

}


//pin change interrupt flag for PCINT0 (PB0)
//https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328
ISR (PCINT0_vect)
{
	SPI_POSITION = 0;
	TEMP1 = 0;
	TEMP2 = 0;
	TEMP3 = 0;
	TEMP4 = 0;
	
	if(ENABLE){
		position();
		} else {
		stop_motor();
	}
}


ISR(SPI_STC_vect)
{

	if(ENABLE && (SPI_POSITION == 0) ){
		SPDR = (char)(CURRENT_POSITION>>8);
		SPI_POSITION++;
	} else if(ENABLE && (SPI_POSITION == 1) ){
		SPDR = (char)CURRENT_POSITION;
		SPI_POSITION++;
	} else if(ENABLE && (SPI_POSITION == 2) ){
		SPDR = MOVING;
		SPI_POSITION++;
		SPI_POSITION = 0;
		return;
	}
	

	if(!ENABLE){
		
		if( SPI_POSITION == 0){
			TEMP1 = SPDR;	
			SPDR = 1; //for diagnostics on the rpi
			SPI_POSITION++;
		} else if( SPI_POSITION == 1){
			TEMP2 = SPDR;
			SPDR = 2; //for diagnostics on the rpi
			SPI_POSITION++;
		} else if( SPI_POSITION == 2){
			TEMP3 = SPDR;
			SPDR = 3; //for diagnostics on the rpi
			SPI_POSITION++;
		} else if ( SPI_POSITION == 3){
			TEMP4 = SPDR;
			SPDR = 4; //for diagnostics on the rpi
			SPI_POSITION++;
		}
		
		// Used to change what the set position is, 240 is just a flag
		if( (SPI_POSITION == 4) && (TEMP1 == 240)){
			SPDR = 5; //for diagnostics
			SET_POSITION = ((uint16_t)TEMP2)*256;
			SET_POSITION = SET_POSITION + TEMP3;
			PERCENT_VELOCITY = TEMP4;
		}
		
		// Used to change what the current position is, 241 is just a flag
		if( (SPI_POSITION == 4) && (TEMP1 == 241)){
			SPDR = 6; //for diagnostics
			CURRENT_POSITION = ((uint16_t)TEMP2)*256;
			CURRENT_POSITION = CURRENT_POSITION + TEMP3;
			SET_POSITION = CURRENT_POSITION;
		}
		
	}
		

}


int main(void)
{
	initialize();  //sets up ports
	while(1){}

}

