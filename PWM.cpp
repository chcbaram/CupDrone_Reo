//----------------------------------------------------------------------------
//    프로그램명 	: PWM
//
//    만든이     	: Baram ( chcbaram@paran.com )
//
//    날  짜     : 
//    
//    최종 수정  	: 
//
//    MPU_Type	: 
//
//    파일명     	: PWM.cpp
//----------------------------------------------------------------------------




#include <Arduino.h> 
#include "PWM.h"



uint8_t PWM_PIN[PWM_MAX_CH] = { 9, 10, 5, 6 };




/*---------------------------------------------------------------------------
     TITLE   : PWM
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
cPWM::cPWM()
{	    
}




/*---------------------------------------------------------------------------
     TITLE   : begin
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cPWM::begin( void )
{
	int8_t i;
    

	for( i=0; i<4; i++ )
    {
		pinMode( PWM_PIN[i], OUTPUT );
		digitalWrite( PWM_PIN[i], LOW );
		pwm_out[i] = 0;
    }
    

	TCCR1A = (1<<WGM11);
	TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS10);
	TCCR3A = (1<<WGM31);
	TCCR3B = (1<<WGM33) | (1<<WGM32) | (1<<CS30);

	
	// EXT_MOTOR_32KHZ

	ICR1   = 0x01FF; // TOP to 511;
	ICR3   = 0x01FF; // TOP to 511;
	TC4H = 0x01;
	OCR4C = 0xFF; // phase and frequency correct mode & top to 511
	TCCR4B = (1<<CS40);             // prescaler to 1

	TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
	TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
	TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A
	TCCR4D = 0;
	TCCR4C |= (1<<COM4D1)|(1<<PWM4D); // connect pin 6 to timer 4 channel D

	set_all(0);
}





/*---------------------------------------------------------------------------
     TITLE   : update
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cPWM::update( void )
{
	uint16_t Temp2;


	OCR1A = (pwm_out[0]) >> 1; // max = 511
	OCR1B = (pwm_out[1]) >> 1;
	OCR3A = (pwm_out[2]) >> 1;

	Temp2 = pwm_out[3];
	Temp2 = Temp2 >> 1;
	TC4H  = Temp2 >> 8;
	OCR4D = Temp2 & 0xFF; 
}





/*---------------------------------------------------------------------------
     TITLE   : set_all
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cPWM::set_all( uint16_t Pwm )
{
	set_out( 0, Pwm );
	set_out( 1, Pwm );
	set_out( 2, Pwm );
	set_out( 3, Pwm );	
}





/*---------------------------------------------------------------------------
     TITLE   : set_out
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cPWM::set_out( uint8_t motor_ch, uint16_t Pwm )
{
	if( motor_ch < PWM_MAX_CH )
	{
		Pwm = constrain( Pwm, 0, 1000 );
		pwm_out[motor_ch] = Pwm;
	}
}


