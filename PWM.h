//----------------------------------------------------------------------------
//    프로그램명 	: PWM
//
//    만든이     	: Cho Han Cheol 
//
//    날  짜     : 
//    
//    최종 수정  	: 
//
//    MPU_Type	: 
//
//    파일명     	: LED.h
//----------------------------------------------------------------------------
#ifndef _PWM_H_
#define _PWM_H_

#include <inttypes.h>
#include <Arduino.h> 




#define PWM_MAX_CH		4





class cPWM
{

public:


public:
	cPWM();

	void begin( void );
	void update( void );

	void set_all( uint16_t Pwm );	
	void set_LT( uint16_t Pwm );
	void set_LB( uint16_t Pwm );
	void set_RT( uint16_t Pwm );
	void set_RB( uint16_t Pwm );

private:
	uint16_t pwm_out[PWM_MAX_CH];
};


#endif
