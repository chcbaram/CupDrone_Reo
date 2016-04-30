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
//    파일명     	: PWM.h
//----------------------------------------------------------------------------
#ifndef _PWM_H_
#define _PWM_H_

#include <inttypes.h>
#include <Arduino.h> 




#define PWM_MAX_CH		4


#define REAR_R			0
#define FRONT_R 		1
#define REAR_L			2
#define FRONT_L 		3




class cPWM
{

public:


public:
	cPWM();

	void begin( void );
	void update( void );

	void set_all( uint16_t Pwm );	
	void set_out( uint8_t motor_ch, uint16_t Pwm );

private:
	uint16_t pwm_out[PWM_MAX_CH];
};


#endif
