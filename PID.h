//----------------------------------------------------------------------------
//    프로그램명 	: PID
//
//    만든이     	: Made by Baram ( chcbaram@paran.com )
//
//    날  짜     : 
//    
//    최종 수정  	: 
//
//    MPU_Type	: 
//
//    파일명     	: PID.h
//----------------------------------------------------------------------------
#ifndef _PID_H_
#define _PID_H_

#include <inttypes.h>
#include <Arduino.h> 



#define PID_ANGLE	0
#define PID_RATE	1




typedef struct
{
  uint8_t P8;
  uint8_t I8;
  uint8_t D8;
} _PID;




class cPID
{

public:


public:
	cPID();

	void set_gain_angle( uint8_t p_gain, uint8_t i_gain, uint8_t d_gain );
	void set_gain_rate( uint8_t p_gain, uint8_t i_gain, uint8_t d_gain );

	void reset();
	int16_t update( uint8_t mode, int16_t target_angle, int16_t current_angle, int16_t current_velocity, uint16_t dt );

private:
	int32_t err_angle_sum;

	int16_t err_rate_delta[3];
	int16_t err_rate_last;
	int32_t err_rate_sum;


	_PID Gain_Angle;
	_PID Gain_Rate;
};


#endif