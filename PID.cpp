//----------------------------------------------------------------------------
//    프로그램명 	: PID
//
//    만든이   	: Baram ( chcbaram@paran.com )
//
//    날  짜     : 
//    
//    최종 수정  	: 
//
//    MPU_Type	: 
//
//    파일명    	: PID.cpp
//----------------------------------------------------------------------------
/*
	Based on Multiwii : https://github.com/multiwii/multiwii-firmware
*/



#include <Arduino.h> 
#include "PID.h"






#define GYRO_I_MAX 	256
#define ACC_I_MAX 	256







/*---------------------------------------------------------------------------
     TITLE   : PID
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
cPID::cPID()
{	
	err_rate_delta[0] = 0;
	err_rate_delta[1] = 0;
	err_rate_delta[2] = 0;

	err_rate_last = 0;
	err_rate_sum  = 0;	

	err_angle_sum = 0;
}





/*---------------------------------------------------------------------------
     TITLE   : set_gain_angle
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cPID::set_gain_angle( uint8_t p_gain, uint8_t i_gain, uint8_t d_gain )
{
     Gain_Angle.P8 = p_gain;
     Gain_Angle.I8 = i_gain;
     Gain_Angle.D8 = d_gain;
}




/*---------------------------------------------------------------------------
     TITLE   : set_gain_rate
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cPID::set_gain_rate( uint8_t p_gain, uint8_t i_gain, uint8_t d_gain )
{
	Gain_Rate.P8 = p_gain;
	Gain_Rate.I8 = i_gain;
	Gain_Rate.D8 = d_gain;
}





/*---------------------------------------------------------------------------
     TITLE   : reset
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cPID::reset( void )
{
	err_rate_sum  = 0;	
	err_angle_sum = 0;
}





/*---------------------------------------------------------------------------
     TITLE   : update
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
int16_t cPID::update( uint8_t mode, int16_t target_angle, int16_t current_angle, int16_t current_velocity, uint16_t dt )
{
	int16_t pid_out = 0;
	int16_t err_angle;
	int16_t err_rate;
	int16_t angle_rate;
	int16_t PTerm = 0;
	int16_t ITerm = 0;
	int16_t DTerm = 0;
	int16_t deltaSum;


	if( mode == PID_ANGLE )
	{
		// 각도 에러 구한다. deg
		//
		target_angle = constrain( target_angle, -500, +500 );
		err_angle    = target_angle - current_angle; 
		
		PTerm = ((int32_t) err_angle * Gain_Angle.P8)>>4;

		err_angle_sum = constrain(err_angle_sum + err_angle,-10000,+10000);
		ITerm         = ((int32_t)err_angle_sum * Gain_Angle.I8)>>12; 

     	angle_rate   = PTerm + ITerm;
	}
	else
	{
		angle_rate   = (((int32_t) (27) * target_angle) >> 5);
	}    


	// 각속도 에러 구한다. deg/s
	//
	err_rate = angle_rate - current_velocity;


	// P 제어기값 계산
	//
	PTerm = ((int32_t) err_rate * Gain_Rate.P8)>>7;


	// I 제어기값 계산
	//
    err_rate_sum += (((int32_t) err_rate * dt)>>11) * Gain_Rate.I8;
    err_rate_sum  = constrain(err_rate_sum, (int32_t) -GYRO_I_MAX<<13, (int32_t) +GYRO_I_MAX<<13);

	ITerm = err_rate_sum>>13;


	// D 제어기값 계산
	//
	err_rate_delta[0] = err_rate - err_rate_last;  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
	err_rate_last     = err_rate;

    //Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
    // would be scaled by different dt each time. Division by dT fixes that.
	err_rate_delta[0] = ((int32_t)err_rate_delta[0] * ((uint16_t)0xFFFF / (dt>>4)))>>6;

	//add moving average here to reduce noise
	deltaSum     = err_rate_delta[2] + err_rate_delta[1] + err_rate_delta[0];
    
    err_rate_delta[2]   = err_rate_delta[1];
    err_rate_delta[1]   = err_rate_delta[0];

    
   	//Solve overflow in calculation above...
    DTerm = ((int32_t)deltaSum * Gain_Rate.D8)>>8;
    

	pid_out =  PTerm + ITerm + DTerm;

#if 0
	static uint32_t tTime;

	if( millis()-tTime > 100 )
	{
		tTime = millis();
		Serial.print(PTerm);
		Serial.print(" ");
		Serial.print(ITerm);
		Serial.print(" ");
		Serial.print(DTerm);
		Serial.print(" ");		
		Serial.print(target_angle);
		Serial.print(" ");
		Serial.print(current_angle);
		Serial.print(" ");	
		Serial.println(err_rate);
	}
#endif
	return pid_out;
}




