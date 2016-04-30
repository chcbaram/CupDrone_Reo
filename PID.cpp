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
	int16_t PTermACC = 0;
	int16_t ITermACC = 0;
	int16_t PTermGYRO = 0;
	int16_t ITermGYRO = 0;

	int16_t deltaSum;



	if( mode == PID_ANGLE )
	{
		
		//-- P 제어기값 계산
		//
		err_angle = constrain( (target_angle<<1), -500, +500 ) - current_angle;		//16 bits is ok here
		PTermACC  = ((int32_t)err_angle * Gain_Angle.P8)>>7;                     	// 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
		PTermACC  = constrain( PTermACC,-Gain_Angle.D8*5, +Gain_Angle.D8*5 );

		//-- I 제어기값 계산
		//
		err_angle_sum = constrain(err_angle_sum + err_angle, -10000, +10000);		// WindUp     //16 bits is ok here
		ITermACC      = ((int32_t)err_angle_sum * Gain_Angle.I8)>>12;				// 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result


        PTerm = PTermACC;
        ITerm = ITermACC;

	}
	else
	{

		//-- P 제어기값 계산 
		//
		if(abs(target_angle)<500) 
			err_rate = (target_angle<<6)/Gain_Rate.P8; 			// 16 bits is needed for calculation: 500*64 = 32000      16 bits is ok for result if P8>5 (P>0.5)
		else 
			err_rate = ((int32_t)target_angle<<6)/Gain_Rate.P8;	// 32 bits is needed for calculation

		err_rate -= current_velocity;
		PTermGYRO = target_angle;
      
		//-- I 제어기값 계산
		//
		err_rate_sum = constrain( err_rate_sum + err_rate,-16000,+16000);	// WindUp   16 bits is ok here		
		if ( abs(current_velocity) > 640 ) err_rate_sum = 0;		
		ITermGYRO = ((err_rate_sum>>7) * Gain_Rate.I8)>>6;			// 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000


        PTerm = PTermGYRO;
        ITerm = ITermGYRO;	
	}    


	PTerm -= ((int32_t)current_velocity * Gain_Rate.P8)>>6; 


	//-- D 제어기 
	//
	err_rate_delta[0] = current_velocity - err_rate_last;  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
	err_rate_last     = current_velocity;


	//add moving average here to reduce noise
	deltaSum     = err_rate_delta[2] + err_rate_delta[1] + err_rate_delta[0];
    
    err_rate_delta[2]   = err_rate_delta[1];
    err_rate_delta[1]   = err_rate_delta[0];


    DTerm = ((int32_t)deltaSum * Gain_Rate.D8)>>5;        // 32 bits is needed for calculation
                      


    // PID 제어 결과 저장 
    //
	pid_out =  PTerm + ITerm - DTerm;


	return pid_out;
}
