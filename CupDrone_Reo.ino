/*
	CupDrone_Reo

	Made by Baram ( chcbaram@paran.com )
*/

#include "LED.h"
#include "IMU.h"
#include "MSP.h"
#include "PWM.h"


cLED 		LED;
cIMU		IMU;
cMSP		MSP;
cPWM		PWM;


int16_t target_roll;
int16_t target_pitch;
int16_t target_yaw;
int16_t target_throtle;




/*---------------------------------------------------------------------------
     TITLE   : setup
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void setup() 
{
	LED.begin();
	MSP.begin();
	IMU.begin();
	PWM.begin();
}





/*---------------------------------------------------------------------------
     TITLE   : loop
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void loop() 
{
	static uint32_t tLed;
	static uint32_t tMsp;


	//-- LED
	//
	if( (millis()-tLed) >= 500 )
	{
		tLed = millis();
		LED.toggle();
	}


	if( MSP.update() == true )
	{
		tMsp = millis();
		target_roll    = MSP.Get_Roll();
		target_pitch   = MSP.Get_Pitch();
		target_yaw     = MSP.Get_Yaw();
		target_throtle = MSP.Get_Throtle();
	}


	if( MSP.Get_ArmMode() == false )
	{
		target_roll    = 0;
		target_pitch   = 0;
		target_yaw     = 0;
		target_throtle = 0;
	}


	//-- FailSafe 기능 
	//
	if( (millis()-tMsp) > 1000 )
	{

	}


	//-- 센서값 업데이트
	//
	IMU.update();



	//-- 모터값 업데이트 
	//
	PWM.update();


	loop_menu();
}




#define GYRO_I_MAX 	256
#define ACC_I_MAX 	256


void pid_control( int16_t roll, int16_t pitch, int16_t yaw, int16_t throtle )
{
	int16_t target;



	//----------PID controller----------
	//
	for( axis=0; axis<3; axis++ ) 
	{
  		switch( axis )
  		{
  			case ROLL:
				target       = constrain( roll, -500, +500 );
	      		errorAngle   = target - IMU.angle[axis]; //16 bits is ok here
		        AngleRateTmp = ((int32_t) errorAngle * conf.pid[PIDLEVEL].P8)>>4;
  				break;

  			case PITCH:
				target     = constrain( pitch, -500, +500 );
	      		errorAngle = target - IMU.angle[axis]; //16 bits is ok here  			
  				break;

  			case YAW:
  				AngleRateTmp = (((int32_t) (27) * yaw) >> 5);
  				break;
  		}



	    //--------low-level gyro-based PID. ----------
	    //Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
	    //-----calculate scaled error.AngleRates
	    //multiplication of rcCommand corresponds to changing the sticks scaling here
	    RateError = AngleRateTmp  - IMU.SEN.gyroData[axis];

	    //-----calculate P component
	    PTerm = ((int32_t) RateError * conf.pid[axis].P8)>>7;

	    //-----calculate I component
	    //there should be no division before accumulating the error to integrator, because the precision would be reduced.
	    //Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
	    //Time correction (to avoid different I scaling for different builds based on average cycle time)
	    //is normalized to cycle time = 2048.
	    errorGyroI[axis]  += (((int32_t) RateError * cycleTime)>>11) * conf.pid[axis].I8;
	    //limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
	    //I coefficient (I8) moved before integration to make limiting independent from PID settings
	    errorGyroI[axis]  = constrain(errorGyroI[axis], (int32_t) -GYRO_I_MAX<<13, (int32_t) +GYRO_I_MAX<<13);
	    ITerm = errorGyroI[axis]>>13;

	    //-----calculate D-term
	    delta          = RateError - lastError[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
	    lastError[axis] = RateError;

	    //Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
	    // would be scaled by different dt each time. Division by dT fixes that.
	    delta = ((int32_t) delta * ((uint16_t)0xFFFF / (cycleTime>>4)))>>6;
	    //add moving average here to reduce noise
	    deltaSum       = delta1[axis]+delta2[axis]+delta;
	    delta2[axis]   = delta1[axis];
	    delta1[axis]   = delta;

	    //DTerm = (deltaSum*conf.pid[axis].D8)>>8;
	    //Solve overflow in calculation above...
	    DTerm = ((int32_t)deltaSum*conf.pid[axis].D8)>>8;
	    //-----calculate total PID output
	    axisPID[axis] =  PTerm + ITerm + DTerm;
	}

}




void loop_menu( void )
{
	static uint32_t tTime;



	if( Serial.available() )
	{
		char Ch = Serial.read();

		if( Ch == '1' )
		{
			PWM.set_all(100);	
			Serial.println("in");
		}
		else if( Ch == 's')
		{
			PWM.set_all(0);
		}
		else if( Ch == '2' )
		{
			while(1)
			{
				IMU.update();

				if( (millis()-tTime) >= 100 )
				{
					tTime = millis();
					Serial.print(0);
					Serial.print(" ");
					Serial.print(IMU.angle[0]/10);
					Serial.print(" ");
					Serial.print(IMU.angle[1]/10);
					Serial.print(" ");
					Serial.println(IMU.angle[2]);
				}			

				if( Serial.available() )
				{
					if( Serial.read() == 'x' ) break;
				}
			}
		}
	}
}
