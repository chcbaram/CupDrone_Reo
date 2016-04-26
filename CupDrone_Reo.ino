/*
	CupDrone_Reo

	Made by Baram ( chcbaram@paran.com )
*/

#include "LED.h"
#include "IMU.h"
#include "MSP.h"
#include "PWM.h"
#include "PID.h"




cLED 		LED;
cIMU		IMU;
cMSP		MSP;
cPWM		PWM;
cPID		PID[3];





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


	PID[ROLL ].set_gain_angle(30, 0, 0);
	PID[PITCH].set_gain_angle(30, 0, 0);
	PID[YAW  ].set_gain_angle(30, 0, 0);


	PID[ROLL ].set_gain_rate(28, 10, 7);
	PID[PITCH].set_gain_rate(28, 10, 7);
	PID[YAW  ].set_gain_rate(68, 45, 0);
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

	uint16_t tIMU;

	int16_t pid_out[3];

	int16_t motor_pwm[4];
	int16_t max_pwm;
	int16_t i;



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
	tIMU = IMU.update();

	if( tIMU > 0 )
	{
		pid_out[ROLL ] = PID[ROLL ].update( PID_ANGLE, target_roll , IMU.angle[ROLL ], -IMU.gyroData[ROLL ], 5000 );
		pid_out[PITCH] = PID[PITCH].update( PID_ANGLE, target_pitch, IMU.angle[PITCH], IMU.gyroData[PITCH], 5000 );
		pid_out[YAW  ] = PID[YAW  ].update( PID_RATE , target_yaw  , IMU.angle[YAW  ], IMU.gyroData[YAW  ], 5000 );


  		#define PIDMIX(X,Y,Z)  target_throtle + pid_out[ROLL]*X + pid_out[PITCH]*Y + 1 * pid_out[YAW]*Z


		motor_pwm[REAR_R ] = PIDMIX(-1,+1,-1); //REAR_R
		motor_pwm[FRONT_R] = PIDMIX(-1,-1,+1); //FRONT_R
		motor_pwm[REAR_L ] = PIDMIX(+1,+1,+1); //REAR_L
		motor_pwm[FRONT_L] = PIDMIX(+1,-1,-1); //FRONT_L
/*
		motor_pwm[REAR_R ] = PIDMIX(-1,+1,+1); //REAR_R
		motor_pwm[FRONT_R] = PIDMIX(-1,+1,-1); //FRONT_R
		motor_pwm[REAR_L ] = PIDMIX(+1,+1,-1); //REAR_L
		motor_pwm[FRONT_L] = PIDMIX(+1,-1,-1); //FRONT_L
*/


		max_pwm = motor_pwm[0];

		for( i=1; i<4; i++)
		{
			if(motor_pwm[i] > max_pwm) max_pwm = motor_pwm[i];		
		}
      
		for( i=0; i<4; i++) 
		{
			if(max_pwm > 1000) // this is a way to still have good gyro corrections if at least one motor reaches its max.
			{
        		motor_pwm[i] -= max_pwm - 1000;
        	}

			motor_pwm[i] = constrain(motor_pwm[i], 0, 1000);

      		if( target_throtle < 50 )
      		{
        		motor_pwm[i] = 0;
        	}

/*
        	PWM.set_LT( motor_pwm[FRONT_L] );
			PWM.set_LB( motor_pwm[REAR_L] );
			PWM.set_RT( motor_pwm[FRONT_R] );
			PWM.set_RB( motor_pwm[REAR_R] );
*/
      		//if (!f.ARMED)
      		//{
        	//	motor[i] = MINCOMMAND;
        	//}
		}
//*/


		Serial.print(pid_out[ROLL]);
		Serial.print(" ");
		Serial.print(pid_out[PITCH]);
		Serial.print(" ");
		Serial.print(pid_out[YAW]);
		Serial.print(" ");


		Serial.print(motor_pwm[REAR_R]);
		Serial.print(" ");
		Serial.print(motor_pwm[FRONT_R]);		
		Serial.print(" ");
		Serial.print(motor_pwm[REAR_L]);
		Serial.print(" ");
		Serial.println(motor_pwm[FRONT_L]);


	}



	//-- 모터값 업데이트 
	//
	PWM.update();


	loop_menu();
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
		else if( Ch == '3' )
		{
			while(1)
			{
				IMU.update();

				if( (millis()-tTime) >= 100 )
				{
					tTime = millis();
					Serial.print(0);
					Serial.print(" ");
					Serial.print(IMU.gyroData[0]);
					Serial.print(" ");
					Serial.print(IMU.gyroData[1]);
					Serial.print(" ");
					Serial.println(IMU.gyroData[2]);
				}			

				if( Serial.available() )
				{
					if( Serial.read() == 'x' ) break;
				}
			}
		}


	}
}
