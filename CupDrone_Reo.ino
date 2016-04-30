/*
	CupDrone_Reo

	Made by Baram ( chcbaram@paran.com )
*/

#include "LED.h"
#include "IMU.h"
#include "MSP.h"
#include "PWM.h"
#include "PID.h"




//-- 옵션 기능 활성화
//
#define _USE_FAIL_SAVE		0


//-- 모터 출력 분배 
//
#define PIDMIX(X,Y,Z)  target_throtle + pid_out[ROLL]*X + pid_out[PITCH]*Y + 1 * pid_out[YAW]*Z


//-- 객체 생성
//
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


/*
	//-- 기존 제어 게인값 
	//
	PID[ROLL ].set_gain_angle(30, 0, 0);
	PID[PITCH].set_gain_angle(30, 0, 0);
	PID[YAW  ].set_gain_angle(30, 0, 0);

	PID[ROLL ].set_gain_rate(28, 10, 7);
	PID[PITCH].set_gain_rate(28, 10, 7);
	PID[YAW  ].set_gain_rate(68, 45, 0);
*/

	//-- 각도 제어기 게인값 설정(P, I, D)
	//
	PID[ROLL ].set_gain_angle(30, 0, 0);
	PID[PITCH].set_gain_angle(30, 0, 0);
	PID[YAW  ].set_gain_angle(30, 0, 0);


	//-- 각속도 제어기 게인값 설정(P, I, D)
	//
	PID[ROLL ].set_gain_rate(28, 0, 7);
	PID[PITCH].set_gain_rate(28, 0, 7);
	PID[YAW  ].set_gain_rate(68, 0, 0);


	//-- 전원 On시에 가속도 센서 캘리브레이션 실시  
	//   수평인 상태에서 진행해야 함
	//
	IMU.SEN.acc_cali_start();
	while( IMU.SEN.acc_cali_get_done() == false )
	{
		IMU.update();
		LED.toggle();
	}
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
	int16_t i;



	//-- LED
	//
	if( (millis()-tLed) >= 500 )
	{
		tLed = millis();

		if( MSP.Get_ArmMode() == true )
		{
			LED.on();
		}
		else
		{
			LED.toggle();
		}
	}


	//-- 명령어 처리
	//
	if( MSP.update() == true )
	{
		tMsp = millis();
		target_roll    = MSP.Get_Roll();
		target_pitch   = MSP.Get_Pitch();
		target_yaw     = MSP.Get_Yaw();
		target_throtle = MSP.Get_Throtle();
	}


	//-- ARM모드가 아니면 모터 정지 
	//
	if( MSP.Get_ArmMode() == false )
	{
		target_roll    = 0;
		target_pitch   = 0;
		target_yaw     = 0;
		target_throtle = 0;
	}


	//-- FailSafe 기능 
	//   1000ms안에 통신 명령어가 들어오지 않으면 정지함 
	//
	if( (millis()-tMsp) > 1000 )
	{
		tMsp = millis();

		#if _USE_FAIL_SAVE == 1
		target_roll    = 0;
		target_pitch   = 0;
		target_yaw     = 0;
		target_throtle = 0;
		#endif
	}


	//-- 센서값 업데이트(200Hz)
	//
	tIMU = IMU.update();


	//-- 센서값 업데이트시 PID 제어 실행  
	//
	if( tIMU > 0 )
	{
		//-- PID 제어값 계산  
		//
		pid_out[ROLL ] = PID[ROLL ].update( PID_ANGLE, target_roll , IMU.angle[ROLL ], IMU.gyroData[ROLL ], tIMU );
		pid_out[PITCH] = PID[PITCH].update( PID_ANGLE, target_pitch, IMU.angle[PITCH], IMU.gyroData[PITCH], tIMU );
		pid_out[YAW  ] = PID[YAW  ].update( PID_RATE , target_yaw  , IMU.angle[YAW  ], IMU.gyroData[YAW  ], tIMU );


		//-- 모터출력 분배
		//
		motor_pwm[REAR_R ] = PIDMIX(-1,+1,-1); 
		motor_pwm[FRONT_R] = PIDMIX(-1,-1,+1); 
		motor_pwm[REAR_L ] = PIDMIX(+1,+1,+1); 
		motor_pwm[FRONT_L] = PIDMIX(+1,-1,-1); 


		//-- 모터 출력 값 범위 설정 
		//
		for( i=0; i<4; i++) 
		{
			motor_pwm[i] = constrain(motor_pwm[i], 0, 1000);

			// 스로틀이 적으면 모터 정지함 
      		if( target_throtle < 50 )
      		{
        		motor_pwm[i] = 0;
        		PID[ROLL ].reset();
        		PID[PITCH].reset();
        		PID[YAW  ].reset();
        	}

        	PWM.set_out( i, motor_pwm[i] );
		}


		#if 0
		static uint32_t tTime;

		if( millis()-tTime > 100 )
		{
			tTime = millis();
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
		#endif

	}



	//-- 모터값 업데이트 
	//
	PWM.update();


	menu_loop();
}





void menu_loop( void )
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
		else if( Ch == 'c' )
		{
			Serial.println("ACC Cali Start");

			IMU.SEN.acc_cali_start();
			while( IMU.SEN.acc_cali_get_done() == false )
			{
				IMU.update();
				//Serial.println( IMU.SEN.calibratingA );
			}

			Serial.print("ACC Cali End ");
		}
	}
}
