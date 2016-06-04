//----------------------------------------------------------------------------
//    프로그램명 	: MPU6050
//
//    만든이     	: Baram ( chcbaram@paran.com )
//
//    날  짜     : 
//    
//    최종 수정  	: 
//
//    MPU_Type	: 
//
//    파일명     	: MPU6050.cpp
//----------------------------------------------------------------------------
/*
	Based on Multiwii : https://github.com/multiwii/multiwii-firmware
*/



#include <Arduino.h> 
#include "I2C.h"
#include "SRF02.h"




#define SRF02_ADDRESS     (0xE0>>1)






/*---------------------------------------------------------------------------
     TITLE   : cSRF02
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
cSRF02::cSRF02()
{	
	distance = 0;
	connected = false;
}




/*---------------------------------------------------------------------------
     TITLE   : begin
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cSRF02::begin( void )
{
	uint8_t ret;


	I2C.write_reg(SRF02_ADDRESS, 0x00, 0x51); // Real Ranging Mode - Result in centimeters
	delay(50);

	ret = I2C.read_reg(SRF02_ADDRESS, 0x00); // Read Soft Revision 

	if( I2C.i2c_errors_count > 0 )
	{
		I2C.i2c_errors_count = 0;
		connected = false;
	}
	else
	{
		if( ret != 0xFF)
		{
			connected = true;
		}
	}	
}



/*---------------------------------------------------------------------------
     TITLE   : update
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cSRF02::update( void ) 
{
	static uint32_t tTime;
	bool ret = false;
	uint16_t range;
	static uint16_t lpf[5] = { 0, };


	if( connected == true )
	{
		if( (micros()-tTime) >= 70000 )	// 70ms
		{
			tTime = micros();

			range  = I2C.read_reg(SRF02_ADDRESS, 0x03);
			range |= I2C.read_reg(SRF02_ADDRESS, 0x02) << 8;

			lpf[0] = range;
			lpf[1] = lpf[0];
			lpf[2] = lpf[1];
			lpf[3] = lpf[2];
			lpf[4] = lpf[3];

			distance = (lpf[0]+lpf[1]+lpf[2]+lpf[3]+lpf[4])/5;

			I2C.write_reg(SRF02_ADDRESS, 0x00, 0x51); // Real Ranging Mode - Result in centimeters

			ret = true;
			//Serial.println( distance );
		}
	}


	return ret;
}



/*---------------------------------------------------------------------------
     TITLE   : is_connected
     WORK    : 
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cSRF02::is_connected() 
{
	return connected;
}

