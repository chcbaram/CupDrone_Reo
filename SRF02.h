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
//    파일명     	: SRF02.h
//----------------------------------------------------------------------------
#ifndef _SRF02_H_
#define _SRF02_H_

#include <inttypes.h>
#include <Arduino.h> 

#include "Define.h"







class cSRF02
{

public:	
	int16_t  distance;
	bool     connected;

public:
	cSRF02();

	void begin( void );
	bool update( void );

	bool is_connected( void );
};


#endif