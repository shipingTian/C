#include "stm32f10x.h"           // Device header
#include "OLED.h"
#include "MPU6050.h"
#include <math.h>
#include "Kalman.h"

int main()
{
	OLED_Init();
	MPU6050_Init();	
	Kalman_Init();

	while(1) 
	{
		Kalman_Calculate();
	    OLED_ShowSignedNum(0,0,pitch,4,OLED_8X16);
    	OLED_ShowSignedNum(0,32,roll,4,OLED_8X16);
		OLED_Update();
    }	
}
