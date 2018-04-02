#include "motor_hat.h"

const int MODE1 = 0x00;
const int MODE2 = 0x01;
const int SUBADR1 = 0x02;
const int SUBADR2 = 0x03;
const int SUBADR3 = 0x04;
const int PRESCALE = 0xFE;
const int LED0_ON_L = 0x06;
const int LED0_ON_H = 0x07;
const int LED0_OFF_L = 0x08;
const int LED0_OFF_H = 0x09;
const int ALL_LED_ON_L = 0xFA;
const int ALL_LED_ON_H = 0xFB;
const int ALL_LED_OFF_L = 0xFC;
const int ALL_LED_OFF_H = 0xFD;

    // Bits
const int RESTART = 0x80;
const int SLEEP = 0x10;
const int ALLCALL = 0x01;
const int NVRT = 0x10;
const int OUTDRV = 0x04;

namespace motor_hat
{
	motor_hat::motor_hat()
	{
		int model = 0;
		
		i2c_add = wiringPiI2CSetup (0x60);
		set_all_pwm(0,0);
		wiringPiI2CWriteReg8(i2c_add, MODE2, OUTDRV);
      		wiringPiI2CWriteReg8(i2c_add, MODE1, ALLCALL);
      		delay(.005);
	
		model = wiringPiI2CReadReg8(i2c_add, 0x00) ;
		model = model & (~0x10);
		
		wiringPiI2CWriteReg8 (i2c_add, 0x00, model) ;
	
		delay(.005);
		
		set_pwm_freq();
	
	}
	
	void motor_hat::set_pwm_freq()
	{
		int oldMode = 0;
		int newMode = 0;
		
		oldMode = wiringPiI2CReadReg8 (i2c_add, 0x00) ;
		newMode = (oldMode & 0x7F) | 0x10;
		wiringPiI2CWriteReg8 (i2c_add, 0x00, newMode) ;

		wiringPiI2CWriteReg8 (i2c_add, 0xFE, 3) ;
		wiringPiI2CWriteReg8 (i2c_add, 0x00, oldMode) ;
		
		delay(.005);
		
		wiringPiI2CWriteReg8 (i2c_add, 0x00, (oldMode | 0x08)) ;
	}
	
	void motor_hat::set_speed(int motor, int speed, int dir)
	{
        	int pwm = 0, in1 = 0, in2 = 0;
        	int num = motor;

	        if (num == 0)
	        {
	                 pwm = 8;
	                 in2 = 9;
	                 in1 = 10;
	        }
	        else if (num == 1)
	        {
	                 pwm = 13;
	                 in2 = 12;
	                 in1 = 11;
	        }
	        else if (num == 2)
	        {
	                 pwm = 2;
	                 in2 = 3;
	                 in1 = 4;
	        }
	        else if(num == 3)
	        {
	                 pwm = 7;
	                 in2 = 6;
	                 in1 = 5;
	        }
	       	int pwm_pin = pwm;
	        int IN1_pin = in1;
        	int IN2_pin = in2;
        	
        	if (dir == 1)
        	{
	           	set_pin(IN2_pin, 0);
	           	set_pin(IN1_pin, 1);
	        }
	        if (dir == -1)
	        {
	          	set_pin(IN1_pin, 0);
	        	set_pin(IN2_pin, 1);
	        }
	        if (dir == 0)
	        {
	            	set_pin(IN1_pin, 0);
            		set_pin(IN2_pin, 0);
           	}
        	
        	if (speed < 0)
	            speed = 0;
	        if (speed > 255)
	            speed = 255;
        	set_pwm(pwm_pin, 0, speed*16);
	}
	
	void motor_hat::set_pin(int pin, int value)
	{
	        if (value == 0)
	        	set_pwm(pin, 0, 4096);
	        if (value == 1)
            		set_pwm(pin, 4096, 0);
        }
        
        void motor_hat::set_pwm(int channel, int on, int off)
        {
                wiringPiI2CWriteReg8(i2c_add, LED0_ON_L+4*channel, on & 0xFF);
	        wiringPiI2CWriteReg8(i2c_add, LED0_ON_H+4*channel, on >> 8);
	        wiringPiI2CWriteReg8(i2c_add, LED0_OFF_L+4*channel, off & 0xFF);
       		wiringPiI2CWriteReg8(i2c_add, LED0_OFF_H+4*channel, off >> 8);
        }
	
	void motor_hat::set_all_pwm(int on, int off)
	{
	        wiringPiI2CWriteReg8(i2c_add, ALL_LED_ON_L, on & 0xFF);
	        wiringPiI2CWriteReg8(i2c_add, ALL_LED_ON_H, on >> 8);
	        wiringPiI2CWriteReg8(i2c_add, ALL_LED_OFF_L, off & 0xFF);
        	wiringPiI2CWriteReg8(i2c_add, ALL_LED_OFF_H, off >> 8);
        }
}