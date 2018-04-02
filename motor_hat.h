#include <wiringPi.h>
#include <wiringPiI2C.h>

namespace motor_hat
{
	class motor_hat
	{
		private:
			void set_pin(int pin, int value);
			void set_all_pwm(int on, int off);
			void set_pwm_freq();
			void set_pwm(int channel, int on, int off);
		public:
			motor_hat();
			void set_speed(int motor, int speed, int dir);
			int i2c_add;
	};


}

