#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "config.h"

#define constrain(amt,low,high) \
	((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


class Motor {
	public:
		int rpm;
		static int counts_per_rev_;
		Motor();
		void init();
		void spin();

	private: 
};

#endif //_MOTOR_H_
