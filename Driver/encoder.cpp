#include "encoder.h"
 
Encoder::Encoder(Encoder_TypeDef _encoder,  uint32_t _counts_per_rev)
{
	encoder = _encoder; 
	counts_per_rev = _counts_per_rev;

	position = 0;//总行驶数
	last_timer = 0;//上一次的计数器寄存器的数值
	last_timer_diff = 0;//单位之间内的运行数值
}

//初始化串口3中断，读取脉冲数
void Encoder::init()
{

}

int32_t Encoder::read(uint16_t timer_value)
{ 
	//获取编码器脉冲数 
	last_timer_diff = timer_value - last_timer;
	last_timer = timer_value;
	position += (int32_t) last_timer_diff;

	return position;
}

//计算脉冲数值转化为里程值
int32_t Encoder::getRPM()
{
		
		uint16_t timer_value = 0; 
	//TODO::usart2 read data
		
		long encoder_ticks = read(timer_value);
		//this function calculates the motor's RPM based on encoder ticks and delta time
		unsigned long current_time = millis();
		unsigned long dt = current_time - prev_update_time_;//当前时间减去上一次时间

		//convert the time from milliseconds to minutes
		double dtm = (double)dt / 60000;  
		double delta_ticks = encoder_ticks - prev_encoder_ticks_;

		//calculate wheel's speed (RPM)

		prev_update_time_ = current_time;
		prev_encoder_ticks_ = encoder_ticks;
		
		return (delta_ticks / counts_per_rev) / dtm;
		return 0;
}


void Encoder::set_pos(int32_t pos)
{
	position = pos;
}
