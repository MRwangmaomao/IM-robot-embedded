
#include "sonar.h"
#include "hardwareserial.h"
#include "RingBuffer.h"

HardwareSerial Serial_Sonar(SERIAL2);
uint8_t distance_sonars[4];
void sonar_init()
{
	Serial_Sonar.begin(9600);
	
}

void read_distances()
{
	if(Serial_Sonar.distance_sonar_i == 7 && Serial_Sonar.sonar_update_flag == true)
	{
		Serial_Sonar.sonar_update_flag = false;
		if(Serial_Sonar.distance_buffer[0] == 0xCB && Serial_Sonar.distance_buffer[1] == 0x55 && Serial_Sonar.distance_buffer[2] == 0x04)
		{
			distance_sonars[0] = Serial_Sonar.distance_buffer[3];
			distance_sonars[1] = Serial_Sonar.distance_buffer[4];
			distance_sonars[2] = Serial_Sonar.distance_buffer[5];
			distance_sonars[3] = Serial_Sonar.distance_buffer[6];
		}
	}
}
