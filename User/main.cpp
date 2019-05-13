#include <stdio.h>
#include "hardwareserial.h"
#include "gy85.h"
#include "motor.h"
#include "encoder.h"
#include "battery.h"
#include "led.h"
#include "PID.h"
#include "sonar.h"
#include "bumper.h"
#include "Kinematics.h"
#include "beep.h"

#include <ros.h>
#include <riki_msgs/Velocities.h>
#include <geometry_msgs/Twist.h>
#include <riki_msgs/PID.h>
#include <riki_msgs/Imu.h>
#include <riki_msgs/Battery.h>
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>
extern uint8_t distance_sonars[4];
  
double required_angular_vel = 0;
double required_linear_vel = 0;
uint32_t previous_command_time = 0;

bool is_first = true;
bool accel, gyro, mag;

PID motor1_pid(K_P, K_I, K_D);
PID motor2_pid(K_P, K_I, K_D);

Motor motor;

//左侧和右侧轮子编码器里程计
Encoder encoder1(ENCODER1, COUNTS_PER_REV);
Encoder encoder2(ENCODER2, COUNTS_PER_REV);

Battery bat(25, 46.0, 53.0);//机械臂电池电量

//运动学
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

Gy85  imu;
Led led;
bool OnOff = false; 
void pid_callback( const riki_msgs::PID& pid);
void command_callback( const geometry_msgs::Twist& cmd_msg);

ros::NodeHandle  nh;

riki_msgs::Imu raw_imu_msg;
riki_msgs::Velocities raw_vel_msg;
riki_msgs::Battery raw_battery_msg;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
ros::Subscriber<riki_msgs::PID> pid_sub("pid", pid_callback);
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
ros::Publisher raw_battery_pub("battery", &raw_battery_msg);
 


//更新pid参数
void pid_callback( const riki_msgs::PID& pid)
{
    motor1_pid.updateConstants(pid.p, pid.i, pid.d);
    motor2_pid.updateConstants(pid.p, pid.i, pid.d);
}

//控制轮子运动
void command_callback( const geometry_msgs::Twist& cmd_msg)
{
    required_linear_vel = cmd_msg.linear.x;
    required_angular_vel = cmd_msg.angular.z;

    previous_command_time = millis();
}

//20ms的控制周期
void move_base()
{
	Kinematics::rpm req_rpm = kinematics.getRPM(required_linear_vel, 0, required_angular_vel);
	
	int current_rpm1 = encoder1.getRPM();
  int current_rpm2 = encoder2.getRPM(); //获取脉冲
	motor.spin();//需要修改，发送指令控制电机运动，控制小车以目标速度移动，移动时间为0.4ms
	
	Kinematics::velocities current_vel;
	current_vel = kinematics.getVelocities(current_rpm1, current_rpm2);
	
	//fill in the object
  raw_vel_msg.linear_x = current_vel.linear_x;
  raw_vel_msg.linear_y = 0.0;
  raw_vel_msg.angular_z = current_vel.angular_z;

   //publish raw_vel_msg object to ROS
   raw_vel_pub.publish(&raw_vel_msg); //实时发送当前运行速度
}

void check_imu()
{
    gyro = imu.check_gyroscope();
    accel = imu.check_accelerometer();
    mag = imu.check_magnetometer();

    if (!accel){
        nh.logerror("Accelerometer NOT FOUND!");
    }   

    if (!gyro){
        nh.logerror("Gyroscope NOT FOUND!");
    }   

    if (!mag){
        nh.logerror("Magnetometer NOT FOUND!");
    }
    is_first = false;
}

void publish_imu()
{
    //geometry_msgs::Vector3 acceler, gyro, mag;
    //this function publishes raw IMU reading
    //measure accelerometer
    if (accel){
        imu.measure_acceleration();
        raw_imu_msg.linear_acceleration = imu.raw_acceleration;
    }

    //measure gyroscope
    if (gyro){
        imu.measure_gyroscope();
        raw_imu_msg.angular_velocity = imu.raw_rotation;
    }

    //measure magnetometer
    if (mag){
        imu.measure_magnetometer();
        raw_imu_msg.magnetic_field = imu.raw_magnetic_field;
    }

    //publish raw_imu_msg object to ROS
    raw_imu_pub.publish(&raw_imu_msg);

}


void stop_base()
{
    required_linear_vel = 0;
    required_angular_vel = 0;
}

void publishBAT()
{
	raw_battery_msg.battery = bat.get_volt();
	raw_battery_pub.publish(&raw_battery_msg);
}

void print_debug()
{
    char buffer[50]; 
			 
		read_distances();
	
		sprintf (buffer, "Sonar AA: %ld", distance_sonars[0]);
    nh.loginfo(buffer);
    sprintf (buffer, "Sonar BB: %ld", distance_sonars[1]);
    nh.loginfo(buffer);
		sprintf (buffer, "Sonar CC: %ld", distance_sonars[2]);
    nh.loginfo(buffer);
		sprintf (buffer, "Sonar DD: %ld", distance_sonars[3]);
    nh.loginfo(buffer); 
}


int main(void) 
{
	
	uint32_t previous_battery_debug_time = 0;
	uint32_t previous_debug_time = 0;
	uint32_t previous_imu_time = 0;
	uint32_t previous_control_time = 0;
	uint32_t previous_bumper_front_time = 0;
	uint32_t previous_bumper_back_time = 0;
	uint32_t publish_vel_time = 0;
	bool flag_beep_bumper = false;
	char battery_buffer[]= "The voltage is lower than 11.3V,Please charge! ";

	SystemInit();
	initialise(); 
	motor.init();
	sonar_init();
	encoder1.init();
	encoder2.init();
	led.init();
	imu.init();
	bat.init();
	bumper_init();
	nh.initNode();	
	nh.advertise(raw_vel_pub);//原始速度
	nh.advertise(raw_imu_pub);//原始imu
	nh.advertise(raw_battery_pub);//电池发布
	nh.subscribe(pid_sub);//pid接收
	nh.subscribe(cmd_sub);//运动指令接收

	while (!nh.connected()){
		nh.spinOnce();
	}   
	nh.loginfo("Rikibase Connected!");


//时间单位为ms
	while(1){
		
		/** Move base **/
		if ((millis() - previous_control_time) >= (1000 / COMMAND_RATE)){
	   move_base();
       previous_control_time = millis();
    }

		
    if ((millis() - previous_command_time) >= 400){
       stop_base();
    }
 
		/** IMU **/
		if ((millis() - previous_imu_time) >= (1000 / IMU_PUBLISH_RATE)){
			if(is_first){
				//sanity check if the IMU exits
				check_imu();
			} else{
				//publish the IMU data
				publish_imu();
			}
			previous_imu_time = millis();
		}
	
		/** Battery **/
		if( (millis() - previous_battery_debug_time) >= (1000 / BAT_PUBLISH_RATE)){
			if(bat.get_volt() < Voltage_Threshold){ 
				nh.logwarn(battery_buffer);			
			}
			publishBAT();
			previous_battery_debug_time = millis();		
		}

		/** Debug **/
		if(DEBUG){  
			if ((millis() - previous_debug_time) >= (1000 / DEBUG_RATE)) {
				
				print_debug();
				previous_debug_time = millis();
			}
    }
 
		/** bumper **/
		if(millis() - previous_bumper_front_time > 300 && BUMPER_FRONT == 0)
		{
			previous_bumper_front_time = millis();
			if(flag_beep_bumper == false) //控制蜂鸣器的音调
			{
				bumper_init(400,30);
				TIM_SetCompare2(TIM3,50);	
			}
			else
			{
				bumper_init(400,30);
				TIM_SetCompare2(TIM3,0);	
			}
			char buffer_bumper[50]; 
		  sprintf (buffer_bumper, "bumper front fffffffffffff");
		  nh.loginfo(buffer_bumper);
		}
		
		if(millis() - previous_bumper_back_time > 300 && BUMPER_BACK == 0)
		{
			previous_bumper_back_time = millis();
			if(flag_beep_bumper == false) //控制蜂鸣器的音调
			{
				bumper_init(400,30);
				TIM_SetCompare2(TIM3,50);	
			}
			else
			{
				bumper_init(400,30);
				TIM_SetCompare2(TIM3,0);	
			}
			char buffer_bumper[50]; 
		  sprintf (buffer_bumper, "bumper back bbbbbbbbbbbb");
		  nh.loginfo(buffer_bumper);
		}
		
		if(BUMPER_BACK == 1 && BUMPER_FRONT == 1)
		{ 
			TIM_SetCompare2(TIM3,0);	
		}
		
		nh.spinOnce();
  } 
}


