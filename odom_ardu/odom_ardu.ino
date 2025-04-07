#include <Encoder.h>
#include <ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>


float shrinkVal(float val, float thresh)
{
  if(val > 0.0){ if(val < thresh){ val = 0.0; } }
  else if(val < 0.0){ if(val > -thresh){ val = 0.0; } }
  return val;
}




int enc_L_pin_A = 21; 
int enc_L_pin_B = 20; 
int enc_R_pin_A = 18; 
int enc_R_pin_B = 19;
int enc_C_pin_A = 3; 
int enc_C_pin_B = 2; 

Encoder   enc_L(enc_L_pin_A, enc_L_pin_B);
Encoder   enc_R(enc_R_pin_A, enc_R_pin_B);
Encoder   enc_C(enc_C_pin_A, enc_C_pin_B);

float enc_L_seq[2] = {0.0, 0.0}; // массив для хранения последних 2 значения показаний энкодеров, переведённых в радианы
float enc_R_seq[2] = {0.0, 0.0}; 
float enc_C_seq[2] = {0.0, 0.0};


ros::NodeHandle nh; 
geometry_msgs::Vector3 robot_dpos; 
geometry_msgs::Vector3Stamped robot_pos;
ros::Publisher pos_pub("/sisyph/odom/pos", &robot_pos); 

float R_w = 0.0387;    // радиус колес в метрах
float  d_RLwc = 0.16979;  // расстояние между боковым роликом и центром
unsigned long interval = 10;      
unsigned long time_now, time_last; 
float interval_fl; 
float w_L, w_R, w_C; 

float ticks_to_rad = 2 * PI / (600*4); 







void setup() 
{
  nh.initNode(); 
  nh.advertise(pos_pub);

  time_last = millis();
  interval_fl = (float)interval/1000; 

	robot_pos.vector.x = 0.0;
	robot_pos.vector.y = 0.0;
	robot_pos.vector.z = 0.0;

}






void loop() 
{
	time_now = millis(); 
	if (time_now - time_last >= interval)  
	{
		time_last = time_now;

		robot_pos.header.stamp = nh.now(); 
		enc_L_seq[1] = enc_L_seq[0];      enc_L_seq[0] = (float)enc_L.read() * ticks_to_rad;
		enc_R_seq[1] = enc_R_seq[0];      enc_R_seq[0] = (float)enc_R.read() * ticks_to_rad;
		enc_C_seq[1] = enc_C_seq[0];      enc_C_seq[0] = (float)enc_C.read() * ticks_to_rad;

		w_L = enc_L_seq[0] - enc_L_seq[1]; 
		w_R = enc_R_seq[0] - enc_R_seq[1]; 
		w_C = enc_C_seq[0] - enc_C_seq[1]; 

		robot_dpos.x =    R_w * (w_L + w_R)/2; 
		robot_dpos.y = -1*R_w * ((w_L - w_R)/2 + w_C);
		robot_dpos.z = -1*R_w * (w_R - w_L) / (2 * d_RLwc); 

		robot_pos.vector.z += robot_dpos.z;    
		robot_pos.vector.x += robot_dpos.x*cos(robot_pos.vector.z) - robot_dpos.y*sin(robot_pos.vector.z); 
		robot_pos.vector.y += robot_dpos.x*sin(robot_pos.vector.z) + robot_dpos.y*cos(robot_pos.vector.z); 

		pos_pub.publish(&robot_pos);
		nh.spinOnce(); 
	}
}




