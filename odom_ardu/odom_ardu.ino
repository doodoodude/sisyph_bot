#include <Encoder.h>
#include <ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TwistStamped.h>

#define BUFF_SIZE 4

// Функция для сдвига элементов массива и добавления нового числа в начало (самое старое число пропадает).
//
// На вход идёт указатель на массив и новая величина, которую нужно добавить в массив.
// Функция модифицирует массив, указатель на который передаётся в неё
void addToBuff(float * buff, float val) 
{
  for(size_t i_=BUFF_SIZE-1; i_>=1; i_--)
  {    
    buff[i_] = buff[i_-1];    
  } 
  buff[0] = val; 
}





// Фунция для вычисления более точной производной на основе последних 4 значений угла поворота.
//
// sequence - массив с 4-мя последними значения величины, для которой вычисляем производную
// dt - равный интервал времени между 4-мя значениями величины в массиве
float getFirstDerivative(float * sequence, float dt) 
{
  return (11*sequence[0]/6 - 3*sequence[1] + 3*sequence[2]/2 - sequence[3]/3)/dt;  // формула из википедии
}



float shrinkVal(float val, float thresh)
{
  if(val > 0.0){ if(val < thresh){ val = 0.0; } }
  else if(val < 0.0){ if(val > -thresh){ val = 0.0; } }
  return val;
}




int    encoderLeftPinA = 21; 
int    encoderLeftPinB = 20; 
int   encoderRightPinA = 18; 
int   encoderRightPinB = 19;
int  encoderCenterPinA = 3; 
int  encoderCenterPinB = 2; 

Encoder     encoderLeft(encoderLeftPinA, encoderLeftPinB);
Encoder   encoderRight(encoderRightPinA, encoderRightPinB);
Encoder encoderCenter(encoderCenterPinA, encoderCenterPinB);

float encoderLeftSequence[BUFF_SIZE] = {0.0, 0.0, 0.0, 0.0}; // массив для хранения последних 4 значения показаний энкодеров, переведённых в радианы
float encoderRightSequence[BUFF_SIZE] = {0.0, 0.0, 0.0, 0.0}; 
float encoderCenterSequence[BUFF_SIZE] = {0.0, 0.0, 0.0, 0.0};


ros::NodeHandle nh; 
geometry_msgs::TwistStamped robot_vel; 
geometry_msgs::TransformStamped robot_tf;
tf::TransformBroadcaster tf_pub;
float z_orient=0.0;
const char frame_id[] = "odom";
const char child_frame_id[] = "robot"; 

float R_w = 0.0387;    // радиус колес в метрах
float  d_RL = 0.16979;  // расстояние между правым и левым роликом
unsigned long interval = 10;       
float interval_fl; 
float w_L, w_R, w_C; 

float ticks_to_rad = 2 * PI / (600*4); 







void setup() 
{
  nh.initNode(); 
  tf_pub.init(nh); 
  interval_fl = (float)interval/1000; 

	robot_tf.header.frame_id = frame_id; 
  robot_tf.child_frame_id = child_frame_id;  
	robot_tf.transform.rotation = tf::createQuaternionFromYaw(0.0);
	robot_tf.transform.translation.x = 0.0;
	robot_tf.transform.translation.y = 0.0;
	robot_tf.transform.translation.z = 0.0;

}






void loop() 
{
  unsigned long time_now = millis(); 
  if (time_now % interval == 0)  // если текущее время делится без остатка на заданный период 
  {

// read() -> int
// int -> float (float32), double (float64)
    addToBuff(encoderLeftSequence,  (float)encoderLeft.read() * ticks_to_rad); 
    addToBuff(encoderRightSequence, (float)encoderRight.read() * ticks_to_rad);
    addToBuff(encoderCenterSequence,(float)encoderCenter.read() * ticks_to_rad);

    w_L   = getFirstDerivative(encoderLeftSequence,   interval_fl); 
    w_R  = getFirstDerivative(encoderRightSequence,  interval_fl);
    w_C = getFirstDerivative(encoderCenterSequence, interval_fl);
 
    robot_vel.twist.linear.x = R_w * (w_L + w_R)/2; 
    robot_vel.twist.linear.y = -R_w * ((w_L - w_R)/2 + w_C);
    robot_vel.twist.angular.z = -R_w * (w_R - w_L) / (2 * d_RL); 

    robot_tf.header.stamp = nh.now(); 
    robot_tf.transform.translation.x += robot_vel.twist.linear.x * interval_fl;
    robot_tf.transform.translation.y += robot_vel.twist.linear.y * interval_fl;
    z_orient += robot_vel.twist.angular.z * interval_fl;
    robot_tf.transform.rotation = tf::createQuaternionFromYaw(z_orient);

    tf_pub.sendTransform(robot_tf);
    nh.spinOnce(); 
  }
}




