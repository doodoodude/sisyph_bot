#include <Encoder.h>
#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

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




int    encoderLeftPinA = 21; // Пин A левого энкодера
int    encoderLeftPinB = 20; // Пин B левого энкодера
int   encoderRightPinA = 18; // Пин A правого энкодера
int   encoderRightPinB = 19; // Пин B правого энкодера
int  encoderCenterPinA = 3; // Пин A 3 энкодера
int  encoderCenterPinB = 2; // Пин B 3 энкодера

Encoder     encoderLeft(encoderLeftPinA, encoderLeftPinB);
Encoder   encoderRight(encoderRightPinA, encoderRightPinB);
Encoder encoderCenter(encoderCenterPinA, encoderCenterPinB);

float encoderLeftSequence[BUFF_SIZE] = {0.0, 0.0, 0.0, 0.0}; // массив для хранения последних 4 значения показаний энкодеров, переведённых в радианы
float encoderRightSequence[BUFF_SIZE] = {0.0, 0.0, 0.0, 0.0}; 
float encoderCenterSequence[BUFF_SIZE] = {0.0, 0.0, 0.0, 0.0};


ros::NodeHandle nh; 
nav_msgs::Odometry robot_odom; 
ros::Publisher pub_odom("/sisyph/odom", &robot_odom); 
float z_orient=0.0;
const char frame_id[] = "odom";
const char child_frame_id[] = "robot"; 

float R_w = 0.0387;    // Радиус колес в метрах
float  d_RL = 0.16979;  // расстояние между правым и левым роликом
unsigned long interval = 10;       
float interval_fl; // интервал, но типа float, в секундах
float w_L, w_R, w_C; 

float ticks_to_rad = 2 * PI / (600*4); 







void setup() 
{
  nh.initNode(); 
  nh.advertise(pub_odom);  
  interval_fl = (float)interval/1000; 

	robot_odom.header.frame_id = frame_id; 
  robot_odom.child_frame_id = child_frame_id;  
	robot_odom.pose.pose.orientation = tf::createQuaternionFromYaw(0.0);
	robot_odom.pose.pose.position.x = 0.0;
	robot_odom.pose.pose.position.y = 0.0;
	robot_odom.pose.pose.position.z = 0.0;

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
 
    robot_odom.header.stamp = nh.now(); // обновляем штамп времени в сообщении одометрии
    robot_odom.twist.twist.linear.x = R_w * (w_L + w_R)/2; // вычисляем линейную скорость робота вдоль оси X
    robot_odom.twist.twist.linear.y = -R_w * ((w_L - w_R)/2 + w_C); // скорость вдоль Y
    robot_odom.twist.twist.angular.z = -R_w * (w_R - w_L) / (2 * d_RL); // угловая скорость вокруг Z
  
    // robot_odom.pose.pose.position.x += robot_odom.twist.twist.linear.x * interval_fl;
    // robot_odom.pose.pose.position.y += robot_odom.twist.twist.linear.y * interval_fl;
    // z_orient += robot_odom.twist.twist.angular.z * interval_fl;
    // robot_odom.pose.pose.orientation = tf::createQuaternionFromYaw(z_orient);

    pub_odom.publish(&robot_odom);
    nh.spinOnce(); 
  }
}




