#include <Encoder.h>
#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
// #include <std_msgs/Float32MultiArray.h>

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
  } // сдвигаем элементы в массиве, начиная с последнего, и ЗАТИРАЯ последний элемент
  buff[0] = val; // добавляем новый элемент в начало
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


ros::NodeHandle nh; // объект для работы ROS 
// nav_msgs::Odometry robot_odometry; // сообщение с одометрией
geometry_msgs::TwistStamped robot_odometry; // сообщение с винтом (линейная скорость и угловая скорость в 3D) и штампом, содержащим время и название системы координат
ros::Publisher pub_odometry("/odometry", &robot_odometry); // паблишер, для отправки сообщений одометрии на ПК
const char frame_id[] = "robot"; // СК робота
const char child_frame_id[] = "base_link"; 

float           wheelRadius = 0.0387;    // Радиус колес в метрах
float distanceBetweenWheels = 0.16324;  // Расстояние между колесом с энкодером и центром одометра в метрах (актуально для всех трёх колёс)
unsigned long      interval = 25;       // Интервал в миллисекундах 
float interval_fl; // интервал, но типа float, в секундах
float speedLeft, speedRight, speedCenter; // текущие скорости колёс

float ticks_to_rad = 2 * PI / (600*4); // коэффициент для перевода тиков энкодера в радианы







void setup() 
{
  nh.initNode(); // инициализируем ноду
  nh.advertise(pub_odometry); // регистрируем publisher в ROS
  interval_fl = (float)interval/1000; // вычисляем интервал типа float, мс --> сек

  robot_odometry.header.frame_id = frame_id; // задаём название для СК публикуемой одометрии
  // robot_odometry.child_frame_id = child_frame_id;

}

// read() -> int
// int -> float (float32), double (float64)




void loop() 
{
  unsigned long time_now = millis(); 
  if (time_now % interval == 0)  // если текущее время делится без остатка на заданный период 
  {

    // Помещаем значение угла поворота в радианах в конечный буффер-очередь
    addToBuff(encoderLeftSequence,  (float)encoderLeft.read() * ticks_to_rad); // сначала переводим тики во float, затем умножаем на коэффициент для перевода в радианы
    addToBuff(encoderRightSequence, (float)encoderRight.read() * ticks_to_rad);
    addToBuff(encoderCenterSequence,(float)encoderCenter.read() * ticks_to_rad);

    // Рассчитываются угловые скорости (рад/с)
    speedLeft   = getFirstDerivative(encoderLeftSequence,   interval_fl); // используем функцию для вычисления более точной производной
    speedRight  = getFirstDerivative(encoderRightSequence,  interval_fl);
    speedCenter = getFirstDerivative(encoderCenterSequence, interval_fl);

    // Расчитываем линейную и угловую скорости робота на плоскости, формула из статьи про кинематику мобильных роботов FTC 
    robot_odometry.header.stamp = nh.now(); // обновляем штамп времени в сообщении одометрии
    // robot_odometry.twist.linear.x = wheelRadius * (speedLeft + speedRight) / 2; // вычисляем линейную скорость робота вдоль оси X
    // robot_odometry.twist.linear.y = wheelRadius * ((speedLeft - speedRight)/2 + speedCenter); // скорость вдоль Y
    // robot_odometry.twist.angular.z = wheelRadius * (speedRight - speedLeft) / (2 * distanceBetweenWheels); // угловая скорость вокруг Z
  
    float speeda = wheelRadius * (speedLeft - speedRight)/2;
    float speedb = wheelRadius * speedCenter;
    robot_odometry.twist.linear.y = speeda;
    robot_odometry.twist.linear.x = speedb;
    robot_odometry.twist.linear.z = speeda - speedb;

    pub_odometry.publish(&robot_odometry); // публикуем сообщение одометрии в топик
    nh.spinOnce(); // НУЖНО для работы ROS 
  }
}




