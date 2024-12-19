#include <ros.h>
#include <std_msgs/Int8MultiArray.h>

ros::NodeHandle nh;

const int motorCount = 4;
// static const float _l = 0.168; 
// static const float _w = 0.25; 
// static const float _r = 0.1; 
int R_EN[motorCount] = {24, 28, 32, 35};
int RPWM[motorCount] = {3, 5, 7, 9};
int L_EN[motorCount] = {22, 26, 30, 34};
int LPWM[motorCount] = {2, 4, 6, 8};
int cmd_sign[motorCount] = {-1, 1, 1, -1};


void commandCallback(const std_msgs::Int8MultiArray& cmd_msg) {
  if(!cmd_msg.data[4]) {
    for (int i = 0; i < motorCount; i++) {
      setMotorSpeed(RPWM[i], LPWM[i], cmd_sign[i]*cmd_msg.data[i]); 
    }
  }
  else
  {
    for (int i = 0; i < motorCount; i++) {
      setMotorSpeed(RPWM[i], LPWM[i], 0); 
    }
  }

}

ros::Subscriber<std_msgs::Int8MultiArray> sub("/sisyph/wheel_cmd", commandCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);

  for (int i = 0; i < motorCount; i++) {
    pinMode(R_EN[i], OUTPUT); // настраиваем пины
    pinMode(RPWM[i], OUTPUT);
    pinMode(L_EN[i], OUTPUT);
    pinMode(LPWM[i], OUTPUT);
    digitalWrite(R_EN[i], HIGH);// "включаем" пин, для управления вращением "вперёд"
    digitalWrite(L_EN[i], HIGH);// "включаем" пин, для управления вращением "назад"
  }
}


void loop() {
  nh.spinOnce();
  delay(10);
//  setMotorSpeed(RPWM[3], LPWM[3], 50); 
//  delay(1000);
//  setMotorSpeed(RPWM[3], LPWM[3], 0);
//  delay(500);
//  setMotorSpeed(RPWM[3], LPWM[3], -50);  
//  delay(2000);
//  setMotorSpeed(RPWM[3], LPWM[3], 0);
//  delay(500);
}



// Функция для установки скорости и направления двигателя
void setMotorSpeed(int RPWM, int LPWM, int speed) {
  if (speed >= 0) {
    // Направление вперед
    analogWrite(RPWM, speed);
    analogWrite(LPWM, 0);
  } else {
    // Направление назад
    analogWrite(RPWM, 0);
    analogWrite(LPWM, -speed);
  }
}
