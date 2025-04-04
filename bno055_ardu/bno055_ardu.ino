#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt8.h>
#include <tf/tf.h>
#include <EEPROM.h>



Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
bool bno_calibrated;
uint16_t BNO_DT_MS = 10;
float BNO_DT_SEC =  (float)(BNO_DT_MS) / 1000.0;
float BNO_DT_SEC_2 = BNO_DT_SEC * BNO_DT_SEC;
float ACCEL_SCALE = 1;
float DEG_2_RAD = 0.01745329251; 
float ACC_THRESH = 0.1;

float accXBuff[2] = {0.0, 0.0};
float accYBuff[2] = {0.0, 0.0};
float posXBuff[2] = {0.0, 0.0};
float posYBuff[2] = {0.0, 0.0};

uint8_t bno_sys_status, bno_gyr_status, bno_acc_status, bno_mag_status;

ros::NodeHandle nh; 
geometry_msgs::PoseStamped robot_pose; 
ros::Publisher pub_odometry("/sisyph/imu/pose", &robot_pose); 
const char frame_id[] = "robot"; 
 
unsigned long ros_sub_interval = 100;   


int R_LED = 22;
int B_LED = 24;
int G_LED = 26;

void setup() 
{
	initGPIO();
	initROS();
	initBNO();
}

void loop() 
{
	if(bno_calibrated)
	{	
		mainLoop();	
	}
	else
	{	
		calibLoop();	
	}
}


void initGPIO()
{
	pinMode(R_LED, OUTPUT);
	pinMode(B_LED, OUTPUT);
	pinMode(G_LED, OUTPUT);
	digitalWrite(R_LED, HIGH);
	digitalWrite(B_LED, HIGH);
	digitalWrite(G_LED, HIGH);	
}


void initROS()
{
	nh.initNode(); 
	nh.advertise(pub_odometry); 		

	robot_pose.header.frame_id = frame_id; 
	robot_pose.pose.orientation = tf::createQuaternionFromYaw(0.0);
	robot_pose.pose.position.x = 0.0;
	robot_pose.pose.position.y = 0.0;
	robot_pose.pose.position.z = 0.0;
}


void initBNO()
{
	bno_calibrated = false;

	while(!bno.begin())
	{
		digitalWrite(R_LED, LOW);
		delay(1000);
		digitalWrite(R_LED, HIGH);
		delay(1000);
	}
	digitalWrite(R_LED, HIGH);

	bno.setExtCrystalUse(true);
	bno.setMode(OPERATION_MODE_IMUPLUS);

	digitalWrite(B_LED, LOW);
	delay(2000);
	digitalWrite(B_LED, HIGH);
}






void calibLoop()
{
    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    bno.getSensor(&sensor);
    if (bnoID == sensor.sensor_id)
    {
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);
        bno.setSensorOffsets(calibrationData);
        foundCalib = true;
    }

	if(!foundCalib)
	{
		uint8_t system, gyr, acc, mag;
		system = gyr = acc = mag = 0;

		do
		{
			bno.getCalibration(&system, &gyr, &acc, &mag);

			if(gyr!=3)
			{
				digitalWrite(R_LED, LOW);
				digitalWrite(B_LED, HIGH);
				digitalWrite(G_LED, HIGH);
				delay(200);
			}
			if(acc!=3)
			{
				digitalWrite(R_LED, HIGH);
				digitalWrite(B_LED, LOW);
				digitalWrite(G_LED, HIGH);
				delay(200);
			}
			// if(mag!=3)
			// {
			// 	digitalWrite(R_LED, HIGH);
			// 	digitalWrite(B_LED, HIGH);
			// 	digitalWrite(G_LED, LOW);
			// 	delay(200);
			// }	
			digitalWrite(R_LED, HIGH);
			digitalWrite(B_LED, HIGH);
			digitalWrite(G_LED, HIGH);	
			delay(200);				

		}while(gyr!=3 || acc!=3);

		adafruit_bno055_offsets_t newCalib;
		bno.getSensorOffsets(newCalib);

		eeAddress = 0;
		bno.getSensor(&sensor);
		bnoID = sensor.sensor_id;

		EEPROM.put(eeAddress, bnoID);

		eeAddress += sizeof(long);
		EEPROM.put(eeAddress, newCalib);		
	}

	digitalWrite(R_LED, LOW);
	digitalWrite(B_LED, LOW);
	digitalWrite(G_LED, LOW);
	delay(400);
	digitalWrite(R_LED, HIGH);
	digitalWrite(B_LED, HIGH);
	digitalWrite(G_LED, HIGH);	
	delay(400);
	digitalWrite(R_LED, LOW);
	digitalWrite(B_LED, LOW);
	digitalWrite(G_LED, LOW);
	delay(400);
	digitalWrite(R_LED, HIGH);
	digitalWrite(B_LED, HIGH);
	digitalWrite(G_LED, HIGH);	

	bno_calibrated = true;
}




void mainLoop()
{
	unsigned long time_now = millis(); 
	if (time_now % BNO_DT_MS == 0) 
	{
		sensors_event_t orientData , linAccData;
		bno.getEvent(&orientData, Adafruit_BNO055::VECTOR_EULER);
		bno.getEvent(&linAccData, Adafruit_BNO055::VECTOR_LINEARACCEL);

		robot_pose.pose.position.x =verletPos(	posXBuff, 
												accXBuff, 
												linAccData.acceleration.x * ACCEL_SCALE,
												BNO_DT_SEC_2, 
												ACC_THRESH);
		robot_pose.pose.position.y =verletPos(	posYBuff, 
												accYBuff, 
												linAccData.acceleration.y * ACCEL_SCALE,
												BNO_DT_SEC_2, 
												ACC_THRESH);

		robot_pose.header.stamp = nh.now();
		robot_pose.pose.orientation = tf::createQuaternionFromYaw(orientData.orientation.roll * DEG_2_RAD);

		pub_odometry.publish(&robot_pose); 
	}

	if (time_now % ros_sub_interval == 0){ nh.spinOnce(); }
}





float shrinkVal(float val, float thresh)
{
  if(val > 0.0){ if(val < thresh){ val = 0.0; } }
  else if(val < 0.0){ if(val > -thresh){ val = 0.0; } }
  return val;
}



float verletPos(float * pos_buff, float * acc_buff, float new_acc, float dt_2, float shrink_thresh) 
{
	acc_buff[1] = acc_buff[0];
	acc_buff[0] = shrinkVal(new_acc, shrink_thresh);			
	float new_pos = 2*pos_buff[0] - pos_buff[1] + acc_buff[1]*dt_2;
	pos_buff[1] = pos_buff[0];
	pos_buff[0] = new_pos;
	return new_pos;
}
