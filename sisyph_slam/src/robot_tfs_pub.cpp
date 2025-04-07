#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/OccupancyGrid.h>

ros::Publisher tf_pub;
tf2_msgs::TFMessage full_tfs_msg;
tf2_msgs::TFMessage noodom_tfs_msg;
ros::Time t_last_map;
double map_too_old_dt_secs = 5;
// double prev_time=0.0;
bool noodom = false;



void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    t_last_map = ros::Time::now();
}


void pos_vector_cb(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    // ROS_INFO_STREAM(msg->header.stamp.toSec()-prev_time);
    // prev_time = msg->header.stamp.toSec();
    full_tfs_msg.transforms[0].header.stamp = msg->header.stamp;
    full_tfs_msg.transforms[0].transform.translation.x = msg->vector.x;
    full_tfs_msg.transforms[0].transform.translation.y = msg->vector.y;
    full_tfs_msg.transforms[0].transform.rotation.z = sin(msg->vector.z/2);
    full_tfs_msg.transforms[0].transform.rotation.w = cos(msg->vector.z/2);

    full_tfs_msg.transforms[1].header.stamp = msg->header.stamp;
    full_tfs_msg.transforms[2].header.stamp = msg->header.stamp;

    if((msg->header.stamp.toSec()-t_last_map.toSec())>map_too_old_dt_secs)
    {
        if(noodom){ ROS_INFO("No map... Started sending map->odom tf");}
        noodom = false;
        tf_pub.publish(full_tfs_msg);        
    }
    else
    {
        if(!noodom){ ROS_INFO("Mapping started! Stopped sending map->odom tf");}
        noodom = true;
        noodom_tfs_msg.transforms[0] = full_tfs_msg.transforms[0];
        noodom_tfs_msg.transforms[1] = full_tfs_msg.transforms[1];
        tf_pub.publish(noodom_tfs_msg); 
    }

}





int main(int argc, char **argv)
{
    ros::init(argc,argv, "odom_tf_pub");
    ros::NodeHandle nh;

    tf_pub = nh.advertise<tf2_msgs::TFMessage>("/tf", 40, false);
    ros::Subscriber pos_vector_sub = nh.subscribe("/sisyph/odom/pos", 40, pos_vector_cb);
    ros::Subscriber map_sub = nh.subscribe("/map", 1, map_cb);

    full_tfs_msg.transforms.resize(3);

    full_tfs_msg.transforms[0].header.stamp = ros::Time::now();
    full_tfs_msg.transforms[0].header.frame_id = "odom";
    full_tfs_msg.transforms[0].child_frame_id = "robot";
    full_tfs_msg.transforms[0].transform.translation.x = 0.0;
    full_tfs_msg.transforms[0].transform.translation.y = 0.0;
    full_tfs_msg.transforms[0].transform.translation.z = 0.0;
    full_tfs_msg.transforms[0].transform.rotation.x = 0.0;
    full_tfs_msg.transforms[0].transform.rotation.y = 0.0;
    full_tfs_msg.transforms[0].transform.rotation.z = 0.0;
    full_tfs_msg.transforms[0].transform.rotation.w = 1.0;

    full_tfs_msg.transforms[1].header.stamp = ros::Time::now();
    full_tfs_msg.transforms[1].header.frame_id = "robot";
    full_tfs_msg.transforms[1].child_frame_id = "laser";
    full_tfs_msg.transforms[1].transform.translation.x = 0.224;
    full_tfs_msg.transforms[1].transform.translation.y = 0.0;
    full_tfs_msg.transforms[1].transform.translation.z = 0.0;
    full_tfs_msg.transforms[1].transform.rotation.x = 0.0;
    full_tfs_msg.transforms[1].transform.rotation.y = 0.0;
    full_tfs_msg.transforms[1].transform.rotation.z = 0.0;
    full_tfs_msg.transforms[1].transform.rotation.w = 1.0;  

    full_tfs_msg.transforms[2].header.stamp = ros::Time::now();
    full_tfs_msg.transforms[2].header.frame_id = "map";
    full_tfs_msg.transforms[2].child_frame_id = "odom";
    full_tfs_msg.transforms[2].transform.translation.x = 0.0;
    full_tfs_msg.transforms[2].transform.translation.y = 0.0;
    full_tfs_msg.transforms[2].transform.translation.z = 0.0;
    full_tfs_msg.transforms[2].transform.rotation.x = 0.0;
    full_tfs_msg.transforms[2].transform.rotation.y = 0.0;
    full_tfs_msg.transforms[2].transform.rotation.z = 0.0;
    full_tfs_msg.transforms[2].transform.rotation.w = 1.0;        

    noodom_tfs_msg.transforms.resize(2);
    noodom_tfs_msg.transforms[0] = full_tfs_msg.transforms[0];
    noodom_tfs_msg.transforms[1] = full_tfs_msg.transforms[1];

    t_last_map = ros::Time::now();

    ros::spin();

    return 0;
};