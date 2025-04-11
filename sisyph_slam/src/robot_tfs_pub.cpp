#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>





int main(int argc, char **argv)
{
    ros::init(argc,argv, "odom_tf_pub");
    ros::NodeHandle nh;

    tf2_msgs::TFMessage full_tfs_msg;
    tf2_msgs::TFMessage noodom_tfs_msg;

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


    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::TransformStamped got_tf;
    ros::Time t_last_map=ros::Time::now();
    ros::Time t_now = ros::Time::now();
    double map_too_old_dt_secs = 10.0;
    double max_time=0.0;
    bool noodom = false;

    ros::Publisher tf_pub = nh.advertise<tf2_msgs::TFMessage>("/tf", 20, false);

    auto pos_vector_cb = [&](const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
        // double dt_time = abs(ros::Time::now().toSec()-msg->header.stamp.toSec());
        // max_time = dt_time>max_time ? dt_time : max_time;
        // ROS_INFO_STREAM("dt_time: "<<ros::Time::now().toSec()-msg->header.stamp.toSec()<<" max_time: "<<max_time);

        t_now = ros::Time::now(); 
        // if(msg->header.stamp<t_now){ t_now = msg->header.stamp; }
        full_tfs_msg.transforms[0].header.stamp = t_now;
        full_tfs_msg.transforms[0].transform.translation.x = msg->vector.x;
        full_tfs_msg.transforms[0].transform.translation.y = msg->vector.y;
        full_tfs_msg.transforms[0].transform.rotation.z = sin(msg->vector.z/2);
        full_tfs_msg.transforms[0].transform.rotation.w = cos(msg->vector.z/2);
    
        full_tfs_msg.transforms[1].header.stamp = t_now;
        full_tfs_msg.transforms[2].header.stamp = t_now;
    
        try
        {
            got_tf = tf_buffer.lookupTransform("odom","map",ros::Time(0.0));
            if(    abs(got_tf.transform.translation.x-full_tfs_msg.transforms[2].transform.translation.x)>0.005
                || abs(got_tf.transform.translation.y-full_tfs_msg.transforms[2].transform.translation.y)>0.005
                      || abs(got_tf.transform.rotation.z-full_tfs_msg.transforms[2].transform.rotation.z)>0.005)
            {
                t_last_map = ros::Time::now();
                full_tfs_msg.transforms[2].transform.translation.x = got_tf.transform.translation.x;
                full_tfs_msg.transforms[2].transform.translation.y = got_tf.transform.translation.y;
                full_tfs_msg.transforms[2].transform.rotation.z = got_tf.transform.rotation.z;
                full_tfs_msg.transforms[2].transform.rotation.w = got_tf.transform.rotation.w;
            }
        }
        catch(const tf2::TransformException& ex)
        {
            ROS_WARN_ONCE("%s",ex.what());
        }
        
        bool do_got_tf = false;
        if((ros::Time::now().toSec()-t_last_map.toSec())>map_too_old_dt_secs)
        {
            do_got_tf = false;
        }else{
            do_got_tf = true;
        }
    
        if(!do_got_tf)
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
    };

    ros::Subscriber pos_vector_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/sisyph/odom/pos", 100, pos_vector_cb);

    ros::spin();

    return 0;
};