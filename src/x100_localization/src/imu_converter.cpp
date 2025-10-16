#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>


double roll = 0.0, pitch = 0.0, yaw = 0.0;


ros::Publisher imu_pub;


void imuCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    roll = msg->x;
    pitch = msg->y;
    yaw = msg->z;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_tf_broadcaster");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("RPY", 10, imuCallback);


    imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 10);

    tf::TransformBroadcaster br;
    ros::Rate rate(50);

    while (ros::ok()) {
        tf::Transform transform;
        tf::Quaternion q;
        q.setRPY(roll, pitch, yaw);

        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        transform.setRotation(q);

       
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "imu_link"));

     
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu_link";

        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();

        
        imu_msg.orientation_covariance[0] = -1; 

        
        imu_pub.publish(imu_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}