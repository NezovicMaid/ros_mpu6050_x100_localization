#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point.h"
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class SquareController
{
public:
    SquareController()
    {
        nh_ = ros::NodeHandle();
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        pose_sub_ = nh_.subscribe("/pose_with_covariance_stamped", 10, &SquareController::poseCallback, this);

        
        waypoints_.push_back(create_point(1.5, 0.0));
        waypoints_.push_back(create_point(1.5, -1.5));
        waypoints_.push_back(create_point(0.0, -1.5));
        waypoints_.push_back(create_point(0.0, 0.0));

        current_waypoint_index_ = 0;
        robot_stopped_ = false;

        
        nh_.param("linear_speed", linear_speed_, 0.3);
        nh_.param("angular_gain", angular_gain_, 1.5);
        nh_.param("distance_tolerance", distance_tolerance_, 0.1);
        nh_.param("turn_only_threshold", turn_only_threshold_, 0.2);

        ROS_INFO("Square Controller (kontinuirani nacin) inicijaliziran.");
        ROS_INFO("Parametri: linearna brzina=%.2f, pojacanje kuta=%.2f", linear_speed_, angular_gain_);
    }

private:
    geometry_msgs::Point create_point(double x, double y) {
        geometry_msgs::Point p;
        p.x = x; p.y = y; p.z = 0;
        return p;
    }

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        if (current_waypoint_index_ >= waypoints_.size()) {
            if (!robot_stopped_) {
                geometry_msgs::Twist stop_msg;
                stop_msg.linear.x = 0;
                stop_msg.angular.z = 0;
                cmd_vel_pub_.publish(stop_msg);
                ROS_INFO("Svi ciljevi obidjeni. Robot zaustavljen.");
                robot_stopped_ = true;
            }
            return;
        }

        geometry_msgs::Point current_target = waypoints_[current_waypoint_index_];
        double x_cilj = current_target.x;
        double y_cilj = current_target.y;

        double x_robot = msg->pose.pose.position.x;
        double y_robot = msg->pose.pose.position.y;

        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        double FI_robot = yaw;
        double FI_cilj = atan2(y_cilj - y_robot, x_cilj - x_robot);
        double ugao_razlika = atan2(sin(FI_cilj - FI_robot), cos(FI_cilj - FI_robot));
        double udaljenost = hypot(x_cilj - x_robot, y_cilj - y_robot);

        geometry_msgs::Twist poruka;

        if (udaljenost < distance_tolerance_) {
            ROS_INFO("Stigao na cilj %zu (%.2f, %.2f).", current_waypoint_index_ + 1, x_cilj, y_cilj);
            current_waypoint_index_++;
            poruka.linear.x = 0.0;
            poruka.angular.z = 0.0;
            cmd_vel_pub_.publish(poruka);
            ros::Duration(0.5).sleep();
            return;
        }

        if (fabs(ugao_razlika) > turn_only_threshold_) {
            ROS_INFO_THROTTLE(0.5, "STANJE: OKRETANJE U MJESTU | Greska kuta: %.2f rad", ugao_razlika);
            poruka.linear.x = 0.0;
            poruka.angular.z = angular_gain_ * ugao_razlika;
            if (poruka.angular.z > 0.5) poruka.angular.z = 0.5;
            if (poruka.angular.z < -0.5) poruka.angular.z = -0.5;
        }
        else {
            ROS_INFO_THROTTLE(0.5, "STANJE: VOZNJA I ISPRAVLJANJE | Udalj: %.2f m | Greska kuta: %.2f rad", udaljenost, ugao_razlika);
            poruka.linear.x = linear_speed_;
            poruka.angular.z = angular_gain_ * ugao_razlika;
        }

        cmd_vel_pub_.publish(poruka);
    }

    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber pose_sub_;
    std::vector<geometry_msgs::Point> waypoints_;
    size_t current_waypoint_index_;
    bool robot_stopped_;

    double linear_speed_;
    double angular_gain_;
    double distance_tolerance_;
    double turn_only_threshold_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "square_controller_node");
    SquareController controller;
    ros::spin();
    return 0;
}