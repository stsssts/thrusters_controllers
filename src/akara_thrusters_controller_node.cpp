#include <ros/ros.h>

#include <eigen3/Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

#include  <geometry_msgs/Wrench.h>
#include "akara_msgs/Thruster.h"

typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,6,1> Vector6d;

class ThrusterController
{
public:
  ThrusterController(ros::NodeHandle& nh)
  {
    tam_ << 0.0000, 0.0000, 0.4048, 0.0064,-0.0018, 0.0000,
            0.0000, 0.0000, 0.4048,-0.0064,-0.0018, 0.0000,
            0.5000,-1.9318, 0.0000, 0.0245, 0.0000,-0.0045,
            0.5000, 1.9318,-0.0000,-0.0245, 0.0000, 0.0045,
            0.0000, 0.0000, 0.4275, 0.0000, 0.0025, 0.0000,
            0.0000, 1.0000,-0.0000,-0.0091, 0.0000, 0.0000;

    wrench_sub_ = nh.subscribe<geometry_msgs::Wrench>(
          "command", 1, &ThrusterController::wrenchCallback, this);
    thrusters_pub_ = nh.advertise<akara_msgs::Thruster>("thrusters", 1);
  }

  void wrenchCallback(const geometry_msgs::WrenchConstPtr& msg)
  {
    akara_msgs::Thruster thruster_msg;
    tf::wrenchMsgToEigen(*msg, wrench_);
    Vector6d thrust = tam_ * wrench_;
    for(int i = 0; i < thrust.rows(); ++i)
      thruster_msg.power.push_back(thrust(i));

    thrusters_pub_.publish(thruster_msg);
  }

  ros::Subscriber wrench_sub_;
  ros::Publisher thrusters_pub_;

  Vector6d wrench_;
  Matrix6d tam_;
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "akara_thrusters_controller_node");
  ros::NodeHandle nh;
  ThrusterController t(nh);
  ros::spin();
}
