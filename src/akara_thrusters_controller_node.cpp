#include <ros/ros.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>

#include <geometry_msgs/Wrench.h>
#include <akara_msgs/Thruster.h>

using Eigen::MatrixXf;
using Eigen::VectorXf;

typedef Eigen::Matrix<float,6,1> Vector6f;

class ThrusterController
{
public:
  ThrusterController(ros::NodeHandle& nh)
  {
    ros::NodeHandle pnh("~");

    if (pnh.hasParam("TAM"))
    {
      std::vector<float> t;
      pnh.getParam("TAM", t);
      vectorToEigenMatrix_(t, tam_);
    }
    else
    {
      /*      x       y       z       r       p       y      */
      tam_ << 0.0000, 0.0000, 1.0000, 0.0000, 0.5000, 0.0000, // nosovaya_podrulka_vertikalnaya
              0.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.5000, // nosovaya_podrulka_gorizontalnaya
              0.0000, 1.0000, 0.0000, 0.0000, 0.0000,-0.5000, // hvostovaya_podrulka_gorizontalnaya
              0.0000, 0.0000, 1.0000, 0.0000,-0.5000, 0.0000, // hvostovaya_podrulka_vertikalnaya
              1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, // hvost_leviy
              1.0000, 0.0000, 0.0000,-0.0000, 0.0000, 0.0000; // hvost_praviy
    }

    ROS_INFO_STREAM("Thrusters configuration:\n" << tam_);

    if (pnh.hasParam("K"))
    {
      std::vector<float> coeffs;
      pnh.getParam("K", coeffs);

      VectorXf K;
      vectorToEigenVector_(coeffs, K);
      tam_ = K.asDiagonal() * tam_;
      ROS_INFO_STREAM("Coefficients: " << K.transpose());
    }

    thruster_msg_.power.resize(tam_.rows());

    wrench_sub_ = nh.subscribe<geometry_msgs::Wrench>(
          "command", 1, &ThrusterController::wrenchCallback, this);
    thrusters_pub_ = nh.advertise<akara_msgs::Thruster>("thrusters", 1);
  }

  void wrenchCallback(const geometry_msgs::WrenchConstPtr& msg)
  {
    wrenchMsgToEigen_(*msg, wrench_);
    eigenVectorToVector_(tam_ * wrench_, thruster_msg_.power);
    thrusters_pub_.publish(thruster_msg_);
  }

private:
  void wrenchMsgToEigen_(const geometry_msgs::Wrench& msg, Vector6f& wrench)
  {
    wrench[0] = msg.force.x;
    wrench[1] = msg.force.y;
    wrench[2] = msg.force.z;
    wrench[3] = msg.torque.x;
    wrench[4] = msg.torque.y;
    wrench[5] = msg.torque.z;
  }

  void vectorToEigenMatrix_(const std::vector<float>& v, MatrixXf& m)
  {
    m.resize(6, v.size()/6);
    for (int i = 0; i < v.size(); ++i)
      m(i) = v[i];
    m.transposeInPlace();
  }

  void vectorToEigenVector_(const std::vector<float>& v, VectorXf& ev)
  {
    ev.resize(v.size());
    for (int i = 0; i < v.size(); ++i)
      ev(i) = v[i];
  }

  void eigenVectorToVector_(const VectorXf& ev, std::vector<float>& v)
  {
    for (int i = 0; i < v.size(); ++i)
      v[i] = ev(i);
  }

  ros::Subscriber wrench_sub_;
  ros::Publisher thrusters_pub_;

  akara_msgs::Thruster thruster_msg_;

  Vector6f wrench_;
  MatrixXf tam_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "akara_thrusters_controller_node");
  ros::NodeHandle nh;
  ThrusterController t(nh);
  ros::spin();
}
