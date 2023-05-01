#pragma once

#include "mj_sim_impl.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
// #include <eigen_conversions/eigen_msg.h>

namespace
{
void pointEigenToMsg(const Eigen::Vector3d & v, geometry_msgs::Point & msg)
{
  msg.x = v.x();
  msg.y = v.y();
  msg.z = v.z();
}

void quaternionEigenToMsg(const Eigen::Quaterniond & v, geometry_msgs::Quaternion & msg)
{
  msg.w = v.w();
  msg.x = v.x();
  msg.y = v.y();
  msg.z = v.z();
}

void vectorEigenToMsg(const Eigen::Vector3d & v, geometry_msgs::Vector3 & msg)
{
  msg.x = v.x();
  msg.y = v.y();
  msg.z = v.z();
}
} // namespace

namespace mc_mujoco
{
class PosePublisher
{
public:
  struct Configuration
  {
    std::string frame_id = "";
    std::string pose_topic_name = "";
    std::string vel_topic_name = "";
    double pub_rate = 30.0;

    void setup(const std::string & mj_robot_name)
    {
      if(pose_topic_name.empty())
      {
        pose_topic_name = "mujoco/" + mj_robot_name + "/pose";
      }
      if(vel_topic_name.empty())
      {
        vel_topic_name = "mujoco/" + mj_robot_name + "/vel";
      }
      if(frame_id.empty())
      {
        frame_id = "robot_map";
      }
    }
  };

public:
  PosePublisher(const MjRobot * mj_robot, double sim_timestep, const Configuration & config)
  : mj_robot_(mj_robot), config_(config)
  {
    config_.setup(mj_robot_->name);

    if(!ros::isInitialized())
    {
      int argc = 0;
      char ** argv;
      ros::init(argc, argv, "mujoco_ros");
    }

    nh_ = std::make_shared<ros::NodeHandle>();
    pose_pub_ = nh_->advertise<geometry_msgs::PoseStamped>(config_.pose_topic_name, 1);
    vel_pub_ = nh_->advertise<geometry_msgs::TwistStamped>(config_.vel_topic_name, 1);
    pub_skip_ = std::max(static_cast<int>(1.0 / (config_.pub_rate * sim_timestep)), 1);
  }

  void update()
  {
    sim_cnt_++;
    if(sim_cnt_ % pub_skip_ != 0)
    {
      return;
    }

    ros::Time stamp_now = ros::Time::now();

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp_now;
    pose_msg.header.frame_id = config_.frame_id;
    pointEigenToMsg(mj_robot_->root_pos, pose_msg.pose.position);
    quaternionEigenToMsg(mj_robot_->root_ori.inverse(), pose_msg.pose.orientation);
    pose_pub_.publish(pose_msg);

    geometry_msgs::TwistStamped vel_msg;
    vel_msg.header.stamp = stamp_now;
    vel_msg.header.frame_id = config_.frame_id;
    vectorEigenToMsg(mj_robot_->root_linvel, vel_msg.twist.linear);
    vectorEigenToMsg(mj_robot_->root_angvel, vel_msg.twist.angular);
    vel_pub_.publish(vel_msg);
  }

public:
  Configuration config_;

protected:
  const std::unique_ptr<const MjRobot> mj_robot_;

  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher pose_pub_;
  ros::Publisher vel_pub_;

  int sim_cnt_ = 0;
  int pub_skip_ = 0;
};
} // namespace mc_mujoco
