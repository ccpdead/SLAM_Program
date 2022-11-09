#ifndef MOVE_SLOW_AND_CLEAR_MOVE_SLOW_AND_CLEAR_H_
#define MOVE_SLOW_AND_CLEAR_MOVE_SLOW_AND_CLEAR_H_

#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <boost/thread.hpp>
#include <dynamic_reconfigure/Reconfigure.h>

namespace move_slow_and_clear 
{
  class MoveSlowAndClear : public nav_core::RecoveryBehavior
  {
    public:
      MoveSlowAndClear();
      ~MoveSlowAndClear();

      /// Initialize the parameters of the behavior
      void initialize (std::string n, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* global_costmap,
          costmap_2d::Costmap2DROS* local_costmap);

      /// Run the behavior
      void runBehavior();

    private:
      void setRobotSpeed(double trans_speed, double rot_speed);
      void distanceCheck(const ros::TimerEvent& e);
      double getSqDistance();

      void removeSpeedLimit();

      ros::NodeHandle private_nh_, planner_nh_;
      costmap_2d::Costmap2DROS* global_costmap_;
      costmap_2d::Costmap2DROS* local_costmap_;
      bool initialized_;
      double clearing_distance_, limited_distance_;
      double limited_trans_speed_, limited_rot_speed_, old_trans_speed_, old_rot_speed_;
      std::string max_trans_param_name_, max_rot_param_name_;
      ros::Timer distance_check_timer_;
      geometry_msgs::PoseStamped speed_limit_pose_;
      boost::thread* remove_limit_thread_;
      boost::mutex mutex_;
      bool limit_set_;
      ros::ServiceClient planner_dynamic_reconfigure_service_;
  };
};

#endif
