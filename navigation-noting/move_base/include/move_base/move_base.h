#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.hpp>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"

namespace move_base {
  //typedefs to help us out with the action server so that we don't hace to type so much
  //!action服务器
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

  //movabase标志
  enum MoveBaseState {
    PLANNING,//正处于规划路径
    CONTROLLING,//正在控制运动
    CLEARING//处于恢复或者清除状态，规划失败或控制运动失败。
  };

  //恢复触发器
  enum RecoveryTrigger
  {
    PLANNING_R,//全局规划失败
    CONTROLLING_R,//局部轨迹规划失败
    OSCILLATION_R//长时间按小范围晃动
  };

  /**
   * @class MoveBase
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
  class MoveBase {
    public:
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
      MoveBase(tf2_ros::Buffer& tf);

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveBase();

      /**
       * @brief 机器人导航到目标点的控制函数，返回值true到达，反着则false
       * @param goal A reference to the goal to pursue
       * @return True if processing of the goal is done, false otherwise
       */
      bool executeCycle(geometry_msgs::PoseStamped& goal);

    private:
      /**
       * @brief  代价地图清除服务
       * @param req The service request 
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
      bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

      /**
       * @brief  movebase类的makePlan成员函数，注意与nav_core中的进行区分，返回true表示规划成功
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  加载恢复行为函数，返回true表示成功加载恢复行为。
       * @param node The ros::NodeHandle to be used for loading parameters 
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
      bool loadRecoveryBehaviors(ros::NodeHandle node);

      /**
       * @brief  加载默认恢复行为
       */
      void loadDefaultRecoveryBehaviors();

      /**
       * @brief  清除机器人周围窗口内的障碍物。
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * @brief  发布零速度函数，禁止机器人移动。
       */
      void publishZeroVelocity();

      /**
       * @brief  恢复movebase状态，同时发布零速度，禁止机器人移动
       */
      void resetState();

      /**
       * @brief 接受目标回调函数
       * 
       * @param goal 
       */
      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

      /**
       * @brief 开辟全局规划线程，进行机器人全局路径规划
       * 
       */
      void planThread();

      /**
       * @brief 控制机器人地盘的主要函数
       * 
       * @param move_base_goal 
       */
      void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

     /**
     * @brief 四元数检查函数，检查四元数是否合法
     * 
     * @param q 
     * @return true 
     * @return false 
     */
      bool isQuaternionValid(const geometry_msgs::Quaternion& q);

      /**
       * @brief 返回机器人的位置
       * 
       * @param global_pose 
       * @param costmap 
       * @return true 
       * @return false 
       */
      bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);

      /**
       * @brief 机器人运动距离计算，和参数oscillation_distance_进行比较
       * 
       * @param p1 
       * @param p2 
       * @return double 
       */
      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      /**
       * @brief 将目标电转化到全局坐标系下
       * 
       * @param goal_pose_msg 
       * @return geometry_msgs::PoseStamped 
       */
      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

      /**
       * @brief This is used to wake the planner at periodic intervals.
       */
      void wakePlanner(const ros::TimerEvent& event);

      /**
       * @brief TF坐标系的转换类 指针
       * 
       */
      tf2_ros::Buffer& tf_;
      /**
       * @brief 该指针指向导航的服务类
       * 
       */
      MoveBaseActionServer* as_;
      /**
       * @brief 局部路径规划器实例后的指针，指向local_planner
       * 
       */
      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
      /**
       * @brief 代价地图指针。planner_costmap_ros_指向全局代价地图；controller_costmap_ros_指向局部代价地图
       * 
       */
      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;

      /**
       * @brief 全局规划器指针
       * 
       */
      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
      std::string robot_base_frame_, global_frame_;

      /**
       * @brief 恢复行为vector，储存全部的恢复行为，默认转圈。
       * 
       */
      std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
      std::vector<std::string> recovery_behavior_names_;
      unsigned int recovery_index_;

      geometry_msgs::PoseStamped global_pose_;
      double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
      double planner_patience_, controller_patience_;
      int32_t max_planning_retries_;
      uint32_t planning_retries_;
      double conservative_reset_dist_, clearing_radius_;
      ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_, recovery_status_pub_;
      ros::Subscriber goal_sub_;
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;

      bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
      bool make_plan_clear_costmap_, make_plan_add_unreachable_goal_;

      double oscillation_timeout_, oscillation_distance_;

      MoveBaseState state_;//存放movebase的状态标志
      RecoveryTrigger recovery_trigger_;//存放movebase的恢复标志

      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
      geometry_msgs::PoseStamped oscillation_pose_;

      /**
       * @brief 一插件的形式实现全局规划器，局部规划期和回复行为，插件可以动态的加载c++类。
       * 
       */
      
      //全局规划器
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
      //局部规划器
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
      //回复行为规划器
      pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

      /**
       * @brief 通过容器保存刚刚计算出的路径，planner_plan_传给latest_plan_，作为一个桥梁，在executeCycle函数中传递给controller_plan_
       * 
       */
      
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;//存放当前的全局路径信息
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;//记录最新的全局路径信息
      std::vector<geometry_msgs::PoseStamped>* controller_plan_;//存放用于局部规划的路径

      //路径规划器标志位
      bool runPlanner_;
      
      //线程锁
      boost::recursive_mutex planner_mutex_;
      //线程锁
      boost::condition_variable_any planner_cond_;

      geometry_msgs::PoseStamped planner_goal_;
      boost::thread* planner_thread_;


      boost::recursive_mutex configuration_mutex_;
      dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;
      
      void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);

      move_base::MoveBaseConfig last_config_;
      move_base::MoveBaseConfig default_config_;
      bool setup_, p_freq_change_, c_freq_change_;
      bool new_global_plan_;
  };
};
#endif

