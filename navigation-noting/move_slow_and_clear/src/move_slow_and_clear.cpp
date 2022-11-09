#include <move_slow_and_clear/move_slow_and_clear.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/obstacle_layer.h>

//￥ 加載插件
PLUGINLIB_EXPORT_CLASS(move_slow_and_clear::MoveSlowAndClear, nav_core::RecoveryBehavior)

namespace move_slow_and_clear
{
  //初始化構造函數
  MoveSlowAndClear::MoveSlowAndClear():global_costmap_(NULL), local_costmap_(NULL), 
                                       initialized_(false), remove_limit_thread_(NULL), limit_set_(false){}

  MoveSlowAndClear::~MoveSlowAndClear()
  {
    delete remove_limit_thread_;
  }

  void MoveSlowAndClear::initialize (std::string n, tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* global_costmap,
      costmap_2d::Costmap2DROS* local_costmap)
  {

    //￥ 給指針分配空間
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //￥ 加載 parameter參數
    ros::NodeHandle private_nh_("~/" + n);
    private_nh_.param("clearing_distance", clearing_distance_, 0.5);
    private_nh_.param("limited_trans_speed", limited_trans_speed_, 0.25);
    private_nh_.param("limited_rot_speed", limited_rot_speed_, 0.45);
    private_nh_.param("limited_distance", limited_distance_, 0.3);
    private_nh_.param("max_trans_param_name", max_trans_param_name_, std::string("max_trans_vel"));
    private_nh_.param("max_rot_param_name", max_rot_param_name_, std::string("max_rot_vel"));

    std::string planner_namespace;
    private_nh_.param("planner_namespace", planner_namespace, std::string("DWAPlannerROS"));
    planner_nh_ = ros::NodeHandle("~/" + planner_namespace);
    planner_dynamic_reconfigure_service_ = planner_nh_.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters", true);
    initialized_ = true;
  }

  //￥ 運行
  void MoveSlowAndClear::runBehavior()
  {
    if(!initialized_)
    {
      ROS_ERROR("This recovery behavior has not been initialized, doing nothing.");
      return;
    }
    ROS_WARN("Move slow and clear recovery behavior started.");
    //￥ 初始化兩個坐標點
    geometry_msgs::PoseStamped global_pose, local_pose;

    global_costmap_->getRobotPose(global_pose);
    local_costmap_->getRobotPose(local_pose);
    //￥ 創建兩個vector向量類型數據
    std::vector<geometry_msgs::Point> global_poly, local_poly;
    geometry_msgs::Point pt;

    //￥ 在這裏，通過兩次計算來更改 global坐標
    for(int i = -1; i <= 1; i+=2)
    {
      pt.x = global_pose.pose.position.x + i * clearing_distance_;
      pt.y = global_pose.pose.position.y + i * clearing_distance_;
      global_poly.push_back(pt);

      pt.x = global_pose.pose.position.x + i * clearing_distance_;
      pt.y = global_pose.pose.position.y + -1.0 * i * clearing_distance_;
      global_poly.push_back(pt);

      pt.x = local_pose.pose.position.x + i * clearing_distance_;
      pt.y = local_pose.pose.position.y + i * clearing_distance_;
      local_poly.push_back(pt);

      pt.x = local_pose.pose.position.x + i * clearing_distance_;
      pt.y = local_pose.pose.position.y + -1.0 * i * clearing_distance_;
      local_poly.push_back(pt);
    }

    //clear the desired space in the costmaps
    std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = global_costmap_->getLayeredCostmap()->getPlugins();
    for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
            boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
          if(plugin->getName().find("obstacles")!=std::string::npos){
            boost::shared_ptr<costmap_2d::ObstacleLayer> costmap;
            costmap = boost::static_pointer_cast<costmap_2d::ObstacleLayer>(plugin);
            costmap->setConvexPolygonCost(global_poly, costmap_2d::FREE_SPACE);
          }
    }
     
    plugins = local_costmap_->getLayeredCostmap()->getPlugins();
    for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
            boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
          if(plugin->getName().find("obstacles")!=std::string::npos){
            boost::shared_ptr<costmap_2d::ObstacleLayer> costmap;
            costmap = boost::static_pointer_cast<costmap_2d::ObstacleLayer>(plugin);
            costmap->setConvexPolygonCost(local_poly, costmap_2d::FREE_SPACE);
          }
    } 

    //lock... just in case we're already speed limited
    boost::mutex::scoped_lock l(mutex_);

    //get the old maximum speed for the robot... we'll want to set it back
    //￥ 配置线速度与角速度的限制。
    if(!limit_set_)
    {
      if(!planner_nh_.getParam(max_trans_param_name_, old_trans_speed_))
      {
        ROS_ERROR("The planner %s, does not have the parameter %s", planner_nh_.getNamespace().c_str(), max_trans_param_name_.c_str());
      }

      if(!planner_nh_.getParam(max_rot_param_name_, old_rot_speed_))
      {
        ROS_ERROR("The planner %s, does not have the parameter %s", planner_nh_.getNamespace().c_str(), max_rot_param_name_.c_str());
      }
    }

    //我们还想保存我们当前的位置，以便我们可以取消我们稍后施加的速度限制
    speed_limit_pose_ = global_pose;

    //限制机器人的速度，直到它移动一定距离
    setRobotSpeed(limited_trans_speed_, limited_rot_speed_);
    limit_set_ = true;
    distance_check_timer_ = private_nh_.createTimer(ros::Duration(0.1), &MoveSlowAndClear::distanceCheck, this);
  }

  double MoveSlowAndClear::getSqDistance()
  {
    //创建一个坐标，并将机器人相对于全局坐标系的坐标保存下来
    geometry_msgs::PoseStamped global_pose;
    global_costmap_->getRobotPose(global_pose);

    double x1 = global_pose.pose.position.x;
    double y1 = global_pose.pose.position.y;

    double x2 = speed_limit_pose_.pose.position.x;
    double y2 = speed_limit_pose_.pose.position.y;

    return (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
  }

  //机器人移动距离判断
  void MoveSlowAndClear::distanceCheck(const ros::TimerEvent& e)
  {
    if(limited_distance_ * limited_distance_ <= getSqDistance())
    {
      ROS_INFO("Moved far enough, removing speed limit.");
      //have to do this because a system call within a timer cb does not seem to play nice
      if(remove_limit_thread_)
      {
        remove_limit_thread_->join();
        delete remove_limit_thread_;
      }
      remove_limit_thread_ = new boost::thread(boost::bind(&MoveSlowAndClear::removeSpeedLimit, this));

      distance_check_timer_.stop();
    }
  }

  void MoveSlowAndClear::removeSpeedLimit()
  {
    boost::mutex::scoped_lock l(mutex_);
    setRobotSpeed(old_trans_speed_, old_rot_speed_);
    limit_set_ = false;
  }

  void MoveSlowAndClear::setRobotSpeed(double trans_speed, double rot_speed)
  {

    {
      dynamic_reconfigure::Reconfigure vel_reconfigure;
      dynamic_reconfigure::DoubleParameter new_trans;
      new_trans.name = max_trans_param_name_;
      new_trans.value = trans_speed;
      vel_reconfigure.request.config.doubles.push_back(new_trans);
      try {
        planner_dynamic_reconfigure_service_.call(vel_reconfigure);
        ROS_INFO_STREAM("Recovery setting trans vel: " << trans_speed);
      }
      catch(...) {
        ROS_ERROR("Something went wrong in the service call to dynamic_reconfigure");
      }
    }
    {
      dynamic_reconfigure::Reconfigure rot_reconfigure;
      dynamic_reconfigure::DoubleParameter new_rot;
      new_rot.name = max_rot_param_name_;
      new_rot.value = rot_speed;
      rot_reconfigure.request.config.doubles.push_back(new_rot);
      try {
        planner_dynamic_reconfigure_service_.call(rot_reconfigure);
        ROS_INFO_STREAM("Recovery setting rot vel: " << rot_speed);
      }
      catch(...) {
        ROS_ERROR("Something went wrong in the service call to dynamic_reconfigure");
      }
    }
  }
};
