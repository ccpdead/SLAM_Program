#include <move_base/move_base.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//!
//@
//^
//$
//%

namespace move_base
{

    /* –目录–
    构造函数 MoveBase::MoveBase | 初始化Action
    控制主体 MoveBase::executeCb | 收到目标，触发全局规划线程，循环执行局部规划
    全局规划线程 void MoveBase::planThread | 调用全局规划
    全局规划 MoveBase::makePlan | 调用全局规划器类方法，得到全局规划路线
    局部规划 MoveBase::executeCycle | 传入全局路线，调用局部规划器类方法，得到速度控制指令 */

    //  <1> 构造函数 MoveBase::MoveBase

    //! MoveBase类的构造函数进行了初始化工作，获取了服务器上的参数值。
    MoveBase::MoveBase(tf2_ros::Buffer &tf) : tf_(tf),
                                              as_(NULL),
                                              planner_costmap_ros_(NULL),
                                              controller_costmap_ros_(NULL),
                                              bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
                                              blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
                                              recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
                                              planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
                                              runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false)

    {
        //实例化一个对象，一直执行回调函数executeCb
        // as_是一个action服务器实例化后的指针，当执行as_->start（）时调用MoveBase::executeCb函数
        as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

        ros::NodeHandle private_nh("~");
        ros::NodeHandle nh;
        recovery_trigger_ = PLANNING_R;

        //$ 加载yaml参数，做没有，则按照默认参数
        std::string global_planner, local_planner;
        //全局规划器
        private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
        //局部规划器
        private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
        // costmap地图的机器人坐标系
        private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
        // costmap地图的全局参考坐标系
        private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
        // planner的规划频率
        private_nh.param("planner_frequency", planner_frequency_, 0.0);
        // cmd_vel的计算频率
        private_nh.param("controller_frequency", controller_frequency_, 20.0);
        private_nh.param("planner_patience", planner_patience_, 5.0);
        private_nh.param("controller_patience", controller_patience_, 15.0);
        //最大规划次数
        private_nh.param("max_planning_retries", max_planning_retries_, -1); // disabled by default
        private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
        private_nh.param("oscillation_distance", oscillation_distance_, 0.5);
        private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
        private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

        //$ 初始化三个plan的缓冲池数组
        planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        //$ 新建planner线程，入口函数planThread
        planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

        //$ vel_pub_速度下发命令，将速度发送给机器人地盘
        //$ current_goal_pub_当前目标发布指令，发布目标点
        vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0);

        //$ 动作行为发布，目标点以及恢复行为的信息
        ros::NodeHandle action_nh("move_base");
        action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
        recovery_status_pub_ = action_nh.advertise<move_base_msgs::RecoveryStatus>("recovery_status", 1);

        //$ 订阅rviz下发的move_base_simple中的goal话题，获取目标点
        ros::NodeHandle simple_nh("move_base_simple");
        // movebase导航目标接收动作服务器，接收到导航目标后，进入到goalCB函数发送导航目标
        goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

        //内接半径
        private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
        //外接半径
        private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
        //代价地图清理半径
        private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
        //保守复位区
        private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);
        //设置是否在movebase规划失败后关闭costmap服务
        private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
        private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
        //启用恢复行为
        private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

        //^初始化全局规划器，和局部规划器的指针和各自的costmap，这里的两个规划器用到的地图实质上是对Costmap2DROS类的实例，
        //^这个类Ros对costmap的封装，类函数start（）回调用各层地图的active（）函数， 开始订阅传感器的话题。

        //! 全局规划器代价地图goal_costmap_ros_
        planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
        planner_costmap_ros_->pause();
        //$ 初始化全局规划器，planner_指针。
        try
        {
            planner_ = bgp_loader_.createInstance(global_planner);
            planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
        }
        catch (const pluginlib::PluginlibException &ex)
        {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
            exit(1);
        }

        //! 本地规划器代价地图controller_costmap_ros_
        controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
        controller_costmap_ros_->pause();
        //$ 初始化本地规划器，tc_指针。
        try
        {
            tc_ = blp_loader_.createInstance(local_planner);
            ROS_INFO("Created local_planner %s", local_planner.c_str());
            tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
        }
        catch (const pluginlib::PluginlibException &ex)
        {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
            exit(1);
        }

        //% 根据传感器数据动态更新全局和本地的代价地图。
        planner_costmap_ros_->start();
        controller_costmap_ros_->start();

        //这里创建一个make_plan的服务，当调用此服务时只规划全局路径，但不移动机器人。
        make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

        //发布一个清除costmap的服务，当调用此服务后，清除机器人全部的costmap（包括global，local）
        clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

        // if we shutdown our costmaps when we're deactivated... we'll do that now
        if (shutdown_costmaps_)
        {
            ROS_DEBUG_NAMED("move_base", "Stopping costmaps initially");
            planner_costmap_ros_->stop();
            controller_costmap_ros_->stop();
        }

        //% 若加载恢复行为没有参数，则加载默认的恢复行为。
        if (!loadRecoveryBehaviors(private_nh))
        {
            //$ 在loadDefaultRecoveryBehaviors函数中加载对应的恢复行为插件，清除代价地图的恢复行为，原地旋转的恢复行为，代价地图重置，再次原地旋转，加载错误报告等等等。。。
            loadDefaultRecoveryBehaviors();
        }

        // initially, we'll need to make a plan
        //初始阶段，我们将当前的状态设置为planning
        state_ = PLANNING;

        // we'll start executing recovery behaviors at the beginning of our list
        //定义恢复列表
        recovery_index_ = 0;

        //$ 执行movebase路径规划的动作服务器
        as_->start();

        //这里创建一个动态参数服务器，用于动态的更改movebase参数
        dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
        dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);

        //每次更新参数后，进入到movebzse：：reconfigureCB函数，从新配置movebase参数
        dsrv_->setCallback(cb);
    }
    //￥ reconfigureCB函数在每次参数被动态更改后进入。
    void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level)
    {
        boost::recursive_mutex::scoped_lock l(configuration_mutex_);

        // The first time we're called, we just want to make sure we have the
        // original configuration
        if (!setup_)
        {
            last_config_ = config;
            default_config_ = config;
            setup_ = true;
            return;
        }

        if (config.restore_defaults)
        {
            config = default_config_;
            // if someone sets restore defaults on the parameter server, prevent looping
            config.restore_defaults = false;
        }

        if (planner_frequency_ != config.planner_frequency)
        {
            planner_frequency_ = config.planner_frequency;
            p_freq_change_ = true;
        }

        if (controller_frequency_ != config.controller_frequency)
        {
            controller_frequency_ = config.controller_frequency;
            c_freq_change_ = true;
        }

        planner_patience_ = config.planner_patience;
        controller_patience_ = config.controller_patience;
        max_planning_retries_ = config.max_planning_retries;
        conservative_reset_dist_ = config.conservative_reset_dist;

        recovery_behavior_enabled_ = config.recovery_behavior_enabled;
        clearing_rotation_allowed_ = config.clearing_rotation_allowed;
        shutdown_costmaps_ = config.shutdown_costmaps;

        oscillation_timeout_ = config.oscillation_timeout;
        oscillation_distance_ = config.oscillation_distance;
        if (config.base_global_planner != last_config_.base_global_planner)
        {
            boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
            // initialize the global planner
            ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
            try
            {
                //$ 加载全局路径规划器
                planner_ = bgp_loader_.createInstance(config.base_global_planner);

                // wait for the current planner to finish planning
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

                // Clean up before initializing the new planner
                planner_plan_->clear();
                latest_plan_->clear();
                controller_plan_->clear();
                resetState();
                planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

                lock.unlock();
            }
            catch (const pluginlib::PluginlibException &ex)
            {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s",
                          config.base_global_planner.c_str(), ex.what());
                planner_ = old_planner;
                config.base_global_planner = last_config_.base_global_planner;
            }
        }

        if (config.base_local_planner != last_config_.base_local_planner)
        {
            // 4boost::shared_ptr系统智能指针
            boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
            // create a local planner
            try
            {
                //￥ 尝试进行局部路径规划
                tc_ = blp_loader_.createInstance(config.base_local_planner);
                // Clean up before initializing the new planner
                planner_plan_->clear();
                latest_plan_->clear();
                controller_plan_->clear();
                resetState();
                tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
            }
            catch (const pluginlib::PluginlibException &ex)
            {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s",
                          config.base_local_planner.c_str(), ex.what());
                tc_ = old_planner;
                config.base_local_planner = last_config_.base_local_planner;
            }
        }

        make_plan_clear_costmap_ = config.make_plan_clear_costmap;
        make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;

        last_config_ = config;
    }

    //%----------------------------------------------------------------
    //导航目标回调函数，接收到导航目标后，通过action服务器发布出去。
    void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goal)
    {
        ROS_DEBUG_NAMED("move_base", "In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
        move_base_msgs::MoveBaseActionGoal action_goal;
        action_goal.header.stamp = ros::Time::now();
        action_goal.goal.target_pose = *goal;

        action_goal_pub_.publish(action_goal);
    }

    //%----------------------------------------------------------------
    void MoveBase::clearCostmapWindows(double size_x, double size_y)
    {
        geometry_msgs::PoseStamped global_pose;

        // clear the planner's costmap
        getRobotPose(global_pose, planner_costmap_ros_);

        std::vector<geometry_msgs::Point> clear_poly;
        double x = global_pose.pose.position.x;
        double y = global_pose.pose.position.y;
        geometry_msgs::Point pt;

        pt.x = x - size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x - size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);
        //将一定范围的costmap空间设置为free——space形式。
        planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

        // clear the controller's costmap
        getRobotPose(global_pose, controller_costmap_ros_);

        clear_poly.clear();
        x = global_pose.pose.position.x;
        y = global_pose.pose.position.y;

        pt.x = x - size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x - size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
    }

    //%----------------------------------------------------------------
    // clear_costmap服务的回调函数，当调用clear_costmap指令后，进入到此函数中。
    bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
    {
        // clear the costmaps
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
        controller_costmap_ros_->resetLayers();

        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
        planner_costmap_ros_->resetLayers();
        return true;
    }
    //%----------------------------------------------------------------
    // make_plan的回调函数，调用此指令后，系统自动进行路径规划，但并不控制机器人前往。
    bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp)
    {
        if (as_->isActive())
        {
            ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
            return false;
        }
        // make sure we have a costmap for our planner
        if (planner_costmap_ros_ == NULL)
        {
            ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
            return false;
        }

        geometry_msgs::PoseStamped start;
        // if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
        if (req.start.header.frame_id.empty())
        {
            geometry_msgs::PoseStamped global_pose;
            if (!getRobotPose(global_pose, planner_costmap_ros_))
            {
                ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
                return false;
            }
            start = global_pose;
        }
        else
        {
            start = req.start;
        }

        if (make_plan_clear_costmap_)
        {
            // update the copy of the costmap the planner uses
            clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
        }

        // first try to make a plan to the exact desired goal
        std::vector<geometry_msgs::PoseStamped> global_plan;
        if (!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty())
        {
            ROS_DEBUG_NAMED("move_base", "Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
                            req.goal.pose.position.x, req.goal.pose.position.y);

            // search outwards for a feasible goal within the specified tolerance
            geometry_msgs::PoseStamped p;
            p = req.goal;
            bool found_legal = false;
            float resolution = planner_costmap_ros_->getCostmap()->getResolution();
            float search_increment = resolution * 3.0;
            if (req.tolerance > 0.0 && req.tolerance < search_increment)
                search_increment = req.tolerance;
            for (float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment)
            {
                for (float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment)
                {
                    for (float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment)
                    {

                        // don't search again inside the current outer layer
                        if (x_offset < max_offset - 1e-9 && y_offset < max_offset - 1e-9)
                            continue;

                        // search to both sides of the desired goal
                        for (float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0)
                        {

                            // if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
                            if (y_offset < 1e-9 && y_mult < -1.0 + 1e-9)
                                continue;

                            for (float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0)
                            {
                                if (x_offset < 1e-9 && x_mult < -1.0 + 1e-9)
                                    continue;

                                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                                if (planner_->makePlan(start, p, global_plan))
                                {
                                    if (!global_plan.empty())
                                    {

                                        if (make_plan_add_unreachable_goal_)
                                        {
                                            // adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                                            //(the reachable goal should have been added by the global planner)
                                            global_plan.push_back(req.goal);
                                        }

                                        found_legal = true;
                                        ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                                        break;
                                    }
                                }
                                else
                                {
                                    ROS_DEBUG_NAMED("move_base", "Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                                }
                            }
                        }
                    }
                }
            }
        }

        // copy the plan into a message to send out
        resp.plan.poses.resize(global_plan.size());
        for (unsigned int i = 0; i < global_plan.size(); ++i)
        {
            resp.plan.poses[i] = global_plan[i];
        }

        return true;
    }
    //%----------------------------------------------------------------
    // movebase析构函数
    MoveBase::~MoveBase()
    {
        recovery_behaviors_.clear();

        delete dsrv_;

        if (as_ != NULL)
            delete as_;

        if (planner_costmap_ros_ != NULL)
            delete planner_costmap_ros_;

        if (controller_costmap_ros_ != NULL)
            delete controller_costmap_ros_;

        planner_thread_->interrupt();
        planner_thread_->join();

        delete planner_thread_;

        delete planner_plan_;
        delete latest_plan_;
        delete controller_plan_;

        planner_.reset();
        tc_.reset();
    }
    //%----------------------------------------------------------------
    /* <4> 全局规划 MoveBase::makePlan
    该函数先进行一些预备工作，如检查全局代价地图、起始位姿，然后将起始位姿的数据格式做转换。 */
    bool MoveBase::makePlan(const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

        //初始化空
        plan.clear();

        //如果没有全局代价地图，返回false，因为全局规划必须基于全局代价地图。
        if (planner_costmap_ros_ == NULL)
        {
            ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
            return false;
        }

        //如果得不到机器人的起始位姿，返回false
        geometry_msgs::PoseStamped global_pose;
        if (!getRobotPose(global_pose, planner_costmap_ros_))
        {
            ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
            return false;
        }

        const geometry_msgs::PoseStamped &start = global_pose;
        /*
            接下来是实际进行全局规划的函数，调用全局规划器的makePlan函数planner_-> makePlan(start, goal, plan)，
            传入机器人当前位姿和目标，得到plan，若规划失败或得到的plan为空，返回false，否则返回true。 */

        //调用BaseGlobalPlanner类的makePlan函数做全局规划
        //如果全局规划失败，或者全局规划为空，则返回false
        if (!planner_->makePlan(start, goal, plan) || plan.empty())
        {
            ROS_DEBUG_NAMED("move_base", "Failed to find a  plan to point (%.2f, %.2f)",
                            goal.pose.position.x,
                            goal.pose.position.y);
            return false;
        }

        return true;
    }
    //%----------------------------------------------------------------
    //发布零速度，禁止机器人移动
    void MoveBase::publishZeroVelocity()
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_pub_.publish(cmd_vel);
    }
    //%----------------------------------------------------------------
    //验证四元数是否正确。
    bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion &q)
    {
        // first we need to check if the quaternion has nan's or infs
        if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
        {
            ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
            return false;
        }

        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

        // next, we need to check if the length of the quaternion is close to zero
        if (tf_q.length2() < 1e-6)
        {
            ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
            return false;
        }

        // next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
        tf_q.normalize();

        tf2::Vector3 up(0, 0, 1);

        double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

        if (fabs(dot - 1) > 1e-3)
        {
            ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
            return false;
        }

        return true;
    }

    //%----------------------------------------------------------------
    //返回给定坐标在全局坐标轴下的位置
    geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped &goal_pose_msg)
    {
        std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
        geometry_msgs::PoseStamped goal_pose, global_pose;
        goal_pose = goal_pose_msg;

        // just get the latest available transform... for accuracy they should send
        // goals in the frame of the planner
        goal_pose.header.stamp = ros::Time();

        try
        {
            tf_.transform(goal_pose_msg, global_pose, global_frame);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
                     goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
            return goal_pose_msg;
        }

        return global_pose;
    }
    //%----------------------------------------------------------------
    //？？？？？
    void MoveBase::wakePlanner(const ros::TimerEvent &event)
    {
        // we have slept long enough for rate
        planner_cond_.notify_one();
    }

    //%----------------------------------------------------------------
    /* <3> 全局规划线程 void MoveBase::planThread()
    planThread()的核心是调用makePlan函数，该函数中实际进行全局规划。全局规划线程时刻等待被executeCB函数唤醒，
    当executeCB函数中唤醒planThread并将标志位runPlanner_设置为真，跳出内部的循环，继续进行下面部分 */
    void MoveBase::planThread()
    {
        ROS_DEBUG_NAMED("move_base_plan_thread", "Starting planner thread...");
        ros::NodeHandle n;
        ros::Timer timer;

        //标志位为假，表示线程已经被唤醒。
        bool wait_for_wake = false;
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        while (n.ok())
        {
            //不断循环，直到 wait_for_wake 和 runPlanner为真，跳出循环。
            while (wait_for_wake || !runPlanner_)
            {
                //如果 waitforwake 是真，或者runplanner是假，不断执行循环，wait等待。
                ROS_DEBUG_NAMED("move_base_plan_thread", "Planner thread is suspending");
                //调用wait（）函数，函数自动调用lock.ublock()释放锁，使得其他被阻塞的线程得以执行。
                planner_cond_.wait(lock);
                wait_for_wake = false;
            }
            // start_time设为当前时间。
            ros::Time start_time = ros::Time::now();

            //把全局中被更新的全局目标planner_goal储存类临时目标。
            geometry_msgs::PoseStamped temp_goal = planner_goal_;
            lock.unlock(); //线程解锁
            ROS_DEBUG_NAMED("move_base_plan_thread", "Planning...");

            //全局规划初始化，清空数据。
            planner_plan_->clear();
            //调用MoveBase类的makePlan函数，如果成功为临时目标制定全局规划 planner_plan_, 则返回true
            bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

            //如果成功为临时目标制定全局规划
            if (gotPlan)
            {
                //打印成功制定全局规划，并打印规划路线上的点数。
                ROS_DEBUG_NAMED("move_base_plan_thread", "Got Plan with %zu points!", planner_plan_->size());
                //将规划好的路径存放到temp_plan中
                std::vector<geometry_msgs::PoseStamped> *temp_plan = planner_plan_;

                lock.lock();
                planner_plan_ = latest_plan_;
                latest_plan_ = temp_plan;
                //最近一次有效全局规划的时间设置为当前时间。
                last_valid_plan_ = ros::Time::now();
                planning_retries_ = 0;
                new_global_plan_ = true;

                ROS_DEBUG_NAMED("move_base_plan_thread", "Generated a plan from the base_global_planner");

                //确保只有我们还没到达目标时才启动controller以局部规划。
                //如果runPlanner_在调用此函数时被置为“真”，将MoveBase状态设置为controlling（局部规划中）；
                if (runPlanner_)
                    state_ = CONTROLLING;
                //如果规划频率小于0；runPlanner_置为假。
                if (planner_frequency_ <= 0)
                    runPlanner_ = false;
                lock.unlock();
            }
            //如果规划失败，且MoveBase还在planning的状态。
            else if (state_ == PLANNING)
            {
                ROS_DEBUG_NAMED("move_base_plan_thread", "No Plan...");
                //容忍本次全局规划的时间 = 上次成功规划的时间 + 容忍时间。
                ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

                lock.lock();
                //对同一目标的全局规划次数记录 +1
                planning_retries_++;
                //如果runplanning为真，且目前超时或超次数，则进入恢复模式（默认为原地旋转）。
                if (runPlanner_ &&
                    (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_)))
                {
                    // MoveBase状态设置为恢复模式
                    state_ = CLEARING;
                    //全局规划标志位false
                    runPlanner_ = false;
                    //发布零速度，保证机器人停止运动。
                    publishZeroVelocity();
                    //全局规划失败。
                    recovery_trigger_ = PLANNING_R;
                }

                lock.unlock();
            }

            // take the mutex for the next iteration
            lock.lock();

            // setup sleep interface if needed
            if (planner_frequency_ > 0)
            {
                ros::Duration sleep_time = (start_time + ros::Duration(1.0 / planner_frequency_)) - ros::Time::now();
                if (sleep_time > ros::Duration(0.0))
                {
                    wait_for_wake = true;
                    timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
                }
            }
        }
    }
    //%----------------------------------------------------------------
    /*<2> 控制主体 MoveBase::executeCb
       executeCb是Action的回调函数，它是MoveBase控制流的主体，它调用了MoveBase内另外几个作为子部分的重要成员函数，先后完成了全局规划和局部规划。
        在函数的开始部分，它对Action收到的目标进行四元数检测、坐标系转换，然后将其设置为全局规划的目标，并设置了一些标志位。 */
    void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr &move_base_goal)
    {
        //首先检测首的的目标位置的旋转四元数是否有效，若无效，则直接返回。
        if (!isQuaternionValid(move_base_goal->target_pose.pose.orientation))
        {
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
        }
        //将目标位置转换到global坐标系下，（geometry_msgs形式）
        geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

        publishZeroVelocity();
        //启动全局规划。
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        //用接收到的目标goal来更新全局变量，即全局规划目标，这个值在planThread中被用来做全局规划的当前目标
        planner_goal_ = goal;
        //全局规划标志位设置为true
        runPlanner_ = true;
        //开始做全局规划并于此处堵塞。
        planner_cond_.notify_one();
        lock.unlock();
        /*     接下来，由于全局规划器线程绑定的函数plannerThread()里有planner_cond_对象的wait函数，
            在这里调用notify会直接启动全局规划器线程，进行全局路径规划。 */

        //全局规划后，发布不妙到current_goal话题上
        current_goal_pub_.publish(goal);

        //局部规划频率。
        ros::Rate r(controller_frequency_);
        //如果代价地图被关闭，在这里重新启动。
        if (shutdown_costmaps_)
        {
            ROS_DEBUG_NAMED("move_base", "Starting up costmaps that were shut down previously");
            planner_costmap_ros_->start();
            controller_costmap_ros_->start();
        }

        //上一次有效的局部规划时间设置为现在
        last_valid_control_ = ros::Time::now();
        //上一次有效的全局规划时间设置为现在
        last_valid_plan_ = ros::Time::now();
        //上一次震荡重置时间设置为现在。
        last_oscillation_reset_ = ros::Time::now();
        //对同一目标的全局规划记录归为0。
        planning_retries_ = 0;

        ros::NodeHandle n;
        //全局规划结束；接下来循环效用executeCycle函数来控制机器人机器人进行局部规划，完成相应的跟随。
        while (n.ok())
        {
            //如果c_freq_change_即局部规划频率需要中途进行更改，用更改后的controller_frequency_来更新r值。
            if (c_freq_change_)
            {
                ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
                r = ros::Rate(controller_frequency_);
                c_freq_change_ = false;
            }
            //这里需要进行判断：
            //  1；如果action服务器被抢占，可能是局部规划进行中接受到新的目标，也可能是受到取消行动的命令。
            //      。如果收到新目标，放弃当前目标，重复上面对目标进行的操作，使用新目标，重新进行全局规划。
            //      。如果受到取消行动命令，则直接结束返回。
            //  2；如果服务器未被抢占，或者被抢占的if结构已经执行完毕，接下来进行局部规划，调用executeCycle函数，并记录局部控制起始时间。

            //如果action的服务器被抢占。
            if (as_->isPreemptRequested())
            {
                if (as_->isNewGoalAvailable())
                {
                    //如果获得了新的目标，接受并储存新目标，并就上述过程重新进行一遍。
                    move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

                    //检测四元数是否有效。
                    if (!isQuaternionValid(new_goal.target_pose.pose.orientation))
                    {
                        as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
                        return;
                    }
                    //将新坐标转换到全局坐标系下（map）
                    goal = goalToGlobalFrame(new_goal.target_pose);

                    //重新恢复行为索引位为0
                    recovery_index_ = 0;
                    //重设MoveBase状态为全局规划中。
                    state_ = PLANNING;

                    //重新调用planThread进行全局规划。
                    lock.lock();
                    planner_goal_ = goal;
                    runPlanner_ = true;
                    planner_cond_.notify_one();
                    lock.unlock();

                    //全局规划成功后，发布新目标到current_goal话题上。
                    ROS_DEBUG_NAMED("move_base", "move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
                    current_goal_pub_.publish(goal);

                    // make sure to reset our timeouts and counters
                    last_valid_control_ = ros::Time::now();
                    last_valid_plan_ = ros::Time::now();
                    last_oscillation_reset_ = ros::Time::now();
                    planning_retries_ = 0;
                }
                else
                {
                    //否则，服务器的抢占是由于接受到了取消行动的命令。
                    //然后重置服务器状态。
                    resetState();

                    // action服务器清除相关内容，并调用setPreempted（）函数。
                    ROS_DEBUG_NAMED("move_base", "Move base preempting the current goal");
                    as_->setPreempted();

                    //取消命令后，返回。
                    return;
                }
            }

            //服务器接收到目标后，如果没有被新目标或取消命令抢占
            //检查目标是否被转换到全局坐标系（map）下。
            if (goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID())
            {
                goal = goalToGlobalFrame(goal);

                //恢复行为索引重置为0；MoveBase状态重置为全局规划中。
                recovery_index_ = 0;
                state_ = PLANNING;

                // we have a new goal so make sure the planner is awake
                lock.lock();
                planner_goal_ = goal;
                runPlanner_ = true;
                planner_cond_.notify_one();
                lock.unlock();

                // publish the goal point to the visualizer
                ROS_DEBUG_NAMED("move_base", "The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
                current_goal_pub_.publish(goal);

                // make sure to reset our timeouts and counters
                last_valid_control_ = ros::Time::now();
                last_valid_plan_ = ros::Time::now();
                last_oscillation_reset_ = ros::Time::now();
                planning_retries_ = 0;
            }

            //记录开始局部规划的时刻为当前时间
            ros::WallTime start = ros::WallTime::now();

            //调用executeCycle函数进行局部规划，传入目标和全局规划路线。
            bool done = executeCycle(goal);

            // if we're done, then we'll return from execute
            if (done)
                return;

            //记录从局部规划开始到这时的时间差。

            ros::WallDuration t_diff = ros::WallTime::now() - start;
            //打印用多长时间完成操作。
            ROS_DEBUG_NAMED("move_base", "Full control cycle time: %.9f\n", t_diff.toSec());

            //用局部规划频率进行休眠。
            r.sleep();
            // cycleTime用来获取从 r实例初始化到 r实例被调用sleep函数的时间间隔。
            //若时间间隔超过了 1/局部规划频率，并且还在局部规划中；打印未达到实际要求，实际上时间是r.cycleTime().toSec()
            if (r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
                ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
        }

        //唤醒全局规划线程，以使局部规划能干净的退出。
        lock.lock();
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //如果节点被关闭，那么Action服务器也被关闭并且返回。
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
        return;
    }

    //%----------------------------------------------------------------
    //计算两点之间的直线距离
    double MoveBase::distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2)
    {
        return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
    }

    //%----------------------------------------------------------------
    /* <5> 局部规划 MoveBase::executeCycle
    executeCycle函数的作用是进行局部规划，函数先声明了将要发布的速度，然后获取当前位姿并格式转换。 */
    bool MoveBase::executeCycle(geometry_msgs::PoseStamped &goal)
    {
        boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
        //首先声明速度度信息。
        geometry_msgs::Twist cmd_vel;

        //声明全局姿态。
        geometry_msgs::PoseStamped global_pose;
        //从全局代价地图上获取机器人的当前位姿。
        getRobotPose(global_pose, planner_costmap_ros_);
        //把当前为子储存在current_position中。
        const geometry_msgs::PoseStamped &current_position = global_pose;

        // feedback指的是从服务端轴哦其反馈给客户端的信息，把当前机器人的位姿反馈给客户端。
        move_base_msgs::MoveBaseFeedback feedback;
        feedback.base_position = current_position;
        as_->publishFeedback(feedback);

        //做几个判断，判断机器人是否被困住（若是，则进入恢复行为，即针对机器人运动异常情况做出的指令，具体内容再该部分学习理解），
        //并检查局部规划的地图是否是当前的，否则发布零速，停止规划，制停机器人。

        //若长时间内移动机器人的距离没有超过震荡距离，则认为机器人在震荡（长时间被困在一片校区与内），进入到恢复行为。
        if (distance(current_position, oscillation_pose_) >= oscillation_distance_)
        {
            //把最新的震荡重置为当前时间
            last_oscillation_reset_ = ros::Time::now();
            //震荡位姿设置为当前的位姿。
            oscillation_pose_ = current_position;

            //如果上次的恢复行为有震荡引起，我们则重新设置恢复行为的引导。
            if (recovery_trigger_ == OSCILLATION_R)
                recovery_index_ = 0;
        }

        //检测局部代价地图是否是当前的。
        if (!controller_costmap_ros_->isCurrent())
        {
            ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety", ros::this_node::getName().c_str());
            publishZeroVelocity();
            return false;
        }
        /*
        通过标志位判定全局规划是否得出新的路线，然后通过指针交换，将latest_plan_（最新的全局规划结果）
        的值传递给controller_plan_即局部规划使用，然后将上一次的局部规划路线传递给latest_plan。 */

        // new_global_plan_标志位在planThread中被置为真，表示生成了全局规划。
        if (new_global_plan_)
        {
            //重置标志位。
            new_global_plan_ = false;

            ROS_DEBUG_NAMED("move_base", "Got a new plan...swap pointers");

            // do a pointer swap under mutex
            std::vector<geometry_msgs::PoseStamped> *temp_plan = controller_plan_;
            //在指针的保护下，交换latest_plan和controller_plan的值。
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            // controller_plan_储存（当前最新的，ok的待到达的全局规划）
            controller_plan_ = latest_plan_;
            //使得全局规划好的planner_plan经由latest——plan一路传递到controller_plan供局部规划器使用。
            latest_plan_ = temp_plan;
            lock.unlock();
            ROS_DEBUG_NAMED("move_base", "pointers swapped!");

            //在实例tc_上调用局部规划器BaseLocalPlanner的类函数setPlan()，
            //把全局规划的结果传递给局部规划器，如果传递失败，推出并返回。
            if (!tc_->setPlan(*controller_plan_))
            {
                // ABORT and SHUTDOWN COSTMAPS
                ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                resetState();

                //如果传递失败，停止全局规划线程。
                lock.lock();
                runPlanner_ = false;
                lock.unlock();

                //停止Action服务器，打印“将全局规划传递到局部规规划器控制失败"
                as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
                return true;
            }

            //如果我们找到有效的局部规划路线，且恢复行为是 "全局规划失败"，重复恢复行为索引归零。
            if (recovery_trigger_ == PLANNING_R)
                recovery_index_ = 0;
        }

        //对MoveBase状态进行判断，由于局部规划器在全局规划结束后才调用，所有会有以下几种结果。
        // the move_base state machine, handles the control logic for navigation
        switch (state_)
        {
        //  1；
        //我们还在进行全局规划状态。
        case PLANNING:
        {
            boost::recursive_mutex::scoped_lock lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one();
        }
            ROS_DEBUG_NAMED("move_base", "Waiting for plan, in the planning state.");
            break;

            //  2；
            //全局规划成功，得到全局路线，这里进行真正的局部规划。
            /*《《
                  。如果已到达了终点，结束局部规划；
                  。如果没有到达终点，检查机器人是否被捆住，若是，则进入恢复行为。
                  。如果没有到达终点，且状态正常，调用局部规划器实例tc_->computeVelocityCommands(cmd_vel)函数，
                    这个函数结合传入的全局规划路线和其他 “因素” 得出局部规划结果，即速度指令，存放在cmd_vel中，将其发布控制机器人运动。
                    (
                    computeVelocityCommands(cmd_vel)函数具体内容在base_local_planner中，他通过在给定速度范围内模拟各个方向的速度
                    得到当前可能的小范围行驶路线，进一步对得到的路线进行贫姑，得分最高的路线对应的速度就是函数得到的结果，所以局部规划
                    更像是不断循环进行的“短期规划”
                    )
            */

            //如果全局规划成功，进入CONTROLLING状态中，开始寻找有效的速度控制
        case CONTROLLING:
            ROS_DEBUG_NAMED("move_base", "In controlling state.");

            //检查是否到达终点，如果到达终点，结束。
            if (tc_->isGoalReached())
            {
                ROS_DEBUG_NAMED("move_base", "Goal reached!");
                resetState();

                //结束全局规划线程。
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                //规划标志位 false
                runPlanner_ = false;
                lock.unlock();

                // Action返回成功。
                as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
                return true;
            }

            //如果未到达终点，检查是否处于震荡状态。
            if (oscillation_timeout_ > 0.0 &&
                last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
            {
                //如果震荡状态超时了，发布0速度。
                publishZeroVelocity();
                // MoveBase状态重置为恢复状态。
                state_ = CLEARING;
                //恢复行为触发器标志为 长时间困在以小片区域（震荡状态）
                recovery_trigger_ = OSCILLATION_R;
            }

            {
                boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

                //局部规划器实例tc_被传入到全局规划后，调用conputeVelocityCommands函数计算速度储存到cmd_vel中。
                if (tc_->computeVelocityCommands(cmd_vel))
                {
                    ROS_DEBUG_NAMED("move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
                    //若成功计算速度，上一次有效的局部控制的时间设置为当前。
                    last_valid_control_ = ros::Time::now();
                    //向地盘发送速度控制消息，一个循环发送一次速度指令。
                    vel_pub_.publish(cmd_vel);
                    if (recovery_trigger_ == CONTROLLING_R)
                        //如果恢复行为触发器的值为局部规划失败，把索引置为0。
                        recovery_index_ = 0;
                }
                /*
                        若computeVelocityCommands(cmd_vel)函数计算失败，
                        且超时，则进入对应恢复行为，若未超时，则发布零速制停机器人，重新全局规划，再进行下一次局部规划。 */
                //若速度计算失败
                else
                {
                    ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
                    //计算局部规划用时限制
                    ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

                    //若局部规划用时超出限制。
                    if (ros::Time::now() > attempt_end)
                    {
                        //发布 0速度，进入恢复行为，触发器重置为局部规划失败。
                        publishZeroVelocity();
                        state_ = CLEARING;
                        recovery_trigger_ = CONTROLLING_R;
                    }
                    //若局部规划用时没有超出限制。
                    else
                    {
                        //发布 0速度，在机器人当前位置再次进入到全局规划中。
                        last_valid_plan_ = ros::Time::now();
                        planning_retries_ = 0;
                        state_ = PLANNING;
                        publishZeroVelocity();

                        // enable the planner thread in case it isn't running on a clock
                        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                        runPlanner_ = true;
                        planner_cond_.notify_one();
                        lock.unlock();
                    }
                }
            }

            break;

        //如果全局规划失败，进入恢复行为模式，我们尝试用用户提供的恢复行为去清除周围障碍物。
        case CLEARING:
            ROS_DEBUG_NAMED("move_base", "In clearing/recovery state");
            //如果允许使用恢复行为，且恢复行为索引值小于恢复姓为数组的大小。
            if (recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size())
            {
                ROS_DEBUG_NAMED("move_base_recovery", "Executing behavior %u of %zu", recovery_index_ + 1, recovery_behaviors_.size());

                move_base_msgs::RecoveryStatus msg;
                msg.pose_stamped = current_position;
                msg.current_recovery_number = recovery_index_;
                msg.total_number_of_recoveries = recovery_behaviors_.size();
                msg.recovery_behavior_name = recovery_behavior_names_[recovery_index_];

                recovery_status_pub_.publish(msg);

                //开始恢复行为，在ececuteCycle的循环中一次次迭代恢复行为。
                recovery_behaviors_[recovery_index_]->runBehavior();

                //上一次震荡重置时间设置为现在。
                last_oscillation_reset_ = ros::Time::now();

                // we'll check if the recovery behavior actually worked
                ROS_DEBUG_NAMED("move_base_recovery", "Going back to planning state");
                last_valid_plan_ = ros::Time::now();
                planning_retries_ = 0;
                state_ = PLANNING;

                // update the index of the next recovery behavior that we'll try
                recovery_index_++;
            }
            //若恢复行为无效。
            else
            {
                //打印 “所有恢复行为都失败了，关闭全局规划器”
                ROS_DEBUG_NAMED("move_base_recovery", "All recovery behaviors have failed, locking the planner and disabling it.");
                // disable the planner thread
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                runPlanner_ = false;
                lock.unlock();

                ROS_DEBUG_NAMED("move_base_recovery", "Something should abort after this.");

                //反馈失败的具体信息。
                if (recovery_trigger_ == CONTROLLING_R)
                {
                    ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
                    as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
                }
                else if (recovery_trigger_ == PLANNING_R)
                {
                    ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
                    as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
                }
                else if (recovery_trigger_ == OSCILLATION_R)
                {
                    ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
                    as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
                }
                resetState();
                return true;
            }
            break;
        default:
            ROS_ERROR("This case should never be reached, something is wrong, aborting");
            resetState();
            //关闭全局规划器线程。
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            runPlanner_ = false;
            lock.unlock();
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
            return true;
        }

        // we aren't done yet
        return false;
    }
    //%----------------------------------------------------------------
    //! 重点！！！ 加载恢复行为模块。
    bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node)
    {
        XmlRpc::XmlRpcValue behavior_list;
        //下面进行recovery behaviors的config参数解析
        if (node.getParam("recovery_behaviors", behavior_list))
        {
            if (behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                for (int i = 0; i < behavior_list.size(); ++i)
                {
                    if (behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
                    {
                        if (behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type"))
                        {
                            // check for recovery behaviors with the same name
                            for (int j = i + 1; j < behavior_list.size(); j++)
                            {
                                if (behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct)
                                {
                                    if (behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type"))
                                    {
                                        std::string name_i = behavior_list[i]["name"];
                                        std::string name_j = behavior_list[j]["name"];
                                        if (name_i == name_j)
                                        {
                                            ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                                                      name_i.c_str());
                                            return false;
                                        }
                                    }
                                }
                            }
                        }
                        else
                        {
                            ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
                            return false;
                        }
                    }
                    else
                    {
                        ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                                  behavior_list[i].getType());
                        return false;
                    }
                }

                // if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
                for (int i = 0; i < behavior_list.size(); ++i)
                {
                    try
                    {
                        // check if a non fully qualified name has potentially been passed in
                        if (!recovery_loader_.isClassAvailable(behavior_list[i]["type"]))
                        {
                            std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
                            for (unsigned int i = 0; i < classes.size(); ++i)
                            {
                                if (behavior_list[i]["type"] == recovery_loader_.getName(classes[i]))
                                {
                                    // if we've found a match... we'll get the fully qualified name and break out of the loop
                                    ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                                             std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                                    behavior_list[i]["type"] = classes[i];
                                    break;
                                }
                            }
                        }
                        //￥ 加载恢复行为函数共享指针
                        boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

                        // shouldn't be possible, but it won't hurt to check
                        if (behavior.get() == NULL)
                        {
                            ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
                            return false;
                        }

                        // initialize the recovery behavior with its name
                        behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
                        recovery_behavior_names_.push_back(behavior_list[i]["name"]);
                        recovery_behaviors_.push_back(behavior);
                    }
                    catch (pluginlib::PluginlibException &ex)
                    {
                        ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
                        return false;
                    }
                }
            }
            else
            {
                ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
                          behavior_list.getType());
                return false;
            }
        }
        else
        {
            // if no recovery_behaviors are specified, we'll just load the defaults
            return false;
        }

        // if we've made it here... we've constructed a recovery behavior list successfully
        return true;
    }
    //%----------------------------------------------------------------
    //加载默认的恢复行为函数
    void MoveBase::loadDefaultRecoveryBehaviors()
    {
        recovery_behaviors_.clear();
        try
        {
            // we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
            ros::NodeHandle n("~");
            n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
            n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

            // first, we'll load a recovery behavior to clear the costmap
            boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
            cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back("conservative_reset");
            recovery_behaviors_.push_back(cons_clear);

            // next, we'll load a recovery behavior to rotate in place
            boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
            if (clearing_rotation_allowed_)
            {
                rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
                recovery_behavior_names_.push_back("rotate_recovery");
                recovery_behaviors_.push_back(rotate);
            }

            // next, we'll load a recovery behavior that will do an aggressive reset of the costmap
            boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
            ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back("aggressive_reset");
            recovery_behaviors_.push_back(ags_clear);

            // we'll rotate in-place one more time
            if (clearing_rotation_allowed_)
            {
                recovery_behaviors_.push_back(rotate);
                recovery_behavior_names_.push_back("rotate_recovery");
            }
        }
        catch (pluginlib::PluginlibException &ex)
        {
            ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
        }

        return;
    }
    //%----------------------------------------------------------------
    //重置movebase
    void MoveBase::resetState()
    {
        // Disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();

        //重置movebase标志
        //设定movebase为规划状态
        state_ = PLANNING;
        //当前恢复列表为0
        recovery_index_ = 0;
        recovery_trigger_ = PLANNING_R;
        //发布0速度，禁止机器人移动
        publishZeroVelocity();

        // if we shutdown our costmaps when we're deactivated... we'll do that now
        if (shutdown_costmaps_)
        {
            ROS_DEBUG_NAMED("move_base", "Stopping costmaps");
            planner_costmap_ros_->stop();
            controller_costmap_ros_->stop();
        }
    }
    //%----------------------------------------------------------------
    //在给定的代价图框架上获取机器人姿势
    bool MoveBase::getRobotPose(geometry_msgs::PoseStamped &global_pose, costmap_2d::Costmap2DROS *costmap)
    {
        tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
        geometry_msgs::PoseStamped robot_pose;
        tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
        robot_pose.header.frame_id = robot_base_frame_; //机器人坐标系
        robot_pose.header.stamp = ros::Time();          // latest available
        ros::Time current_time = ros::Time::now();      // save time for checking tf delay later

        // get robot pose on the given costmap frame
        try
        {
            tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
        }
        catch (tf2::LookupException &ex)
        {
            ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf2::ConnectivityException &ex)
        {
            ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf2::ExtrapolationException &ex)
        {
            ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
            return false;
        }

        // check if global_pose time stamp is within costmap transform tolerance
        if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
        {
            ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. "
                                   "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f",
                              costmap->getName().c_str(),
                              current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
            return false;
        }

        return true;
    }
};
