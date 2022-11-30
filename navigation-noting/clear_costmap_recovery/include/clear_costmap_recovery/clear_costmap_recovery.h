#ifndef CLEAR_COSTMAP_RECOVERY_H_
#define CLEAR_COSTMAP_RECOVERY_H_
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>

namespace clear_costmap_recovery{
  /**
   * @class ClearCostmapRecovery
   * @brief A recovery behavior that reverts the navigation stack's costmaps to the static map outside of a user-specified region.
   */
  class ClearCostmapRecovery : public nav_core::RecoveryBehavior {
    public:
      /**
       * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
       * @param
       * @return
       */
      ClearCostmapRecovery();

      /**
       * @brief  Initialization function for the ClearCostmapRecovery recovery behavior
       * @param tf A pointer to a transform listener
       * @param global_costmap A pointer to the global_costmap used by the navigation stack
       * @param local_costmap A pointer to the local_costmap used by the navigation stack
       */
      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

      /**
       * @brief  Run the ClearCostmapRecovery recovery behavior. Reverts the
       * costmap to the static map outside of a user-specified window and
       * clears unknown space around the robot.
       */
      void runBehavior();

    private:
      void clear(costmap_2d::Costmap2DROS* costmap);
      void clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap, double pose_x, double pose_y);
      costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
      std::string name_;
      tf2_ros::Buffer* tf_;
      bool initialized_;
      bool force_updating_; ///< force costmap update after clearing, so we don't need to wait for update thread
      double reset_distance_;
      bool invert_area_to_clear_;
      std::string affected_maps_; ///< clear only local, global or both costmaps
      std::set<std::string> clearable_layers_; ///< Layer names which will be cleared.
  };
};
#endif
