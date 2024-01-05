#ifndef ASTAR_PLANNER_H__
#define ASTAR_PLANNER_H__

#include "astar.h"
#include "corr_gen.h"

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <Eigen/Dense>
#include <queue>
#include <set>
#include <vector>
#include <algorithm>
#include <costmap_2d/costmap_2d_ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <memory>

namespace GlobalPlanner{
    
class AstarGlobalPlanner : public nav_core::BaseGlobalPlanner{

public:
    AstarGlobalPlanner (ros::NodeHandle &){}
    AstarGlobalPlanner (){}
    AstarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* cmap_ptr);
    
    /** overriden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* cmap_ptr);
    bool makePlan(const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal, 
                  std::vector<geometry_msgs::PoseStamped>& plan
              );
private:
    bool is_initlized = false;
    std::string name_;
    costmap_2d::Costmap2DROS* cmap_ptr_;

    std::shared_ptr<AstarSearch> searcher_;

    std::shared_ptr<CorridorGenerator> expander_;
    
    std::vector<Eigen::Vector2d> path_;

    std::vector<Bounding_box> corridors_;
};

}
#endif