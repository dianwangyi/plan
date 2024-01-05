// #include "GlobalPlanner/astar_planner.h"
#include "astar_planner.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(GlobalPlanner::AstarGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace GlobalPlanner{

AstarGlobalPlanner::AstarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* cmap_ptr){
    initialize(name, cmap_ptr);
}

void AstarGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* cmap_ptr){
    is_initlized = false;

    if(!is_initlized){
        name_ = name;
        cmap_ptr_ = cmap_ptr;

        searcher_ = std::make_shared<AstarSearch>(name, cmap_ptr);
        searcher_->initialize(name, cmap_ptr);
        ROS_INFO("searcher init");


        expander_ = std::make_shared<CorridorGenerator>(name, cmap_ptr);
        expander_->init(name, cmap_ptr);
        ROS_INFO("searcher init");

        ROS_INFO("AstarGlobalPlanner inited");
        is_initlized = true;
    }
    else
        ROS_WARN("This planner has already been initialized... doing nothing");
}

bool AstarGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal, 
                  std::vector<geometry_msgs::PoseStamped>& plan
              )
{   
    if( !is_initlized )
        return false;

    plan.clear();
    ROS_INFO("start search");

    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;

    poseStampedMsgToTF(goal, goal_tf);
    poseStampedMsgToTF(start, start_tf);

    if( !searcher_->search(start, goal, path_) )
        return false;

    for(const auto pos : path_){
        geometry_msgs::PoseStamped pose = goal;

        pose.pose.position.x = pos[0];
        pose.pose.position.y = pos[1];
        pose.pose.position.z = 0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        ROS_INFO("AstarGlobalPlanner : x = %f y = %f", pos[0], pos[1]);
        plan.push_back(pose);
    }
    ROS_INFO("end search");

    if(!expander_->AstarPathCorr(path_, corridors_)){
        return false;
    }
    ROS_INFO("end expand");
    
    ROS_INFO("corridor_ size = %d", corridors_.size());

    for(int i = 0; i < corridors_.size(); i++){
        ROS_INFO("Astar planner corridor");
        ROS_INFO("corr[%d] xmin = %d, xmax = %d, ymin = %d, ymax = %d", 
                i, corridors_[i].xmin, corridors_[i].xmax, corridors_[i].ymin, corridors_[i].ymax);
    }
    return true;
}

}

