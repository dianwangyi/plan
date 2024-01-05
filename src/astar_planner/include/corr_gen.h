#ifndef CORRIDORGENERATE_H__
#define CORRIDORGENERATE_H__

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <Eigen/Dense>
#include <queue>
#include <set>
#include <vector>
#include <algorithm>
#include <costmap_2d/costmap_2d_ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "astar.h"
/**
 * bounding box
 * P1-------P2
 * |        |
 * |        |
 * |        |
 * P4-------P3
 * 
 * */
struct Bounding_box
{
    Eigen::Vector2i center;
    Eigen::Vector2i P1;
    Eigen::Vector2i P2;
    Eigen::Vector2i P3;
    Eigen::Vector2i P4;

    int xmin;
    int xmax;
    int ymin;
    int ymax;

    Bounding_box(){};
    ~Bounding_box(){};

    Bounding_box(Eigen::Vector2i const pt, const double& resolution){
        xmin = pt[0];
        xmax = pt[0];
        ymin = pt[1];
        xmax = pt[1];
        
        center(0) = pt[0] + resolution/2;
        center(1) = pt[1] + resolution/2;
    }

    Bounding_box(Eigen::Vector2i const p2, Eigen::Vector2i const p3){
        xmin = std::min(p2[0], p3[0]);
        xmax = std::max(p2[0], p3[0]);
        ymin = std::min(p2[1], p3[1]);
        ymax = std::max(p2[1], p3[1]);

        P1 = Eigen::Vector2i(xmin, ymax);
        P2 = Eigen::Vector2i(xmax, ymax);
        P3 = Eigen::Vector2i(xmax, ymin);
        P4 = Eigen::Vector2i(xmin, ymin);

        center(0) = (xmin + xmax) / 2;
        center(1) = (ymin + ymax) / 2;
    }
};

class CorridorGenerator{
public:
    CorridorGenerator(){};

    CorridorGenerator(std::string name, costmap_2d::Costmap2DROS* cmap_ptr){
        init(name, cmap_ptr);
    }

    bool init(std::string name, costmap_2d::Costmap2DROS* cmap_ptr);

    bool AstarPathCorr(const std::vector<Eigen::Vector2d>& keyPath, 
                        std::vector<Bounding_box> &corridors);

    std::vector<Bounding_box> corridors_;

private:
    // std::vector<std::vector<Eigen::Vector2d>> corridors_;
    
    Eigen::Vector2d path_;

    ros::Publisher _corr_pub;

    //map params
    std::string _frame_id;
    costmap_2d::Costmap2DROS *cmap_ptr_;
    Eigen::Vector2d origin_; //地图原点
    double resolution_;      //分辨率
    uint map_size_x_, map_size_y_;      //地图大小
    double world_size_x_, world_size_y_;   
    unsigned char* map_;


    int max_inflate_iter = 20;
    bool initialized_ = false;

private:
    Eigen::Vector2i posToIndex(Eigen::Vector2d position);  //世界系位置转地图栅格索引
    Eigen::Vector2d indexToPos(Eigen::Vector2i index);      //地图栅格索引转世界系位置
    void visCorridors(bool conceal);

    bool rectangleExpand(Bounding_box& box);
};

#endif