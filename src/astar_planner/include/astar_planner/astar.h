#ifndef ASTAR_H
#define ASTAR_H

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

#define inf 1 << 20

namespace AstarPlanner{

struct Node{
    Eigen::Vector2i index_;     //地图索引
    Eigen::Vector2d position_;  //世界系位置
    double g_score_, f_score_, h_score, d_score_;  
    
    int state_ = -1; // -1 未扩展，0 可通行，1不可通行
    Node *parent_;              //父节点用于回溯

    Node(Eigen::Vector2i index, Eigen::Vector2d position) {
        index_ = index;
        position_ = position;
        parent_ = nullptr;
        g_score_ = inf;
        f_score_ = 0;
        d_score_ = 0;
        state_ = -1;
        //待补充
    }
    Node(Eigen::Vector2i index, Eigen::Vector2d position, int state) {
        index_ = index;
        position_ = position;
        parent_ = nullptr;
        g_score_ = inf;
        f_score_ = 0;
        d_score_ = 0;
        state_ = state;
    }
    // Node(geometry_msgs::PoseStamped &)
    ~Node() {}

    double getCost(){
        return g_score_ + h_score;
    }
};

struct compare {
    bool operator() (const Node *node1, const Node *node2) const {
        return node1->f_score_ > node2->f_score_;
    }
};

class AstarGlobalPlanner : public nav_core::BaseGlobalPlanner{

public:
    AstarGlobalPlanner (ros::NodeHandle &){}
    AstarGlobalPlanner (){}
    AstarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* cmap_ptr);
    
    ros::NodeHandle ROSNodeHandle;
    ros::Publisher _plan_pub;
    ros::Publisher _gridmap_pub;
    ros::Publisher _corr_pub;

    std::string _frame_id;
    /** overriden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* cmap_ptr);
    bool makePlan(const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal, 
                  std::vector<geometry_msgs::PoseStamped>& plan
              );

private:
    std::priority_queue<Node*, std::vector<Node *>, compare> open_list_;
    std::vector<Node *>  path_node_;
    std::vector<Eigen::Vector2d> path_;
    std::vector<Node *> visited_node_; 
    std::vector<Eigen::Vector2d> keyPath;
    std::vector<std::vector<Eigen::Vector2d>> corridors_;
    //map params
    Node * **node_map_ = nullptr;
    costmap_2d::Costmap2DROS *cmap_ptr_;

    int enlargement_factor = 1;
    //map params
    Eigen::Vector2d origin_; //地图原点
    double resolution_;      //分辨率
    uint map_size_x_, map_size_y_;      //地图大小
    double world_size_x_, world_size_y_;    
    int obs_thread = 60;

    unsigned char* cost_;
    double tie_breaker_; 
    int heu_method_;
    double heu_weight_;
    enum {EUCLIDEAN, MANHATTAN, DIAGONAL}; //三种启发式函数
    bool initialized_;

    bool search(Eigen::Vector2d start, Eigen::Vector2d goal); 
    Eigen::Vector2i posToIndex(Eigen::Vector2d position);  //世界系位置转地图栅格索引
    Eigen::Vector2d indexToPos(Eigen::Vector2i index);      //地图栅格索引转世界系位置
    double getHeu(Eigen::Vector2d cur, Eigen::Vector2d goal);
    void traceback(Node *goal_node);    //回溯得到整条路径上的node
    void getPath(std::vector<geometry_msgs::PoseStamped>& plan);
    std::vector<Eigen::Vector2d> getVisitedNode();
    void visPath(std::vector<Eigen::Vector2d> path);
    void resetNodeMap();
    void pubGirdmap();
    void initNodeMap();
    void releaseNodeMap();
    void updateNodeMap(); 
    bool checkIdx(Eigen::Vector2i idx);
    double calc_obs_cost(Eigen::Vector2i idx);
    std::vector<Eigen::Vector2d> getKeyPoint();
    bool isClear(Node *start, Node *end);
    void visPath();
    void visKeyPath(std::vector<geometry_msgs::PoseStamped> &plan);
    std::vector<Eigen::Vector2d> corrExpand(Eigen::Vector2i center);
    void generateCorridor();
    void visCorridors(bool conceal);
    // void conceal_old_corridor();
};

}

#endif