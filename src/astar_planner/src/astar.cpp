#include "astar_planner/astar.h"
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(AstarPlanner::AstarGlobalPlanner, nav_core::BaseGlobalPlanner)
using namespace Eigen;

namespace AstarPlanner{


    AstarGlobalPlanner::AstarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* cmap_ptr){
        initialize(name, cmap_ptr);
    }


/** overriden classes from interface nav_core::BaseGlobalPlanner **/
void AstarGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* cmap_ptr){
    initialized_ = false;
    if(!initialized_){
    // resetNodeMap();

        cmap_ptr_ = cmap_ptr;

        ros::NodeHandle private_nh("~" + name);
        _plan_pub = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
        _gridmap_pub = private_nh.advertise<nav_msgs::OccupancyGrid>("my_map", 1);
        _corr_pub = private_nh.advertise<visualization_msgs::MarkerArray>("normals_marker_array", 100);
        _frame_id = cmap_ptr_->getGlobalFrameID();

        tie_breaker_ = 1.0 + 1.0 / 1000;
        //tie_breaker_ = 1.0;
        heu_weight_ = 1;
        heu_method_ = EUCLIDEAN;
        //heu_method_ = MANHATTAN;
        //heu_method_ = DIAGONAL;

        //map
        initNodeMap();

        ROS_INFO("BAstar planner initialized successfully");
        initialized_ = true;
    }

}

void AstarGlobalPlanner::initNodeMap(){
    if(node_map_ != nullptr){
        releaseNodeMap();
    }

    origin_[0] =  cmap_ptr_->getCostmap()->getOriginX();
    origin_[1] =  cmap_ptr_->getCostmap()->getOriginY();
    // cout << "origin_:" << origin_[0] << " " << origin_[1] << endl;
    resolution_ = cmap_ptr_->getCostmap()->getResolution();
    // cout << "resolution_:" << resolution_ << endl;
    map_size_x_ = cmap_ptr_->getCostmap()->getSizeInCellsX();
    map_size_y_ = cmap_ptr_->getCostmap()->getSizeInCellsY();
    ROS_INFO("map_size_x_ : %d, map_size_y : %d", map_size_x_, map_size_y_);
    // cout << "map_size: "<< map_size_x_ << " " << map_size_y_ << endl;
    Eigen::Vector2i map_size;
    map_size << map_size_x_, map_size_y_;
    Eigen::Vector2d world_size;
    world_size = indexToPos(map_size);
    world_size_x_ = world_size[0];
    world_size_y_ = world_size[1];
    cost_ =  cmap_ptr_->getCostmap()->getCharMap();
    // cout << cost_[0] << endl;   //先x后y

    std::vector<std::vector<int>> tmp_map(map_size_x_ / enlargement_factor, 
                        std::vector<int>(map_size_y_ / enlargement_factor, -1));

    // ROS_INFO("tmp map x = %d, y = %d", tmp_map.size(), tmp_map[0].size());

    int cur = 0;
    for(int i = 0; i < map_size_x_; i++){
        for(int j = 0; j < map_size_y_; j++){
            int cost = cost_[cur];

            if(tmp_map[i/enlargement_factor][j/enlargement_factor] != 1
                && cost == 0)
                tmp_map[i/enlargement_factor][j/enlargement_factor] = 0;
            else if(tmp_map[i/enlargement_factor][j/enlargement_factor] != 1 
                && cost < 0)
                tmp_map[i/enlargement_factor][j/enlargement_factor] = -1;
            else
                tmp_map[i/enlargement_factor][j/enlargement_factor] = 1;
            cur ++;
        }
    }
    // ROS_INFO("node map x = %d, y = %d", tmp_map.size(), tmp_map[0].size());

    int node_map_size_x = map_size_x_ / enlargement_factor;
    int node_map_size_y = map_size_y_ / enlargement_factor;
    node_map_ = new Node * *[node_map_size_x];

    for (int i = 0; i < node_map_size_x; ++i) {
        node_map_[i] = new Node * [node_map_size_y];

        for (int j = 0; j < node_map_size_y; ++j) {
            int num = tmp_map[i][j];

            Eigen::Vector2i index;
            index << i, j;
            Eigen::Vector2d position = indexToPos(index);
            node_map_[i][j] = new Node(index, position);

            node_map_[i][j]->state_ = num;
        }
    }
    ROS_INFO("map init finish");
}

void AstarGlobalPlanner::releaseNodeMap(){
    if(node_map_ == nullptr)
        return;

    int node_map_size_x = map_size_x_ / enlargement_factor;
    int node_map_size_y = map_size_y_ / enlargement_factor;

    for(int i = 0; i < node_map_size_x; i++){
        for(int j = 0; j < node_map_size_y; j++){
            delete node_map_[i][j];
            node_map_[i][j] = nullptr;
        }
        delete[] node_map_[i];
        node_map_[i] = nullptr;
    }
    delete[] node_map_;
    node_map_ = nullptr;
}


void AstarGlobalPlanner::updateNodeMap(){
    cost_ =  cmap_ptr_->getCostmap()->getCharMap();
    // cout << cost_[0] << endl;   //先x后y

    std::vector<std::vector<int>> tmp_map(map_size_x_ / enlargement_factor, 
                        std::vector<int>(map_size_y_ / enlargement_factor, -1));

    // ROS_INFO("tmp map x = %d, y = %d", tmp_map.size(), tmp_map[0].size());

    int cur = 0;
    for(int i = 0; i < map_size_x_; i++){
        for(int j = 0; j < map_size_y_; j++){
            int cost = cost_[cur];

            if(tmp_map[i/enlargement_factor][j/enlargement_factor] != 1
                && cost == 0)
                tmp_map[i/enlargement_factor][j/enlargement_factor] = 0;
            else if(tmp_map[i/enlargement_factor][j/enlargement_factor] != 1 
                && cost < 0)
                tmp_map[i/enlargement_factor][j/enlargement_factor] = -1;
            else
                tmp_map[i/enlargement_factor][j/enlargement_factor] = 1;
            cur ++;
        }
    }

    int node_map_size_x = map_size_x_ / enlargement_factor;
    int node_map_size_y = map_size_y_ / enlargement_factor;


    for (int i = 0; i < node_map_size_x; ++i) {
        for (int j = 0; j < node_map_size_y; ++j) {
            int num = tmp_map[i][j];

            node_map_[i][j]->g_score_ = inf;
            node_map_[i][j]->f_score_ = 0;
            node_map_[i][j]->parent_ = nullptr;
            
            node_map_[i][j]->state_ = num;
        }
    }
    ROS_INFO("map update finish");
}

bool AstarGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                const geometry_msgs::PoseStamped& goal, 
                std::vector<geometry_msgs::PoseStamped>& plan
            ){
    if (!initialized_)
    {
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }
    
    ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
              goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    // resetNodeMap();
    updateNodeMap();

    pubGirdmap();

    ROS_INFO("start search");
    ros::Time time1 = ros::Time::now();

    Vector2d start_pos(start.pose.position.x, start.pose.position.y);
    Vector2d goal_pos(goal.pose.position.x, goal.pose.position.y);

    Eigen::Vector2i start_index = posToIndex(start_pos);

    ROS_INFO("start index [%i, %i]", start_index[0], start_index[1]);
    ROS_INFO("start pos [%f, %f]", start_pos[0], start_pos[1]);

    // ROS_INFO("goal index [%i, %i]", goal_pos[0], goal_pos[1]);
    ROS_INFO("goal pos [%f, %f]", goal_pos[0], goal_pos[1]);
    Node *cur_node = node_map_[start_index[0]][start_index[1]];

    cur_node->parent_ = nullptr;
    cur_node->g_score_ = 0.0;
    cur_node->f_score_ = heu_weight_ * getHeu(cur_node->position_, goal_pos);
    Vector2i goal_index = posToIndex(goal_pos);
    ROS_INFO("goal cost = %d", cost_[goal_index[0] + goal_index[1] * map_size_x_]);
    if(!checkIdx(goal_index)){
        ROS_INFO("goal is invaild x = %d, y = %d", goal_index[0], goal_index[1]);
        return false;
    }

    while (!open_list_.empty()) open_list_.pop();
    int max_it = 300000, it = 0;
    open_list_.push(cur_node);

    bool find = false;
    while (!open_list_.empty() && it < max_it) {
        cur_node = open_list_.top();
        it ++;
        // ROS_INFO("current f_score : %f", cur_node->f_score_);
        // // cout << "current f_score " << cur_node->f_score_ << endl;
        // ROS_INFO("current index [%i, %i]", cur_node->index_[0], cur_node->index_[1]);
        // ROS_INFO("goal index [%i, %i]", goal_index[0], goal_index[1]);

        if (cur_node->index_ == goal_index) {
            ROS_INFO("find goal");
            
            find = true;
            Node *goal_node = cur_node;
            traceback(goal_node);
            break;
        }

        open_list_.pop();
        Node *neighbor_ptr;
        //expand neighbor nodes
        for (int i = -1; i < 2; ++i) {
            for (int j = -1; j < 2; ++j) {

                // ROS_INFO("expanding");

                if (i == 0 && j == 0) 
                    continue;
                
                Vector2i neighbor_index;
                neighbor_index << cur_node->index_[0] + i, cur_node->index_[1] + j;
                Vector2d neughbor_pos = indexToPos(neighbor_index);
                // ROS_INFO("neigh state = %d", node_map_[neighbor_index[0]][neighbor_index[1]]->state_);
                // ROS_INFO("neighbor_index x = %d, y = %d", neighbor_index[0], neighbor_index[1]);
                // ROS_INFO("neighbor_ptr->g_score_ = %f", node_map_[neighbor_index[0]][neighbor_index[1]]->g_score_);
  
                //超出地图范围，不扩展
                if (neighbor_index[0] < 0 || neighbor_index[1] < 0 || 
                    neighbor_index[0] >= map_size_x_ / enlargement_factor ||
                    neighbor_index[1] >= map_size_y_ / enlargement_factor) {
                    // ROS_INFO("out of map");
                    
                    continue;
                }

                //障碍物节点不扩展
                //这个函数没有
                //cout << cmap_ptr_->getCostmap()->getCost(neighbor_index[0], neighbor_index[1]) << endl;
                if (cost_[neighbor_index[0] + map_size_x_ * neighbor_index[1]] >= obs_thread &&
                    cost_[neighbor_index[0] + map_size_x_ * neighbor_index[1]] != costmap_2d::NO_INFORMATION) {
                    
                    // ROS_INFO("find obs");
                        
                    continue;
                } 

                // if(node_map_[neighbor_index[0]][neighbor_index[1]]->state_ != 0){
                //     // occupied
                //     // ROS_INFO("find obs");

                //     continue;
                // }

                neighbor_ptr = node_map_[neighbor_index[0]][neighbor_index[1]];
                double edge_cost = sqrt(pow(neughbor_pos[0] - cur_node->position_[0], 2) + 
                                        pow(neughbor_pos[1] - cur_node->position_[1], 2));
                
                double obs_cost = calc_obs_cost(neighbor_index);
                //未处理过的
                if (neighbor_ptr->g_score_ == inf) {
                    neighbor_ptr->g_score_ = cur_node->g_score_ + edge_cost + obs_cost; 
                    neighbor_ptr->f_score_ = neighbor_ptr->g_score_ + heu_weight_ * getHeu(neighbor_ptr->position_, goal_pos);
                    neighbor_ptr->parent_ = cur_node;

                    // ROS_INFO("openlist push");

                    open_list_.push(neighbor_ptr);

                } else {
                    if (neighbor_ptr->g_score_ > cur_node->g_score_ +  edge_cost + obs_cost) {
                        neighbor_ptr->g_score_ = cur_node->g_score_ +  edge_cost + obs_cost;
                        neighbor_ptr->f_score_ = neighbor_ptr->g_score_ + heu_weight_ * getHeu(neighbor_ptr->position_, goal_pos);
                        neighbor_ptr->parent_ = cur_node;
                     
                        // ROS_INFO("openlist update");


                    } else {
                        // ROS_INFO("other");
                        continue;
                    }
                }
            }
        }
    }

    if (find) {
        ros::Time time2 = ros::Time::now();
        double time = (time2 - time1).toSec();
        ROS_WARN("Time in Astar searching is %f", time);
        // getPath(plan);
        // visPath(path);
        keyPath = getKeyPoint();
        generateCorridor();

        // visKeyPath(plan);
        std::vector<geometry_msgs::PoseStamped> AstarPlan, KeyPointPlan;
        nav_msgs::Path path, keypath;
        for(const auto pos : keyPath){
            geometry_msgs::PoseStamped pose = goal;
            
            double x = pos[0];
            double y = pos[1];
            
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;

            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            ROS_INFO("x = %f y = %f", x, y);
            path.poses.push_back(pose);
            plan.push_back(pose);
        }
        path.header.frame_id = plan[0].header.frame_id;
        path.header.stamp = plan[0].header.stamp;
        
        _plan_pub.publish(path);  

        visCorridors(false);

        // for(const auto pos : path_node_){
        //     geometry_msgs::PoseStamped pose = goal;
            
        //     double x = pos->position_[0];
        //     double y = pos->position_[1];
            
        //     pose.pose.position.x = x;
        //     pose.pose.position.y = y;
        //     pose.pose.position.z = 0.0;

        //     pose.pose.orientation.x = 0.0;
        //     pose.pose.orientation.y = 0.0;
        //     pose.pose.orientation.z = 0.0;
        //     pose.pose.orientation.w = 1.0;

        //     ROS_INFO("x = %f y = %f", x, y);
        //     path.poses.push_back(pose);
        //     plan.push_back(pose);

        // }

        ROS_INFO("path published");

        return true;
    }

    ROS_INFO("search failed!");
    if(it >= max_it)
        ROS_INFO("out of time");

    return false;

}


Eigen::Vector2i AstarGlobalPlanner::posToIndex(Eigen::Vector2d position) {
    Eigen::Vector2i index1;
    index1[0] = (int)((position[0] - origin_[0]) / (enlargement_factor * resolution_));
    index1[1] = (int)((position[1] - origin_[1]) / (enlargement_factor * resolution_));
    return index1;
    
}

Eigen::Vector2d AstarGlobalPlanner::indexToPos(Eigen::Vector2i index) {
    Vector2d pos;
    pos[0] = origin_[0] + (index[0] + 0.5) * enlargement_factor * resolution_;
    pos[1] = origin_[1] + (index[1] + 0.5) * enlargement_factor * resolution_;
    return pos;
}

double AstarGlobalPlanner::getHeu(Eigen::Vector2d cur, Eigen::Vector2d goal) {

    double heu;
    double dx, dy, min;
    switch (heu_method_)
    {
    case EUCLIDEAN:
        heu = tie_breaker_ * (goal - cur).norm();
        // cout << "欧式距离启发函数" << endl;
        break;
    case MANHATTAN:
        heu = tie_breaker_ * (fabs(goal[0] - cur[0]) + fabs(goal[1] - cur[1]));
        break;
    case DIAGONAL:
        dx = fabs(goal[0] - cur[0]);
        dy = fabs(goal[1] - cur[1]);
        min = std::min(dx, dy);
        if (dx == min) {
            heu = tie_breaker_ * (dy - dx + sqrt(2) * dx);
        } 
        if (dy == min) {
            heu = tie_breaker_ * (dx - dy + sqrt(2) * dy);
        }
        break;
    
    default:
        heu = 0;
        // cout << "dijkstra" << endl;
        break;
    }

    return heu;
}

void AstarGlobalPlanner::traceback(Node *goal_node) {
    Node *cur_node = goal_node;
    path_node_.clear();
    path_node_.push_back(cur_node);

    while (cur_node->parent_) {
        cur_node = cur_node->parent_;
        path_node_.push_back(cur_node);
    }

    reverse(path_node_.begin(), path_node_.end());
}
void AstarGlobalPlanner::getPath(std::vector<geometry_msgs::PoseStamped>& plan) {
    std::vector<Eigen::Vector2d> path;
    for (auto node : path_node_) {
        path.push_back(node->position_);
    }
    path_node_.clear(); //不断选起点和终点
    // cout << "size:" << path.size() << endl;

    nav_msgs::Path nav_path;
    nav_path.header.frame_id = _frame_id;
    nav_path.header.stamp = ros::Time::now();
    
    geometry_msgs::PoseStamped pos;
    for (const auto pose : path) {
        pos.pose.position.x = pose[0];
        pos.pose.position.y = pose[1];

        nav_path.poses.push_back(pos);
        plan.push_back(pos);
    }
    _plan_pub.publish(nav_path);
}

void AstarGlobalPlanner::visPath(std::vector<Eigen::Vector2d> path) {
    
    nav_msgs::Path nav_path;
    nav_path.header.frame_id = _frame_id;
    nav_path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pos;
    for (const auto pose : path) {
        pos.pose.position.x = pose[0];
        pos.pose.position.y = pose[1];
        nav_path.poses.push_back(pos);
    }
    _plan_pub.publish(nav_path);
}

void AstarGlobalPlanner::resetNodeMap() {
    for (int i = 0; i < map_size_x_/enlargement_factor; ++i) {
        for (int j = 0; j < map_size_y_/enlargement_factor; ++j) {
            
            node_map_[i][j]->g_score_ = inf;
            node_map_[i][j]->f_score_ = 0;
            node_map_[i][j]->parent_ = nullptr;
            node_map_[i][j]->state_ = -1;
        }

    }
}

void AstarGlobalPlanner::pubGirdmap(){
    int width = map_size_x_ / enlargement_factor;
    int height = map_size_y_ / enlargement_factor;
    ROS_INFO("map_size_x_ = %d, map_size_y = %d", map_size_x_, map_size_y_);
    
    ROS_INFO("before grid map size w = %d, h = %d", width, height);

    nav_msgs::OccupancyGrid my_map;

    my_map.info.width = width;
    my_map.info.height = width;
    my_map.info.map_load_time = ros::Time(0);
    my_map.info.resolution = resolution_ * enlargement_factor;

    my_map.info.origin.position.x = origin_[0];
    my_map.info.origin.position.y = origin_[1];
    my_map.info.origin.position.z = 0;

    my_map.info.origin.orientation.x = 0;
    my_map.info.origin.orientation.y = 0;
    my_map.info.origin.orientation.z = 0;

    my_map.header.frame_id = "map";

    for(int i = 0; i < width; i++){
        for(int j = 0; j < height; j++){
            int num = node_map_[i][j]->state_;
            if(num < 0)
                my_map.data.push_back(-1);
            else if(num > 0)
                my_map.data.push_back(100);
            else
                my_map.data.push_back(0);
        }
    }
    ROS_INFO("grid map size w = %d, h = %d", width, height);
    _gridmap_pub.publish(my_map);
}

bool AstarGlobalPlanner::checkIdx(Eigen::Vector2i idx){
    if(node_map_ == nullptr)
        return false;
    
    int num = node_map_[idx[0]][idx[1]]->state_;

    if(num >= 0 && num <= obs_thread)
        return true;
    return false;
}

double AstarGlobalPlanner::calc_obs_cost(Eigen::Vector2i idx){
    // for(int i = -2; i <= 2; i++){
    //     for(int j = -2; j <= 2; j++){
    //         if(i == 0 && j == 0)
    //             continue;

    //         Vector2i neighbor_index;
    //         neighbor_index << idx[0] + i, idx[1] + j;
    //         Vector2d neughbor_pos = indexToPos(neighbor_index);

    //         //超出地图范围，不扩展
    //         if (neighbor_index[0] < 0 || neighbor_index[1] < 0 || 
    //             neighbor_index[0] >= map_size_x_ / enlargement_factor ||
    //             neighbor_index[1] >= map_size_y_ / enlargement_factor) {
    //             // ROS_INFO("out of map");
                
    //             continue;
    //         }
    //         if(cost_[neighbor_index[0] + map_size_x_ * neighbor_index[1]] != 0 &&
    //             cost_[neighbor_index[0] + map_size_x_ * neighbor_index[1]] != costmap_2d::NO_INFORMATION)
    //             // return 10.0 * sqrt((double)x*x + (double)y*y);
    //             return 100.0;
    //     }
    // }    

    return cost_[ idx[0] + map_size_x_ * idx[1] ] / 10;
}


std::vector<Eigen::Vector2d> interpolation(std::vector<Eigen::Vector2d> &keypath){
    int n = keypath.size();
    std::vector<Eigen::Vector2d> path;
    for(int i = 0; i < n-1; i++){ 
        double dis = sqrt( (keypath[i][1] - keypath[i+1][1]) * (keypath[i][1] - keypath[i+1][1])
                      + (keypath[i][0] - keypath[i+1][0]) * (keypath[i][0] - keypath[i+1][0]) );
         
        double disX = sqrt( (keypath[i][0] - keypath[i+1][0]) * (keypath[i][0] - keypath[i+1][0]) );
        double disY = sqrt( (keypath[i][1] - keypath[i+1][1]) * (keypath[i][1] - keypath[i+1][1]) );
        
        double seg = dis / 0.05;
        
        double subX = disX / seg, subY = disY / seg;

        double startX = keypath[i][0];
        double startY = keypath[i][1];
        
        for(int j = 0; j < seg; j++){
            double x = startX + j * subX; 
            double y = startY + j * subY;

            Eigen::Vector2d interPoint(x, y);
            path.push_back(interPoint); 
            ROS_INFO("interpolation x = %f, y = %f", interPoint[0], interPoint[1]);
        }
    }
    return path;
}

std::vector<Eigen::Vector2d> AstarGlobalPlanner::getKeyPoint(){
    std::vector<Node*> keyPoint;

    int n = path_node_.size();

    // for (auto node : path_node) {
    //     // keypath.push_back(node->position);
    //     ROS_INFO("get path = %d, %d", node->index[0], node->index[1]);
    // }

    keyPoint.push_back(path_node_.front());
    
    Node *startNode = path_node_.front();
    
    ROS_INFO("n = %d", n);

    for(int i = 0; i < n-1; ){
        startNode = path_node_[i];

        for(int j = n-1; j >= i; j--){
            Node *endNode = path_node_[j];

            if(isClear(startNode, endNode)){

                keyPoint.push_back(endNode);
                i = j;
                break;
            }
        }
    }

    std::vector<Eigen::Vector2d> keypath;
    for (auto node : keyPoint) {
        keypath.push_back(node->position_);

        ROS_INFO("get key path = %f, %f", node->position_[0], node->position_[1]);
    }
    // path_node.clear(); //不断选起点和终点

    // keypath = interpolation(keypath);

    for (auto node : keypath) {
        // ROS_INFO("get interpolation key path = %f, %f", node[0], node[1]);
    }
    return keypath;
}

bool AstarGlobalPlanner::isClear(Node *start, Node *end){
    if(start == end)
        return true;
    
    int startX = std::min(start->index_[0], end->index_[0]);
    int startY = std::min(start->index_[1], end->index_[1]);

    int endX = std::max(start->index_[0], end->index_[0]);
    int endY = std::max(start->index_[1], end->index_[1]);

    for(int i = startX; i <= endX; i++){
        for(int j = startY; j <= endY; j++){
            if(cost_[i + j * map_size_x_] >= obs_thread)
                return false;
        }
    }
    return true;
}

// void AstarGlobalPlanner::visPath(){

// }
// void visKeyPath(std::vector<geometry_msgs::PoseStamped> &plan)
void AstarGlobalPlanner::visKeyPath(std::vector<geometry_msgs::PoseStamped> &plan){
    // std::vector<geometry_msgs::PoseStamped> plan;
    nav_msgs::Path path;

    for(const auto pos : keyPath){
        geometry_msgs::PoseStamped pose;

        double x = pos[0];
        double y = pos[1];
        
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        // ROS_INFO("x = %f y = %f", x, y);
        plan.push_back(pose);
    }

    path.header.frame_id = plan[0].header.frame_id;
    path.header.stamp = plan[0].header.stamp;
        
    _plan_pub.publish(path);  
}

// 根据关键点生成安全走廊
// 走廊保存在corridors_中；
void AstarGlobalPlanner::generateCorridor(){
    // 隐藏（清除）上条走廊
    visCorridors(true);
    corridors_.clear();

    int n = keyPath.size();
    for(int i = 0; i < n-1; i++){
        // 找到矩形的四个点的位置
        Eigen::Vector2d start = keyPath[i];
        Eigen::Vector2d end = keyPath[i+1];
        
        ROS_INFO("corr[%d] x = %f y = %f", i, start[0], start[1]);

        double xmin = std::min(start[0], end[0]);
        double xmax = std::max(start[0], end[0]);
        double ymin = std::min(start[1], end[1]);
        double ymax = std::max(start[1], end[1]);

        std::vector<Eigen::Vector2d> corr;
        corr.push_back(Eigen::Vector2d(xmin, ymin));
        corr.push_back(Eigen::Vector2d(xmax, ymin));
        corr.push_back(Eigen::Vector2d(xmax, ymax));
        corr.push_back(Eigen::Vector2d(xmin, ymax));

        corridors_.push_back(corr);
    }
}

// 可视化走廊
// bool conceal 是否隐藏上条轨迹
void AstarGlobalPlanner::visCorridors(bool conceal){
    visualization_msgs::MarkerArray MarkerArray;//定义MarkerArray对象

    int num = corridors_.size();
    for(int i = 0; i < num; i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = _frame_id;
        marker.header.stamp = ros::Time();
        marker.ns = "corridors";
        marker.id = i;

        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        if(conceal)
            marker.color.a = 0.0;
        else
            marker.color.a = 1.0;

        marker.color.r = 0.0;
        marker.scale.x = 0.1;
        geometry_msgs:: Point p[4];
        for(int j = 0; j < 4; j++){
            p[j].x = corridors_[i][j][0];
            p[j].y = corridors_[i][j][1];
            p[j].z = 0;
            // ROS_INFO("plan corr[%d] x = %f, y = %f",i, corridors_[i][j][0], corridors_[i][j][1]);
        }
        // ROS_INFO("\n");

        // 每两个点连一条线，共需要8个点生成4条线
        for(int j = 0; j < 3; j++){
            marker.points.push_back(p[j]);
            marker.points.push_back(p[j+1]);
        }

        // 这是左下和左上点连成的线
        marker.points.push_back(p[3]);
        marker.points.push_back(p[0]);
        // marker.points.push_back(p[i]);
        MarkerArray.markers.push_back(marker);
    }
    _corr_pub.publish(MarkerArray);
}

struct corrExpander{
    Eigen::Vector2i center;
    bool can_expand = true;
    Eigen::Vector2i dir;

    Eigen::Vector2i start;
    Eigen::Vector2i expand_delta;

    Eigen::Vector2i boundary;

    int len, dep;

    corrExpander(Eigen::Vector2i center, Eigen::Vector2i dir, Eigen::Vector2i );
};

// std::vector<Eigen::Vector2d> AstarGlobalPlanner::corrExpand(Eigen::Vector2i center){
//     // bool dir[4];
//     std::vector<std::pair<int, int>> dir;
//     dir[0].first = -1;
//     dir[0].second = -1;

//     dir[1].first = 1;
//     dir[1].second = -1;

//     dir[2].first = 1;
//     dir[2].second = 1;

//     dir[3].first = -1;
//     dir[3].second = 1;
    
//     bool can_expand = true;
//     while(can_expand){
//         int dep = 1, len = dep + 1;

//         for(int i = 0; i < 4; i++){
//             if(dir[i].first == 0){
//                 can_expand = false;
//                 continue;
//             }
//             can_expand = true;
//             int layer_start_x = center[0] + dir[i].first * (dep-1);
//             int layer_start_y = center[1] + dir[i].second * (dep);
//             // Eigen::Vector2i  = center[]
//             for(int j = 0; j < len; j++){

//             }


//         }
//     }
    
// }

};