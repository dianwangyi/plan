#include "corr_gen.h"

// 初始化，获取地图与地图参数
bool CorridorGenerator::init(std::string name, costmap_2d::Costmap2DROS* cmap_ptr){
    initialized_ = false;
    if(!initialized_){
        corridors_.clear();
        cmap_ptr_ = cmap_ptr;

        ros::NodeHandle private_nh("~" + name + "corr_gen");
        _corr_pub = private_nh.advertise<visualization_msgs::MarkerArray>("corridors", 100);

        //map
        origin_[0] =  cmap_ptr_->getCostmap()->getOriginX();
        origin_[1] =  cmap_ptr_->getCostmap()->getOriginY();

        resolution_ = cmap_ptr_->getCostmap()->getResolution();
        _frame_id = cmap_ptr_->getGlobalFrameID();
        map_size_x_ = cmap_ptr_->getCostmap()->getSizeInCellsX();
        map_size_y_ = cmap_ptr_->getCostmap()->getSizeInCellsY();
        ROS_INFO("map_size_x_ : %d, map_size_y : %d", map_size_x_, map_size_y_);

        Eigen::Vector2i map_size;
        map_size << map_size_x_, map_size_y_;
        Eigen::Vector2d world_size;
        world_size = indexToPos(map_size);
        world_size_x_ = world_size[0];
        world_size_y_ = world_size[1];
        map_ =  cmap_ptr_->getCostmap()->getCharMap();

        ROS_INFO("CorridorGenerator initialized successfully");
        initialized_ = true;
    }
}

/**
 * bounding box
 * P1-------P2
 * |        |
 * |        |
 * |        |
 * P4-------P3
 * 
 * */
bool CorridorGenerator::AstarPathCorr(const std::vector<Eigen::Vector2d>& keyPath, 
                                        std::vector<Bounding_box> &corridors){
    if(!initialized_){
        ROS_INFO("uninitialized");
        return false;
    }

    // 清除上一组走廊；
    visCorridors(true);
    corridors_.clear();

    int n = keyPath.size();
    corridors_.clear();

    for(int i = 0; i < n-1; i++){
        // 找到矩形的四个点的位置
        Eigen::Vector2i start = posToIndex(keyPath[i]);
        Eigen::Vector2i end = posToIndex(keyPath[i+1]);
        
        Bounding_box corr(start, end);
        corridors_.push_back(corr); 

        ROS_INFO("corr[%d] xmin = %d, xmax = %d, ymin = %d, ymax = %d", 
                        i, corr.xmin, corr.xmax, corr.ymin, corr.ymax);
    }

    for(int i = 0; i < corridors_.size(); i++){
        rectangleExpand(corridors_[i]);
    }
    visCorridors(false);
    corridors = corridors_;
}

/**
 *  bounding_box
 *  P1--------P2
 *   |        |
 *   |        |
 *   |        |
 *  P4--------P3
**/

bool CorridorGenerator::rectangleExpand(Bounding_box& box){
    // bool has
    bool has_P2_x=false,has_P2_y=false,has_P4_x=false,has_P4_y=false,stop=false;

    Eigen::Vector2i temp_P2=box.P2;
    Eigen::Vector2i temp_P4=box.P4;

    for(int i=0;i<max_inflate_iter;++i){
        // cout<<i<<endl;
        // cout<<temp_P2(0)<<" "<<temp_P2(1)<<" "<<temp_P4(0)<<" "<<temp_P4(1)<<endl;
        if(!has_P2_x||!has_P2_y){
            if(!has_P2_x){
                temp_P2(1)+=1;
                for(int j = temp_P2(0);j >= temp_P4(0); --j){
                    if(map_[temp_P2(1)*map_size_x_+j] >= 100 || temp_P2(1) >= map_size_y_){
                        has_P2_x=true;
                        break;
                    }
                }
                if(has_P2_x){
                    temp_P2(1)-=1;
                }
            }

            if(!has_P2_y){
                temp_P2(0)+=1;
                for(int j=temp_P2(1); j>=temp_P4(1); --j){
                    if(map_[j*map_size_x_+temp_P2(0)] >= 100  || temp_P2(0) >= map_size_x_){
                        has_P2_y=true;
                        break;
                    }
                }
                if(has_P2_y){
                    temp_P2(0)-=1;
                }
            }            
        }
        if(!has_P4_x||!has_P4_y){
            if(!has_P4_x){
                temp_P4(1)-=1;
                for(int j=temp_P4(0);j<=temp_P2(0);++j){
                    if(temp_P4(1)<0){
                        has_P4_x=true;
                        break;
                    }
                    if(map_[temp_P4(1)*map_size_x_+j] >= 100 || temp_P4(1)<0){
                        has_P4_x=true;
                        break;
                    }
                }
                if(has_P4_x){
                    temp_P4(1)+=1;
                }
            }
            if(!has_P4_y){
                temp_P4(0)-=1;
                for(int j=temp_P4(1);j<=temp_P2(1);++j){
                    if(temp_P4(0)<0){
                        has_P4_y=true;
                        break;
                    }
                    if(map_[j*map_size_x_+temp_P4(0)] >= 100 || temp_P4(0)<0){
                        has_P4_y=true;
                        break;
                    }
                }
                if(has_P4_y){
                    temp_P4(0)+=1;
                }
            }
        }

        if(has_P2_x && has_P2_y && has_P4_x && has_P4_y)
            break;
    }

    box.P2(1)=temp_P2(1);
    box.P1(1)=temp_P2(1);

    box.P2(0)=temp_P2(0);
    box.P3(0)=temp_P2(0);

    box.P3(1)=temp_P4(1);
    box.P4(1)=temp_P4(1);

    box.P4(0)=temp_P4(0);
    box.P1(0)=temp_P4(0);

    box.xmin = std::min(box.P2[0], box.P4[0]);
    box.xmax = std::max(box.P2[0], box.P4[0]);
    box.ymin = std::min(box.P2[1], box.P4[1]);
    box.ymax = std::max(box.P2[1], box.P4[1]);
}

/**
 *  bounding_box
 *  P1--------P2
 *   |        |
 *   |        |
 *   |        |
 *  P4--------P3
**/

// 可视化走廊
// bool conceal 是否隐藏上条轨迹
void CorridorGenerator::visCorridors(bool conceal){
    if(corridors_.empty()){
        ROS_INFO("no corridor to show");
        return;
    }

    visualization_msgs::MarkerArray MarkerArray;//定义MarkerArray对象

    int num = corridors_.size();

    double color = 0.0;
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

        marker.color.b = 0.2 + color;
        marker.color.r = 0.1 + 0.5 * color;
        color += 0.1;

        marker.scale.x = 0.05;
        geometry_msgs:: Point p[4];

        std::vector<Eigen::Vector2d> points;

        points.push_back( indexToPos(corridors_[i].P1) );
        points.push_back( indexToPos(corridors_[i].P2) );
        points.push_back( indexToPos(corridors_[i].P3) );
        points.push_back( indexToPos(corridors_[i].P4) );

        for(int j = 0; j < 4; j++){
            p[j].x = points[j][0];
            p[j].y = points[j][1];
            p[j].z = 0;

            // ROS_INFO("plan corr[%d] x = %f, y = %f",i, points[j][0], points[j][1]);
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

Eigen::Vector2i CorridorGenerator::posToIndex(Eigen::Vector2d position) {
    Eigen::Vector2i index1;
    index1[0] = (int)((position[0] - origin_[0]) / (resolution_));
    index1[1] = (int)((position[1] - origin_[1]) / (resolution_));
    return index1;
}

Eigen::Vector2d CorridorGenerator::indexToPos(Eigen::Vector2i index) {
    Eigen::Vector2d pos;
    pos[0] = origin_[0] + (index[0] + 0.5)  * resolution_;
    pos[1] = origin_[1] + (index[1] + 0.5)  * resolution_;
    return pos;
}
