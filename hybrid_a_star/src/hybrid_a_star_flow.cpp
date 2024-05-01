

// #include "hybrid_a_star/hybrid_a_star_flow.h"
#include <hybrid_a_star/hybrid_a_star_flow.h>
#include <hybrid_a_star/timer.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>


// HybridAStarFlow::HybridAStarFlow() { }
HybridAStarFlow::~HybridAStarFlow() { std::cout << "des HybridAStarFlow" << std::endl; }
double Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);

    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}


HybridAStarFlow::HybridAStarFlow(ros::NodeHandle &nh) {
    double steering_angle = nh.param("planner/steering_angle", 10);
    int steering_angle_discrete_num = nh.param("planner/steering_angle_discrete_num", 1);
    double wheel_base = nh.param("planner/wheel_base", 1.0);
    double segment_length = nh.param("planner/segment_length", 1.6);
    int segment_length_discrete_num = nh.param("planner/segment_length_discrete_num", 8);
    double steering_penalty = nh.param("planner/steering_penalty", 1.05);
    double steering_change_penalty = nh.param("planner/steering_change_penalty", 1.5);
    double reversing_penalty = nh.param("planner/reversing_penalty", 2.0);
    double shot_distance = nh.param("planner/shot_distance", 5.0);
    map_resolution = nh.param("planner/map_resolution",0.2);//比例尺 1 : 5
    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybridAStar>(
            steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base,
            steering_penalty, reversing_penalty, steering_change_penalty, shot_distance
    );
    costmap_sub_ptr_ = std::make_shared<CostMapSubscriber>(nh, "/map", 1);
    init_pose_sub_ptr_ = std::make_shared<InitPoseSubscriber2D>(nh, "/initialpose", 1);
    goal_pose_sub_ptr_ = std::make_shared<GoalPoseSubscriber2D>(nh, "/move_base_simple/goal", 1);

    path_pub_ = nh.advertise<nav_msgs::Path>("searched_path", 1);
    searched_tree_pub_ = nh.advertise<visualization_msgs::Marker>("searched_tree", 1);
    vehicle_path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vehicle_path", 1);
    way_point_pub_ = nh.advertise<visualization_msgs::MarkerArray>("way_points",1);
    corridor_pub_ = nh.advertise<visualization_msgs::MarkerArray>("corridor_vis", 1);

    has_map_ = false; 
    has_endPt_ = false;
    has_startPt_ = false;
    hybrid_AStar_plan_ = false;
}

bool HybridAStarFlow::Run() {
    ReadData();

    if (!has_map_) { //load costmap
        if (costmap_deque_.empty()) {
            return hybrid_AStar_plan_;
        }

        current_costmap_ptr_ = costmap_deque_.front();//draw out the first costmap
        costmap_deque_.pop_front();

        
        kinodynamic_astar_searcher_ptr_->Init(
                current_costmap_ptr_->info.origin.position.x,//地图原点所在位置（即像素(0,0)在世界坐标系的坐标）,单位m
                1.0 * current_costmap_ptr_->info.width * current_costmap_ptr_->info.resolution,
                current_costmap_ptr_->info.origin.position.y,
                1.0 * current_costmap_ptr_->info.height * current_costmap_ptr_->info.resolution,
                current_costmap_ptr_->info.resolution, //state_grid_resolution
                map_resolution
        );

        unsigned int map_w = std::floor(current_costmap_ptr_->info.width / map_resolution);//地图栅格在碰撞检测中才会使用
        unsigned int map_h = std::floor(current_costmap_ptr_->info.height / map_resolution);

        for (unsigned int w = 0; w < map_w; ++w) { // map is world map. scale up from a picture
            for (unsigned int h = 0; h < map_h; ++h) {
                auto x = static_cast<unsigned int> ((w + 0.5) * map_resolution
                                                    / current_costmap_ptr_->info.resolution);
                auto y = static_cast<unsigned int> ((h + 0.5) * map_resolution
                                                    / current_costmap_ptr_->info.resolution);

                if (current_costmap_ptr_->data[y * current_costmap_ptr_->info.width + x]) { //读取栅格地图占用概率，概率越大，越有可能是障碍物
                    kinodynamic_astar_searcher_ptr_->SetObstacle(w, h);//*map_data_ = 1为有障碍物
                }
            }
        }
        std::cout<<"------------------- a map is loaded -------------------"<<std::endl;
        has_map_ = true;
    }
    costmap_deque_.clear();
    while (HasStartPose() && HasGoalPose()) {
        has_startPt_ = has_endPt_ = true;
        std::cout<<"------------------- start and end points is loaded -------------------"<<std::endl;
        //读取位置
        InitPoseData();
        //清空数据
        point_set.clear();
        path.clear();
        partition_trajectories.clear();
        corridors.clear();
        
        double start_yaw = tf::getYaw(current_init_pose_ptr_->pose.pose.orientation);
        double goal_yaw = tf::getYaw(current_goal_pose_ptr_->pose.orientation);
        
        Vec3d start_state = Vec3d(
                current_init_pose_ptr_->pose.pose.position.x,
                current_init_pose_ptr_->pose.pose.position.y,
                start_yaw
        );
        Vec3d goal_state = Vec3d(
                current_goal_pose_ptr_->pose.position.x,
                current_goal_pose_ptr_->pose.position.y,
                goal_yaw
        );
        // Vec3d start_state = Vec3d(
        //         6.6533584594726562,
        //         23.902116775512695,
        //         -1.5216081142425539
        // );
        // Vec3d goal_state = Vec3d(
        //         40.897262573242188,
        //         6.661097526550293,
        //         1.6439623832702637
        // );
        start_pt_ = start_state;
        end_pt_ = goal_state;
        if (kinodynamic_astar_searcher_ptr_->Search(start_state, goal_state)) {
            // std::cout<<"start_pt_ x "<<start_pt_[0]<<std::endl;
            // std::cout<<"start_pt_ y "<<start_pt_[1]<<std::endl;
            // std::cout<<"end_pt_ x "<<end_pt_[0]<<std::endl;
            // std::cout<<"end_pt_ y "<<end_pt_[1]<<std::endl;
            hybrid_AStar_plan_ = true;
            path = kinodynamic_astar_searcher_ptr_->GetPath();

            for (const auto &i : path){
                point_set.emplace_back(i.head(2));
            }
            Timer time_bef_corridor;
            all_corridor = kinodynamic_astar_searcher_ptr_->corridorGeneration(path);
            std::cout << "Time consume in corridor generation is: " << time_bef_corridor.End() <<" ms"<< std::endl;
            kinodynamic_astar_searcher_ptr_->timeAllocation(all_corridor,start_state,goal_state);
            PublishPath(path);
            PublishWayPoint(path);
            PublishVehiclePath(path, 0.82, 2.4, 5u);
            PublishSearchedTree(kinodynamic_astar_searcher_ptr_->GetSearchedTree());//whis is SearchedTree?
            PublishCorridor(all_corridor);
            // nav_msgs::Path path_ros;
            // geometry_msgs::PoseStamped pose_stamped;

            // for (const auto &pose: path) {
            //     pose_stamped.header.frame_id = "world";
            //     pose_stamped.pose.position.x = pose.x();
            //     pose_stamped.pose.position.y = pose.y();
            //     pose_stamped.pose.position.z = 0.0;

            //     pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.z());

            //     path_ros.poses.emplace_back(pose_stamped);
            // }

            // path_ros.header.frame_id = "world";
            // path_ros.header.stamp = ros::Time::now();
            // static tf::TransformBroadcaster transform_broadcaster;
            // for (const auto &pose: path_ros.poses) {
            //     tf::Transform transform;
            //     transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, 0.0));

            //     tf::Quaternion q;
            //     q.setX(pose.pose.orientation.x);
            //     q.setY(pose.pose.orientation.y);
            //     q.setZ(pose.pose.orientation.z);
            //     q.setW(pose.pose.orientation.w);
            //     transform.setRotation(q);

            //     transform_broadcaster.sendTransform(tf::StampedTransform(transform,
            //                                                              ros::Time::now(), "world",
            //                                                              "ground_link")
            //     );

            //     ros::Duration(0.05).sleep();

            // }

            // tf实现 车辆模型移动 --.
            // static tf::TransformBroadcaster transform_broadcaster; //动态坐标转换
            // for (const auto &pose: path) {
            //     tf::Transform transform;
            //     transform.setOrigin(tf::Vector3(pose.x(), pose.y(), 0.0));
            //     tf::Quaternion q;
            //     q.setRPY(0,0,pose.z());
            //     transform.setRotation(q);
            //     transform_broadcaster.sendTransform(tf::StampedTransform(transform,
            //                                                              ros::Time::now(), "world",
            //                                                              "ground_link")
            //     );
            //     ros::Duration(0.05).sleep();

            // }

            //tf2实现
            // static tf2_ros::TransformBroadcaster transform_broadcaster;
            // for(const auto& pose : path){
                
            //     geometry_msgs::TransformStamped tfs;
            //     tfs.header.frame_id = "world";
            //     tfs.header.stamp = ros::Time::now();
                
            //     tfs.child_frame_id = "ground_link";
            //     tfs.transform.translation.x = pose.x();
            //     tfs.transform.translation.y = pose.y();
            //     tfs.transform.translation.z = 0.0;

            //     tf2::Quaternion q;
            //     q.setRPY(0,0,pose.z());
            //     tfs.transform.rotation.x = q.getX();
            //     tfs.transform.rotation.y = q.getY();
            //     tfs.transform.rotation.z = q.getZ();
            //     tfs.transform.rotation.w = q.getW();

            //     transform_broadcaster.sendTransform(tfs);
            
            //     ros::Duration(0.05).sleep();
            // }
        //Hybrid Astar partition
            HybridAStartResult result;
            DataTransform(path, &result);
            if (!kinodynamic_astar_searcher_ptr_->TrajectoryPartition(result, &partition_trajectories)) {
                ROS_ERROR("Hybrid Astar partition failed");
                return false;
            }
            int segment = 0;
            for(const auto & trajectory : partition_trajectories){
                segment++;
                VectorVec3d post_path;
                for(int i = 0; i < trajectory.x.size(); i++){
                    post_path.push_back({trajectory.x[i], trajectory.y[i], 0});
                }
                auto temp = kinodynamic_astar_searcher_ptr_->corridorGeneration(post_path,segment);
                kinodynamic_astar_searcher_ptr_->timeAllocation(temp.second,start_state,goal_state);
                // kinodynamic_astar_searcher_ptr_->timeAllocation(temp.first,start_state,goal_state);
                corridors.push_back(temp);
            }
            // size_t size = partition_trajectories.size();
            // std::vector<Eigen::MatrixXd> xWS_vec;
            // std::vector<Eigen::MatrixXd> uWS_vec;
            // std::vector<Eigen::MatrixXd> state_result_ds_vec;
            // std::vector<Eigen::MatrixXd> control_result_ds_vec;
            // std::vector<Eigen::MatrixXd> time_result_ds_vec;
            // std::vector<Eigen::MatrixXd> l_warm_up_vec;
            // std::vector<Eigen::MatrixXd> n_warm_up_vec;
            // std::vector<Eigen::MatrixXd> dual_l_result_ds_vec;
            // std::vector<Eigen::MatrixXd> dual_n_result_ds_vec;
            // xWS_vec.resize(size);
            // uWS_vec.resize(size);
            // state_result_ds_vec.resize(size);
            // control_result_ds_vec.resize(size);
            // time_result_ds_vec.resize(size);
            // l_warm_up_vec.resize(size);
            // n_warm_up_vec.resize(size);
            // dual_l_result_ds_vec.resize(size);
            // dual_n_result_ds_vec.resize(size);

            // // In for loop
            // std::cout << "Trajectories size in smoother is " << size <<std::endl;
            // for (size_t i = 0; i < size; ++i) {
            //     LoadHybridAstarResultInEigen(&partition_trajectories[i], &xWS_vec[i],
            //                                 &uWS_vec[i]);
            //     // checking initial and ending points
            //     bool flag = 0;
            //     if (flag) {
            //         std::cout << "trajectory id: " << i <<std::endl;
            //         std::cout << "trajectory partitioned size: " << xWS_vec[i].cols() <<std::endl;
            //         std::cout << "initial point: " << xWS_vec[i].col(0).transpose() <<std::endl;
            //         std::cout << "ending point: "
            //             << xWS_vec[i].col(xWS_vec[i].cols() - 1).transpose() <<std::endl;
            //     }
            //     Eigen::MatrixXd last_time_u(2, 1);
            //     double init_v = 0.0;
            //     // Stitching point control and velocity is set for first piece of
            //     // trajectories. In the next ones, control and velocity are assumed to be
            //     // zero as the next trajectories always start from vehicle static state
            //     // if (i == 0) {
            //     //     const double init_steer = trajectory_stitching_point.steer();
            //     //     const double init_a = trajectory_stitching_point.a();
            //     //     last_time_u << init_steer, init_a;
            //     //     init_v = trajectory_stitching_point.v();
            //     // } else {
            //     //     last_time_u << 0.0, 0.0;
            //     //     init_v = 0.0;
            //     // }
            //     last_time_u << 0.0, 0.0;
            //     init_v = 0.0;
            //     // TODO(Jinyun): Further testing
            //     const auto smoother_start_timestamp = std::chrono::system_clock::now();
            //     if (!GenerateDecoupledTraj(
            //             xWS_vec[i], last_time_u(1, 0), init_v, 
            //             &state_result_ds_vec[i], &control_result_ds_vec[i],
            //             &time_result_ds_vec[i])) {
            //         std::cout << "Smoother fail at " << i << "th trajectory" <<std::endl;
            //         std::cout << i << "th trajectory size is " << xWS_vec[i].cols() <<std::endl;
            //         return false;
            //         // return Status(
            //         //     ErrorCode::PLANNING_ERROR,
            //         //     "iterative anchoring smoothing problem failed to solve");
            //     }
            //     const auto smoother_end_timestamp = std::chrono::system_clock::now();
            //     std::chrono::duration<double> smoother_diff =
            //         smoother_end_timestamp - smoother_start_timestamp;
            //     std::cout << "Open space trajectory smoothing total time: "
            //             << smoother_diff.count() * 1000.0 << " ms at the " << i
            //             << "th trajectory." <<std::endl;
            //     std::cout << "The " << i << "th trajectory pre-smoothing size is "
            //             << xWS_vec[i].cols() << "; post-smoothing size is "
            //             << state_result_ds_vec[i].cols() <<std::endl;
                    
                
                
                    
                
                // const auto smoother_end_timestamp = std::chrono::system_clock::now();
                // std::chrono::duration<double> smoother_diff =
                //     smoother_end_timestamp - smoother_start_timestamp;
                // std::cout << "Open space trajectory smoothing total time: "
                //         << smoother_diff.count() * 1000.0 << " ms at the " << i
                //         << "th trajectory." <<std::endl;
                // std::cout << "The " << i << "th trajectory pre-smoothing size is "
                //         << xWS_vec[i].cols() << "; post-smoothing size is "
                //         << state_result_ds_vec[i].cols() <<std::endl;
            // }
            // debug
    //        std::cout << "visited nodes: " << kinodynamic_astar_searcher_ptr_->GetVisitedNodesNumber() << std::endl;    
            
        }
        else{
            ROS_ERROR(" --> Hybrid A star failed! please restart.");
            hybrid_AStar_plan_ = false;
            break;
        }

        
        kinodynamic_astar_searcher_ptr_->Reset();
    }
    return hybrid_AStar_plan_;
}

void HybridAStarFlow::ReadData() {
    costmap_sub_ptr_->ParseData(costmap_deque_);
    init_pose_sub_ptr_->ParseData(init_pose_deque_);
    goal_pose_sub_ptr_->ParseData(goal_pose_deque_);

}

void HybridAStarFlow::InitPoseData() {
    current_init_pose_ptr_ = init_pose_deque_.front();
    init_pose_deque_.pop_front();

    current_goal_pose_ptr_ = goal_pose_deque_.front();
    goal_pose_deque_.pop_front();

}

bool HybridAStarFlow::HasGoalPose() {
    
    return !goal_pose_deque_.empty();
}

bool HybridAStarFlow::HasStartPose() {
    
    return !init_pose_deque_.empty();
}

void HybridAStarFlow::PublishPath(const VectorVec3d &path) {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());//创建四元数

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = timestamp_;

    path_pub_.publish(nav_path);
}

void HybridAStarFlow::PublishWayPoint(const VectorVec3d &path){
    visualization_msgs::MarkerArray way_point_array;
    for (unsigned int i = 0; i < path.size(); i++) {
        visualization_msgs::Marker way_point;

        if (i == 0) {
            way_point.action = visualization_msgs::Marker::DELETEALL;
        }

        way_point.header.frame_id = "world";
        way_point.ns = "way_points";
        way_point.header.stamp = ros::Time(0);
        way_point.type = visualization_msgs::Marker::SPHERE;
        way_point.id = i;
        way_point.scale.x = 0.1;
        way_point.scale.y = 0.1;
        way_point.scale.z = 0.1;
        way_point.color.a = 1.0;

        way_point.color.r = 1.0;
        way_point.color.b = 0.0;
        way_point.color.g = 0.0;

        way_point.pose.position.x = path[i].x();
        way_point.pose.position.y = path[i].y();
        way_point.pose.position.z = 0.0;

        way_point.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].z());
        way_point_array.markers.emplace_back(way_point);
    }
    // way_point_list.header.frame_id = "world";
    // way_point_list.ns = "way_points";
    // way_point_list.header.stamp = ros::Time::now();
    // way_point_list.action = visualization_msgs::Marker::ADD;
    // way_point_list.type = visualization_msgs::Marker::POINTS;

    // way_point_list.scale.x = 0.2;
    // way_point_list.scale.y = 0.2;

    // way_point_list.color.a = 1.0;
    // way_point_list.color.r = 0.0;
    // way_point_list.color.g = 0.0;
    // way_point_list.color.b = 0.0;

    // way_point_list.pose.orientation.w = 1.0;
    // way_point_list.pose.orientation.x = 0.0;
    // way_point_list.pose.orientation.y = 0.0;
    // way_point_list.pose.orientation.z = 0.0;
    // geometry_msgs::Point point;
    // for(const auto &way_point : path){
    //     point.x = way_point.x();
    //     point.y = way_point.y();
    //     point.z = 0.0;
    //     way_point_list.points.emplace_back(point);
    // }

    way_point_pub_.publish(way_point_array);
}

void HybridAStarFlow::PublishVehiclePath(const VectorVec3d &path, double width,
                                         double length, unsigned int vehicle_interval = 5u) {
    visualization_msgs::MarkerArray vehicle_array;

    for (unsigned int i = 0; i < path.size(); i += vehicle_interval) {
        visualization_msgs::Marker vehicle;

        if (i == 0) {
            vehicle.action = 3;
        }

        vehicle.header.frame_id = "world";
        vehicle.ns = "vehicle_prefile";
        vehicle.header.stamp = ros::Time::now();
        vehicle.type = visualization_msgs::Marker::CUBE;
        vehicle.id = static_cast<int>(i / vehicle_interval);
        vehicle.scale.x = length;
        vehicle.scale.y = width;
        vehicle.scale.z = 0.01;
        vehicle.color.a = 0.2;

        vehicle.color.r = 1.0;
        vehicle.color.b = 0.5;
        vehicle.color.g = 0.0;

        vehicle.pose.position.x = path[i].x();
        vehicle.pose.position.y = path[i].y();
        vehicle.pose.position.z = 0.0;

        vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].z());
        vehicle_array.markers.emplace_back(vehicle);
    }

    vehicle_path_pub_.publish(vehicle_array);
}

void HybridAStarFlow::PublishSearchedTree(const VectorVec4d &searched_tree) {
    visualization_msgs::Marker tree_list;
    tree_list.header.frame_id = "world";
    tree_list.header.stamp = ros::Time::now();
    tree_list.type = visualization_msgs::Marker::LINE_LIST;
    tree_list.action = visualization_msgs::Marker::ADD;
    tree_list.ns = "searched_tree";// show in rviz
    tree_list.scale.x = 0.02;

    tree_list.color.a = 1.0;
    tree_list.color.r = 0;
    tree_list.color.g = 0;
    tree_list.color.b = 0;

    tree_list.pose.orientation.w = 1.0;
    tree_list.pose.orientation.x = 0.0;
    tree_list.pose.orientation.y = 0.0;
    tree_list.pose.orientation.z = 0.0;

    geometry_msgs::Point point;
    for (const auto &i: searched_tree) {
        point.x = i.x();
        point.y = i.y();
        point.z = 0.0;
        tree_list.points.emplace_back(point);

        point.x = i.z();
        point.y = i.w();
        point.z = 0.0;
        tree_list.points.emplace_back(point);
    }

    searched_tree_pub_.publish(tree_list);
}
visualization_msgs::MarkerArray corridor_array;
void HybridAStarFlow::PublishCorridor(const std::vector<Cube> &corridor){
    for(auto & mk: corridor_array.markers) 
        mk.action = visualization_msgs::Marker::DELETE;  // 删除上一次的cube  // 删除上一次的cube
    corridor_pub_.publish(corridor_array);

    corridor_array.markers.clear();  // 和DELETE操作重复

    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.ns = "corridor";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.a = 0.08;
    mk.color.r = 0.0;
    mk.color.g = 1.0;
    mk.color.b = 1.0;

    int idx = 0;
    for(int i = 0; i < int(corridor.size()); i++)
    {   
        mk.id = idx;

        mk.pose.position.x = (corridor[i].vertex(1, 0) + corridor[i].vertex(0, 0) ) / 2.0; 
        mk.pose.position.y = (corridor[i].vertex(1, 1) + corridor[i].vertex(2, 1) ) / 2.0; 
        mk.pose.position.z = 0.0;   // 二维

        mk.scale.x = abs((corridor[i].vertex(1, 0) - corridor[i].vertex(0, 0) ));
        mk.scale.y = abs((corridor[i].vertex(2, 1) - corridor[i].vertex(1, 1) ));

        mk.scale.z = 0.1; 


        idx ++;
        corridor_array.markers.emplace_back(mk);
    }

    corridor_pub_.publish(corridor_array);
}
Eigen::Vector3d HybridAStarFlow::getStartEndDistance() {
    Eigen::Vector3d distance;
    Vec3d start_state = Vec3d(
                current_init_pose_ptr_->pose.pose.position.x,
                current_init_pose_ptr_->pose.pose.position.y,
                0
        );
        Vec3d goal_state = Vec3d(
                current_goal_pose_ptr_->pose.position.x,
                current_goal_pose_ptr_->pose.position.y,
                0
        );
    distance = start_state - goal_state;
    return distance;
}
std::vector<Eigen::Vector2d> HybridAStarFlow::getPointSet() {
    return point_set;
}

void HybridAStarFlow::DataTransform(const VectorVec3d& raw_path, HybridAStartResult* result){
    for(const auto & point : raw_path){
        result->x.emplace_back(point(0));
        result->y.emplace_back(point(1));
        result->phi.emplace_back(point(2));
    }
}

std::vector<HybridAStartResult> HybridAStarFlow::getPatitionTrajectory()  {
    return partition_trajectories;
}


// TODO(Jinyun): deprecate the use of Eigen in trajectory smoothing
void HybridAStarFlow::LoadHybridAstarResultInEigen(
    HybridAStartResult* result, Eigen::MatrixXd* xWS, Eigen::MatrixXd* uWS) {
  // load Warm Start result(horizon is timestep number minus one)
  size_t horizon = result->x.size() - 1;
  xWS->resize(4, horizon + 1);
  uWS->resize(2, horizon);
  Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->x.data(), horizon + 1);
  Eigen::VectorXd y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->y.data(), horizon + 1);
  Eigen::VectorXd phi = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->phi.data(), horizon + 1);
  Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->v.data(), horizon + 1);
  Eigen::VectorXd steer = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->steer.data(), horizon);
  Eigen::VectorXd a =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(result->a.data(), horizon);
  xWS->row(0) = std::move(x);
  xWS->row(1) = std::move(y);
  xWS->row(2) = std::move(phi);
  xWS->row(3) = std::move(v);
  uWS->row(0) = std::move(steer);
  uWS->row(1) = std::move(a);
  }

//   bool HybridAStarFlow::GenerateDecoupledTraj(
//     const Eigen::MatrixXd& xWS, const double init_a, const double init_v,
//     Eigen::MatrixXd* state_result_dc, Eigen::MatrixXd* control_result_dc,
//     Eigen::MatrixXd* time_result_dc) {
//   DiscretizedTrajectory smoothed_trajectory;
//   if (!iterative_anchoring_smoother_->Smooth(
//           xWS, init_a, init_v, obstacles_vertices_vec, &smoothed_trajectory)) {
//     return false;
//   }

//   LoadResult(smoothed_trajectory, state_result_dc, control_result_dc,
//              time_result_dc);
//   return true;
// }