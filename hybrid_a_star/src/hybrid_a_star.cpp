#include <hybrid_a_star/hybrid_a_star.h>
#include <hybrid_a_star/display_tools.h>
#include <hybrid_a_star/timer.h>
#include <hybrid_a_star/trajectory_optimizer.h>

using common::math::Vec2d;

HybridAStar::HybridAStar(double steering_angle, int steering_angle_discrete_num, double segment_length,
                         int segment_length_discrete_num, double wheel_base, double steering_penalty,
                         double reversing_penalty, double steering_change_penalty, double shot_distance,
                         int grid_size_phi) {
    /*
    前向模拟分为两次循环迭代
    1. 离散车辆的转向角度steering_radian_。共2*steering_discrete_num_次，一次steering_radian_step_size_弧度
    2. 离散该转向角度下的车辆向前的距离segment_length_。共segment_length_discrete_num_次，一次move_step_size_长度
    

    */
    wheel_base_ = wheel_base;
    segment_length_ = segment_length; 
    steering_radian_ = steering_angle * M_PI / 180.0; // angle to radian
    steering_discrete_num_ = steering_angle_discrete_num;
    steering_radian_step_size_ = steering_radian_ / steering_discrete_num_;
    move_step_size_ = segment_length / segment_length_discrete_num;
    segment_length_discrete_num_ = static_cast<int>(segment_length_discrete_num);
    steering_penalty_ = steering_penalty;
    steering_change_penalty_ = steering_change_penalty;
    reversing_penalty_ = reversing_penalty;
    shot_distance_ = shot_distance;

    CHECK_EQ(static_cast<float>(segment_length_discrete_num_ * move_step_size_), static_cast<float>(segment_length_))
        << "The segment length must be divisible by the step size. segment_length: "
        << segment_length_ << " | step_size: " << move_step_size_;

    rs_path_ptr_ = std::make_shared<RSPath>(wheel_base_ / std::tan(steering_radian_));//转弯半径
    tie_breaker_ = 1.0 + 1e-3;

    STATE_GRID_SIZE_PHI_ = grid_size_phi;
    ANGULAR_RESOLUTION_ = 360.0 / STATE_GRID_SIZE_PHI_ * M_PI / 180.0;
}

HybridAStar::~HybridAStar() {
    ReleaseMemory();
}

void HybridAStar::Init(double x_lower, double x_upper, double y_lower, double y_upper,
                       double state_grid_resolution, double map_grid_resolution) {
    // SetVehicleShape(4.7, 2.0, 1.3);
    SetVehicleShape(2.44, 0.82, 1.22);
    lstcube.clear();
    map_x_lower_ = x_lower;
    map_x_upper_ = x_upper;
    map_y_lower_ = y_lower;
    map_y_upper_ = y_upper;
    STATE_GRID_RESOLUTION_ = state_grid_resolution;
    MAP_GRID_RESOLUTION_ = map_grid_resolution;

    STATE_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / STATE_GRID_RESOLUTION_);
    STATE_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / STATE_GRID_RESOLUTION_);

    MAP_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / MAP_GRID_RESOLUTION_);
    MAP_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / MAP_GRID_RESOLUTION_);
    ROS_WARN("map size configuration parameters :");
    std::cout << "the resolution of this picture(m/pixel): " << STATE_GRID_RESOLUTION_ << std::endl;
    std::cout << "the resolution of this map(m/m): " << MAP_GRID_RESOLUTION_ << std::endl;
    std::cout << "the width of real world is(m): " << MAP_GRID_SIZE_X_ << std::endl;
    std::cout << "the heigh of real world is(m): " << MAP_GRID_SIZE_Y_ << std::endl;
    std::cout << "the total pixel width of picture is(pixel): " << STATE_GRID_SIZE_X_ << std::endl;
    std::cout << "the total pixel heigh of picture is(pixel): " << STATE_GRID_SIZE_Y_ << std::endl;
    std::cout << "the x of origin in this picture(m): " << map_x_lower_ << std::endl;
    std::cout << "the y of origin in this picture(m): " << map_y_lower_ << std::endl;
    std::cout << "the x of up bound in this picture(m): " << map_x_upper_ << std::endl;
    std::cout << "the y of up bound in this picture(m): " << map_y_upper_ << std::endl;
    std::cout << "--------------------***----------------------- " << std::endl;
    if (map_data_) {
        delete[] map_data_;
        map_data_ = nullptr;
    }

    map_data_ = new uint8_t[MAP_GRID_SIZE_X_ * MAP_GRID_SIZE_Y_];

    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {

            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }
            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }

        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }

    state_node_map_ = new StateNode::Ptr **[STATE_GRID_SIZE_X_];
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
        state_node_map_[i] = new StateNode::Ptr *[STATE_GRID_SIZE_Y_];
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
            state_node_map_[i][j] = new StateNode::Ptr[STATE_GRID_SIZE_PHI_];
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                state_node_map_[i][j][k] = nullptr;
            }
        }
    }
}

inline bool HybridAStar::LineCheck(double x0, double y0, double x1, double y1) {
    bool steep = (std::abs(y1 - y0) > std::abs(x1 - x0));

    if (steep) {
        std::swap(x0, y0);
        std::swap(y1, x1);
    }

    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    auto delta_x = x1 - x0;
    auto delta_y = std::abs(y1 - y0);
    auto delta_error = delta_y / delta_x;
    decltype(delta_x) error = 0;
    decltype(delta_x) y_step;
    auto yk = y0;

    if (y0 < y1) {
        y_step = 1;
    } else {
        y_step = -1;
    }

    auto N = static_cast<unsigned int>(x1 - x0);
    for (unsigned int i = 0; i < N; ++i) {
        if (steep) {
            if (HasObstacle(Vec2i(yk, x0 + i * 1.0))
                || BeyondBoundary(Eigen::Vector2d(yk * MAP_GRID_RESOLUTION_,
                                        (x0 + i) * MAP_GRID_RESOLUTION_))
                    ) {
                return false;
            }
        } else {
            if (HasObstacle(Vec2i(x0 + i * 1.0, yk))
                || BeyondBoundary(Eigen::Vector2d((x0 + i) * MAP_GRID_RESOLUTION_,
                                        yk * MAP_GRID_RESOLUTION_))
                    ) {
                return false;
            }
        }

        error += delta_error;
        if (error >= 0.5) {
            yk += y_step;
            error = error - 1.0;
        }
    }

    return true;
}

bool HybridAStar::CheckCollision(const double &x, const double &y, const double &theta) {
    Timer timer;
    Mat2d R;
    R << std::cos(theta), -std::sin(theta),
            std::sin(theta), std::cos(theta);

    MatXd transformed_vehicle_shape;
    transformed_vehicle_shape.resize(8, 1);
    for (unsigned int i = 0; i < 4u; ++i) {
        transformed_vehicle_shape.block<2, 1>(i * 2, 0)
                = R * vehicle_shape_.block<2, 1>(i * 2, 0) + Eigen::Vector2d(x, y);
    }

    Vec2i transformed_pt_index_0 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(0, 0)
    );
    Vec2i transformed_pt_index_1 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(2, 0)
    );

    Vec2i transformed_pt_index_2 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(4, 0)
    );

    Vec2i transformed_pt_index_3 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(6, 0)
    );

    double y1, y0, x1, x0;
    // pt1 -> pt0
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_1.x());
    y1 = static_cast<double>(transformed_pt_index_1.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // pt2 -> pt1
    x0 = static_cast<double>(transformed_pt_index_1.x());
    y0 = static_cast<double>(transformed_pt_index_1.y());
    x1 = static_cast<double>(transformed_pt_index_2.x());
    y1 = static_cast<double>(transformed_pt_index_2.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // pt3 -> pt2
    x0 = static_cast<double>(transformed_pt_index_2.x());
    y0 = static_cast<double>(transformed_pt_index_2.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // pt0 -> pt3
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!LineCheck(x0, y0, x1, y1)) {
        return false;
    }

    check_collision_use_time += timer.End();
    num_check_collision++;
    return true;
}

bool HybridAStar::HasObstacle(const int grid_index_x, const int grid_index_y) const {
    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_
            && grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_
            && (map_data_[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1));
}

bool HybridAStar::HasObstacle(const Vec2i &grid_index) const {//map index check
    int grid_index_x = grid_index[0];
    int grid_index_y = grid_index[1];

    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_
            && grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_
            && (map_data_[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1));
}

void HybridAStar::SetObstacle(unsigned int x, unsigned int y) {
    if (x < 0u || x > static_cast<unsigned int>(MAP_GRID_SIZE_X_)
        || y < 0u || y > static_cast<unsigned int>(MAP_GRID_SIZE_Y_)) {
        return;
    }

    map_data_[x + y * MAP_GRID_SIZE_X_] = 1;
}

void HybridAStar::SetObstacle(const double pt_x, const double pt_y) {
    if (pt_x < map_x_lower_ || pt_x > map_x_upper_ ||
        pt_y < map_y_lower_ || pt_y > map_y_upper_) {
        return;
    }

    int grid_index_x = static_cast<int>((pt_x - map_x_lower_) / MAP_GRID_RESOLUTION_);
    int grid_index_y = static_cast<int>((pt_y - map_y_lower_) / MAP_GRID_RESOLUTION_);

    map_data_[grid_index_x + grid_index_y * MAP_GRID_SIZE_X_] = 1;
}

void HybridAStar::SetVehicleShape(double length, double width, double rear_axle_dist) {
    vehicle_shape_.resize(8);
    vehicle_shape_.block<2, 1>(0, 0) = Eigen::Vector2d(-rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(2, 0) = Eigen::Vector2d(length - rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(4, 0) = Eigen::Vector2d(length - rear_axle_dist, -width / 2);
    vehicle_shape_.block<2, 1>(6, 0) = Eigen::Vector2d(-rear_axle_dist, -width / 2);

    const double step_size = move_step_size_;
    const auto N_length = static_cast<unsigned int>(length / step_size);
    const auto N_width = static_cast<unsigned int> (width / step_size);
    vehicle_shape_discrete_.resize(2, (N_length + N_width) * 2u);

    const Eigen::Vector2d edge_0_normalized = (vehicle_shape_.block<2, 1>(2, 0)
                                     - vehicle_shape_.block<2, 1>(0, 0)).normalized();
    for (unsigned int i = 0; i < N_length; ++i) {
        vehicle_shape_discrete_.block<2, 1>(0, i + N_length)
                = vehicle_shape_.block<2, 1>(4, 0) - edge_0_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, i)
                = vehicle_shape_.block<2, 1>(0, 0) + edge_0_normalized * i * step_size;
    }

    const Eigen::Vector2d edge_1_normalized = (vehicle_shape_.block<2, 1>(4, 0)
                                     - vehicle_shape_.block<2, 1>(2, 0)).normalized();
    for (unsigned int i = 0; i < N_width; ++i) {
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i)
                = vehicle_shape_.block<2, 1>(2, 0) + edge_1_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i + N_width)
                = vehicle_shape_.block<2, 1>(6, 0) - edge_1_normalized * i * step_size;
    }
}

__attribute__((unused)) Eigen::Vector2d HybridAStar::CoordinateRounding(const Eigen::Vector2d &pt) const {
    return MapGridIndex2Coordinate(Coordinate2MapGridIndex(pt));
}

Eigen::Vector2d HybridAStar::MapGridIndex2Coordinate(const Vec2i &grid_index) const {
    Eigen::Vector2d pt;
    pt.x() = ((double) grid_index[0] + 0.5) * MAP_GRID_RESOLUTION_ + map_x_lower_;
    pt.y() = ((double) grid_index[1] + 0.5) * MAP_GRID_RESOLUTION_ + map_y_lower_;

    return pt;
}

Vec3i HybridAStar::State2Index(const Vec3d &state) const {//连续状态转离散状态 state resolution
    Vec3i index;

    index[0] = std::min(std::max(int((state[0] - map_x_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_X_ - 1);
    index[1] = std::min(std::max(int((state[1] - map_y_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_Y_ - 1);
    index[2] = std::min(std::max(int((state[2] - (-M_PI)) / ANGULAR_RESOLUTION_), 0), STATE_GRID_SIZE_PHI_ - 1);

    return index;
}

Vec2i HybridAStar::Coordinate2MapGridIndex(const Eigen::Vector2d &pt) const { // map resolution
    Vec2i grid_index;

    grid_index[0] = int((pt[0] - map_x_lower_) / MAP_GRID_RESOLUTION_);
    grid_index[1] = int((pt[1] - map_y_lower_) / MAP_GRID_RESOLUTION_);
    return grid_index;
}

void HybridAStar::GetNeighborNodes(const StateNode::Ptr &curr_node_ptr,
                                   std::vector<StateNode::Ptr> &neighbor_nodes) {
    neighbor_nodes.clear();
    // 1. 离散车辆的转向角度steering_radian_。共2*steering_discrete_num_次，一次steering_radian_step_size_弧度
    // 2. 离散该转向角度下的车辆向前的距离segment_length_。共segment_length_discrete_num_次，一次move_step_size_长度

    for (int i = -steering_discrete_num_; i <= steering_discrete_num_; ++i) {
        VectorVec3d intermediate_state;
        bool has_obstacle = false;

        double x = curr_node_ptr->state_.x();
        double y = curr_node_ptr->state_.y();
        double theta = curr_node_ptr->state_.z();

        const double phi = i * steering_radian_step_size_;

        // forward
        for (int j = 1; j <= segment_length_discrete_num_; j++) {
            DynamicModel(move_step_size_, phi, x, y, theta);
            intermediate_state.emplace_back(Vec3d(x, y, theta));

            if (!CheckCollision(x, y, theta)) {
                has_obstacle = true;
                break;
            }
        }

        Vec3i grid_index = State2Index(intermediate_state.back());
        if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {
            auto neighbor_forward_node_ptr = new StateNode(grid_index);
            neighbor_forward_node_ptr->intermediate_states_ = intermediate_state;
            neighbor_forward_node_ptr->state_ = intermediate_state.back();
            neighbor_forward_node_ptr->steering_grade_ = i;
            neighbor_forward_node_ptr->direction_ = StateNode::FORWARD;
            neighbor_nodes.push_back(neighbor_forward_node_ptr);
        }

        // backward
        has_obstacle = false;
        intermediate_state.clear();
        x = curr_node_ptr->state_.x();
        y = curr_node_ptr->state_.y();
        theta = curr_node_ptr->state_.z();
        for (int j = 1; j <= segment_length_discrete_num_; j++) {
            DynamicModel(-move_step_size_, phi, x, y, theta);
            intermediate_state.emplace_back(Vec3d(x, y, theta));

            if (!CheckCollision(x, y, theta)) {
                has_obstacle = true;
                break;
            }
        }

        if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {
            grid_index = State2Index(intermediate_state.back());
            auto neighbor_backward_node_ptr = new StateNode(grid_index);
            neighbor_backward_node_ptr->intermediate_states_ = intermediate_state;
            neighbor_backward_node_ptr->state_ = intermediate_state.back();
            neighbor_backward_node_ptr->steering_grade_ = i;
            neighbor_backward_node_ptr->direction_ = StateNode::BACKWARD;
            neighbor_nodes.push_back(neighbor_backward_node_ptr);
        }
    }
}

void HybridAStar::DynamicModel(const double &step_size, const double &phi,
                               double &x, double &y, double &theta) const {
    x = x + step_size * std::cos(theta);
    y = y + step_size * std::sin(theta);
    theta = Mod2Pi(theta + step_size / wheel_base_ * std::tan(phi));
}

double HybridAStar::Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);
  /*θ= 
        θ+2π, −2π<θ<−π
        θ−2π, π≤θ<2π
        θ, −π≤θ<π
        而且C++中的反三角函数给出的结果映射到了[-pi, pi]
 */
      if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }
    
    return v;
}

bool HybridAStar::BeyondBoundary(const Eigen::Vector2d &pt) const { // map coradinate check
    return pt.x() < map_x_lower_ || pt.x() > map_x_upper_ || pt.y() < map_y_lower_ || pt.y() > map_y_upper_;
}

double HybridAStar::ComputeH(const StateNode::Ptr &current_node_ptr,
                             const StateNode::Ptr &terminal_node_ptr) {
    double h;
    // L2
//    h = (current_node_ptr->state_.head(2) - terminal_node_ptr->state_.head(2)).norm();

    // L1
    h = (current_node_ptr->state_.head(2) - terminal_node_ptr->state_.head(2)).lpNorm<1>();

    if (h < 3.0 * shot_distance_) {
        h = rs_path_ptr_->Distance(current_node_ptr->state_.x(), current_node_ptr->state_.y(),
                                   current_node_ptr->state_.z(),
                                   terminal_node_ptr->state_.x(), terminal_node_ptr->state_.y(),
                                   terminal_node_ptr->state_.z());
    }

    return h;
}

double HybridAStar::ComputeG(const StateNode::Ptr &current_node_ptr,
                             const StateNode::Ptr &neighbor_node_ptr) const {
    double g;//转向惩罚+与前一次转向方向不同惩罚+倒退惩罚
    if (neighbor_node_ptr->direction_ == StateNode::FORWARD) {
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * steering_change_penalty_;
            } else {
                g = segment_length_ * steering_change_penalty_ * steering_penalty_;
            }
        } else {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_;
            } else {
                g = segment_length_ * steering_penalty_;
            }
        }
    } else {
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * steering_change_penalty_ * 100*reversing_penalty_;
            } else {
                g = segment_length_ * steering_change_penalty_ * steering_penalty_ * 100*reversing_penalty_;
            }
        } else {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * 100*reversing_penalty_;
            } else {
                g = segment_length_ * steering_penalty_ * 100*reversing_penalty_;
            }
        }
    }

    return g;
}

bool HybridAStar::Search(const Vec3d &start_state, const Vec3d &goal_state) {
    Timer search_used_time;//定义开始时间

    double neighbor_time = 0.0, compute_h_time = 0.0, compute_g_time = 0.0;

    const Vec3i start_grid_index = State2Index(start_state);
    const Vec3i goal_grid_index = State2Index(goal_state);

    auto goal_node_ptr = new StateNode(goal_grid_index);
    goal_node_ptr->state_ = goal_state;
    goal_node_ptr->direction_ = StateNode::NO;
    goal_node_ptr->steering_grade_ = 0;

    auto start_node_ptr = new StateNode(start_grid_index);
    start_node_ptr->state_ = start_state;
    start_node_ptr->steering_grade_ = 0;
    start_node_ptr->direction_ = StateNode::NO;
    start_node_ptr->node_status_ = StateNode::IN_OPENSET;
    start_node_ptr->intermediate_states_.emplace_back(start_state);
    start_node_ptr->g_cost_ = 0.0;
    start_node_ptr->f_cost_ = ComputeH(start_node_ptr, goal_node_ptr);

    state_node_map_[start_grid_index.x()][start_grid_index.y()][start_grid_index.z()] = start_node_ptr;
    state_node_map_[goal_grid_index.x()][goal_grid_index.y()][goal_grid_index.z()] = goal_node_ptr;

    openset_.clear();
    openset_.insert(std::make_pair(0, start_node_ptr));

    std::vector<StateNode::Ptr> neighbor_nodes_ptr;
    StateNode::Ptr current_node_ptr;
    StateNode::Ptr neighbor_node_ptr;

    int count = 0;
    while (!openset_.empty()) {
        current_node_ptr = openset_.begin()->second;
        current_node_ptr->node_status_ = StateNode::IN_CLOSESET;
        openset_.erase(openset_.begin());

        if ((current_node_ptr->state_.head(2) - goal_node_ptr->state_.head(2)).norm() <= shot_distance_) {
            double rs_length = 0.0;
            if (AnalyticExpansions(current_node_ptr, goal_node_ptr, rs_length)) {
                terminal_node_ptr_ = goal_node_ptr;

                StateNode::Ptr grid_node_ptr = terminal_node_ptr_->parent_node_;
                while (grid_node_ptr != nullptr) {
                    grid_node_ptr = grid_node_ptr->parent_node_;
                    path_length_ = path_length_ + segment_length_;
                }
                path_length_ = path_length_ - segment_length_ + rs_length;

                std::cout << "ComputeH use time(ms): " << compute_h_time << std::endl;
                std::cout << "check collision use time(ms): " << check_collision_use_time << std::endl;
                std::cout << "GetNeighborNodes use time(ms): " << neighbor_time << std::endl;
                std::cout << "average time of check collision(ms): "
                          << check_collision_use_time / num_check_collision
                          << std::endl;
                ROS_INFO("\033[1;32m --> Time in Hybrid A star is %f ms, path length: %f  \033[0m\n",
                         search_used_time.End(), path_length_);

                check_collision_use_time = 0.0;
                num_check_collision = 0.0;
                return true;
            }
        }

        Timer timer_get_neighbor;
        GetNeighborNodes(current_node_ptr, neighbor_nodes_ptr);
        neighbor_time = neighbor_time + timer_get_neighbor.End();

        for (unsigned int i = 0; i < neighbor_nodes_ptr.size(); ++i) {
            neighbor_node_ptr = neighbor_nodes_ptr[i];

            Timer timer_compute_g;
            const double neighbor_edge_cost = ComputeG(current_node_ptr, neighbor_node_ptr);
            compute_g_time = compute_g_time + timer_get_neighbor.End();

            Timer timer_compute_h;
            // const double current_h = ComputeH(current_node_ptr, goal_node_ptr) * tie_breaker_;
            //因为前项运动距离大于栅格对角线长度，所以不需要考虑共占栅格问题
            const double neighbor_h = ComputeH(neighbor_node_ptr, goal_node_ptr) ;//应该计算每次邻居节点到终点的h
            compute_h_time = compute_h_time + timer_compute_h.End();

            const Vec3i &index = neighbor_node_ptr->grid_index_;
            if (state_node_map_[index.x()][index.y()][index.z()] == nullptr) {//该临节点所在状态栅格未被扩展
                neighbor_node_ptr->g_cost_ = current_node_ptr->g_cost_ + neighbor_edge_cost;
                neighbor_node_ptr->parent_node_ = current_node_ptr;
                neighbor_node_ptr->node_status_ = StateNode::IN_OPENSET;
                neighbor_node_ptr->f_cost_ = neighbor_node_ptr->g_cost_ + neighbor_h;
                openset_.insert(std::make_pair(neighbor_node_ptr->f_cost_, neighbor_node_ptr));
                state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
                continue;
            } else if (state_node_map_[index.x()][index.y()][index.z()]->node_status_ == StateNode::IN_OPENSET) {
                double g_cost_temp = current_node_ptr->g_cost_ + neighbor_edge_cost;

                if (state_node_map_[index.x()][index.y()][index.z()]->g_cost_ > g_cost_temp) {
                    neighbor_node_ptr->g_cost_ = g_cost_temp;
                    neighbor_node_ptr->f_cost_ = g_cost_temp + neighbor_h;
                    neighbor_node_ptr->parent_node_ = current_node_ptr;
                    neighbor_node_ptr->node_status_ = StateNode::IN_OPENSET;

                    delete state_node_map_[index.x()][index.y()][index.z()];//删除原来记录在此栅格的节点，因为其Intermediate状态和最新的Intermediate状态不同，方向也可能不同
                    state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
                } else {
                    delete neighbor_node_ptr;
                }
                continue;
            } else if (state_node_map_[index.x()][index.y()][index.z()]->node_status_ == StateNode::IN_CLOSESET) {
                delete neighbor_node_ptr;
                continue;
            }
        }

        count++;
        if (count > 50000) {
            ROS_WARN("Exceeded the number of iterations, the search failed");
            return false;
        }
    }

    return false;
}

VectorVec4d HybridAStar::GetSearchedTree() {
    VectorVec4d tree;
    Vec4d point_pair;

    visited_node_number_ = 0;
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                if (state_node_map_[i][j][k] == nullptr || state_node_map_[i][j][k]->parent_node_ == nullptr) {
                    continue;
                }

                const unsigned int number_states = state_node_map_[i][j][k]->intermediate_states_.size() - 1;
                for (unsigned int l = 0; l < number_states; ++l) {
                    point_pair.head(2) = state_node_map_[i][j][k]->intermediate_states_[l].head(2);
                    point_pair.tail(2) = state_node_map_[i][j][k]->intermediate_states_[l + 1].head(2);

                    tree.emplace_back(point_pair);
                }

                point_pair.head(2) = state_node_map_[i][j][k]->intermediate_states_[0].head(2);
                point_pair.tail(2) = state_node_map_[i][j][k]->parent_node_->state_.head(2);
                tree.emplace_back(point_pair);
                visited_node_number_++;
            }
        }
    }

    return tree;
}

void HybridAStar::ReleaseMemory() {
    if (map_data_ != nullptr) {
        delete[] map_data_;
        map_data_ = nullptr;
    }

    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }

                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }

            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }

        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }

    terminal_node_ptr_ = nullptr;
}

__attribute__((unused)) double HybridAStar::GetPathLength() const {
    return path_length_;
}

VectorVec3d HybridAStar::GetPath() const {
    VectorVec3d path;

    std::vector<StateNode::Ptr> temp_nodes;

    StateNode::Ptr state_grid_node_ptr = terminal_node_ptr_;
    while (state_grid_node_ptr != nullptr) {
        temp_nodes.emplace_back(state_grid_node_ptr);
        state_grid_node_ptr = state_grid_node_ptr->parent_node_;
    }

    std::reverse(temp_nodes.begin(), temp_nodes.end());
    for (const auto &node: temp_nodes) {
        path.insert(path.end(), node->intermediate_states_.begin(),
                    node->intermediate_states_.end());
    }

    return path;
}

void HybridAStar::Reset() {
    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
            }
        }
    }

    path_length_ = 0.0;
    terminal_node_ptr_ = nullptr;
}

bool HybridAStar::AnalyticExpansions(const StateNode::Ptr &current_node_ptr,
                                     const StateNode::Ptr &goal_node_ptr, double &length) {
    VectorVec3d rs_path_poses = rs_path_ptr_->GetRSPath(current_node_ptr->state_,
                                                        goal_node_ptr->state_,
                                                        move_step_size_, length);

    for (const auto &pose: rs_path_poses)
        if (BeyondBoundary(pose.head(2)) || !CheckCollision(pose.x(), pose.y(), pose.z())) {
            return false;
        };

    goal_node_ptr->intermediate_states_ = rs_path_poses;
    goal_node_ptr->parent_node_ = current_node_ptr;

    auto begin = goal_node_ptr->intermediate_states_.begin();
    goal_node_ptr->intermediate_states_.erase(begin);

    return true;
}

std::pair<VecCube, VecCube> HybridAStar::corridorGeneration(const VectorVec3d &path_coord, int segment){
    VecCube SmoothPathcubeList;
    VecCube bsplinecubeList;
    Vec3d pt;
    lstcube.clear();
    for (int i = 0; i < (int)path_coord.size(); i++)
    {
        pt = path_coord[i];

        Cube cube = generateCube(pt);
        auto result = inflateCube(cube, lstcube);
        if((0 == i) || (path_coord.size() - 1 == i) || (1 == i % 2)) {
            bsplinecubeList.push_back(result.first);
        }
        if(result.second == false)  // 当前路径点膨胀一次之后的cube被上一个cube完全包含,对该点进行剪枝
            continue;

        cube = result.first;
        
        lstcube = cube;
        SmoothPathcubeList.push_back(cube);
    }
    return {SmoothPathcubeList, bsplinecubeList};
}
VecCube HybridAStar::corridorGeneration(const VectorVec3d &path_coord){
    VecCube cubeList;
    Vec3d pt;
    lstcube.clear();
    for (int i = 0; i < (int)path_coord.size(); i++)
    {
        pt = path_coord[i];

        Cube cube = generateCube(pt);
        auto result = inflateCube(cube, lstcube);

        if(result.second == false)  // 当前路径点膨胀一次之后的cube被上一个cube完全包含,对该点进行剪枝
            continue;

        cube = result.first;
        // cube.printBox();
        lstcube = cube;
        cubeList.push_back(cube);
    }
    return cubeList;
}

Cube HybridAStar::generateCube(Vec3d pt){
    /*
           P4-----------P3 
           /           /           /y 
          /           /           /    
        P1-----------P2          /      
                                /--------> x               
                                
                                  
                  
*/       
    Cube cube;
    cube.index = 0;
    pt(0) = std::max(std::min(pt(0), map_x_upper_), map_x_lower_); //_pt_min_x < pt < _pt_max_x
    pt(1) = std::max(std::min(pt(1), map_y_upper_), map_y_lower_);
    //pt(2) = max(min(pt(2), _pt_max_z), _pt_min_z);


    cube.center = pt;

    double x_u = pt(0);
    double x_l = pt(0);
    
    double y_u = pt(1);
    double y_l = pt(1);
    
    // double z_u = pc_coord(2);
    // double z_l = pc_coord(2);

    // 将cube初始化为一个点
    cube.vertex.row(0) = Vec3d(x_l, y_l, 0);  
    cube.vertex.row(1) = Vec3d(x_u, y_l, 0);  
    cube.vertex.row(2) = Vec3d(x_u, y_u, 0);  
    cube.vertex.row(3) = Vec3d(x_l, y_u, 0); 
    // cube.vertex.row(0) = Vec3d(x_u, y_l, 0);  
    // cube.vertex.row(1) = Vec3d(x_u, y_u, 0);  
    // cube.vertex.row(2) = Vec3d(x_l, y_u, 0);  
    // cube.vertex.row(3) = Vec3d(x_l, y_l, 0); 
    // cube.vertex.row(0) = Vector3d(x_u, y_l, z_u);  
    // cube.vertex.row(1) = Vector3d(x_u, y_u, z_u);  
    // cube.vertex.row(2) = Vector3d(x_l, y_u, z_u);  
    // cube.vertex.row(3) = Vector3d(x_l, y_l, z_u);  
    // cube.vertex.row(4) = Vector3d(x_u, y_l, z_l);  
    // cube.vertex.row(5) = Vector3d(x_u, y_u, z_l);  
    // cube.vertex.row(6) = Vector3d(x_l, y_u, z_l);  
    // cube.vertex.row(7) = Vector3d(x_l, y_l, z_l);  

    return cube;
}

std::pair<Cube, bool> HybridAStar::inflateCube(const Cube &cube, const Cube &lstcube){
    Cube cubeMax = cube;

    // Inflate sequence: right, left, front, back, above, below                                                                              
    Eigen::MatrixXi vertex_idx = Eigen::MatrixXi::Zero(8,3);
    
    // 判断当前的路径点是否触碰障碍,因为传入的cube是一个点
    for (int i = 0; i < 4; i++)
    { 
        double coord_x = std::max(std::min(cube.vertex(i, 0), map_x_upper_), map_x_lower_);
        double coord_y = std::max(std::min(cube.vertex(i, 1), map_y_upper_), map_y_lower_);
        double coord_z = cube.vertex(i, 2);
        // double coord_z = max(min(cube.vertex(i, 2), _pt_max_z), _pt_min_z);
        Vec3d coord(coord_x, coord_y, coord_z);
        Vec2i map_pt_index = Coordinate2MapGridIndex(coord.head(2));
        
        if( HasObstacle(map_pt_index) || BeyondBoundary(coord.head(2)))
        {       
            ROS_ERROR("[Planning Node] path has node in obstacles or Beyond Boundary!");
            return std::make_pair(cubeMax, false);
        }
        
        vertex_idx.row(i).head(2) = map_pt_index;  // 若未触碰障碍,将该点的x, y, z坐标赋值给vertex_idx的对应行,等待膨胀
    }

    int id_x, id_y, id_z;

    /*
               P4------------P3 
               /|           /|              ^
              / |          / |              | z
            P1--|---------P2 |              |
             |  P8--------|--p7             |
             | /          | /               /--------> y
             |/           |/               /  
            P5------------P6              / x
    */           

    bool collide;

    Eigen::MatrixXi vertex_idx_lst = vertex_idx;   // 存储未膨胀前cube的idx

    // 依次将cube的某个面(例如P1-P4-P8-P5)向对应的坐标轴方向扩展_step_length, 并检查这个面是否触碰障碍物
    int iter = 0;
    
    while(iter < _max_inflate_iter) // 迭代次数也就是最大扩展距离
    {   
        // Y Axis
        int y_lo = std::max(0, vertex_idx(1, 1) - _step_length);
        int y_up = std::min(MAP_GRID_SIZE_Y_, vertex_idx(2, 1) + _step_length);

        // Y+ now is the right side : (p2 -- p3 -- p7 -- p6) face
        // ############################################################################################################
        collide = false;
        Eigen::Vector2d tem_pos;
        for(id_y = vertex_idx(2, 1); id_y <= y_up; id_y++ )
        {   
            if( collide == true) 
                break;
            
            for(id_x = vertex_idx(2, 0); id_x >= vertex_idx(3, 0); id_x-- ) // P2P3
            {
                tem_pos = MapGridIndex2Coordinate({id_x,id_y});
                bool occupy = ( HasObstacle( (int64_t)id_x, (int64_t)id_y) || BeyondBoundary(tem_pos)); 
                if(occupy) // the voxel is occupied
                {   
                    collide = true;
                    break;
                }

            }
        }

        if(collide)
        {
            vertex_idx(2, 1) = std::max(id_y-2, vertex_idx(2, 1));   // _step_length = 1, 若有障碍说明之前cube就已经到达边界 //cause when collide = true, id_y++ run one more time
            vertex_idx(3, 1) = std::max(id_y-2, vertex_idx(3, 1));   // 此时id_y = y_up+1
            // vertex_idx(6, 1) = std::max(id_y-2, vertex_idx(6, 1));   // max函数的意义不明 -- is mean the most lowst value is vertex_idx(6, 1)
            // vertex_idx(5, 1) = std::max(id_y-2, vertex_idx(5, 1));
        }
        else
            vertex_idx(2, 1) = vertex_idx(3, 1)  = id_y - 1;  // for循环后id_y = y_up+1
            // vertex_idx(1, 1) = vertex_idx(2, 1) = vertex_idx(6, 1) = vertex_idx(5, 1) = id_y - 1;  // for循环后id_y = y_up+1


        // Y- now is the left side : (p1 -- p4 -- p8 -- p5) face 
        // ############################################################################################################
        collide  = false;

        for(id_y = vertex_idx(1, 1); id_y >= y_lo; id_y-- ) 
        {   
            if( collide == true)   // 退出多层for循环
                break;
            
            for(id_x = vertex_idx(1, 0); id_x >= vertex_idx(0, 0); id_x-- ) // P1P4
            {    
                if( collide == true) 
                    break;
                tem_pos = MapGridIndex2Coordinate({id_x,id_y});
                bool occupy = ( HasObstacle( (int64_t)id_x, (int64_t)id_y) || BeyondBoundary(tem_pos));  
                if(occupy) // the voxel is occupied
                {   
                    collide = true;
                    break;
                }

            }
        }

        if(collide)
        {
            vertex_idx(1, 1) = std::min(id_y+2, vertex_idx(1, 1));
            vertex_idx(0, 1) = std::min(id_y+2, vertex_idx(0, 1));
            // vertex_idx(7, 1) = std::min(id_y+2, vertex_idx(7, 1));
            // vertex_idx(4, 1) = std::min(id_y+2, vertex_idx(4, 1));
        }
        else
            vertex_idx(1, 1) = vertex_idx(0, 1)  = id_y + 1;   
            // vertex_idx(0, 1) = vertex_idx(3, 1) = vertex_idx(7, 1) = vertex_idx(4, 1) = id_y + 1;   
        


        // X Axis
        int x_lo = std::max(0, vertex_idx(0, 0) - _step_length);
        int x_up = std::min(MAP_GRID_SIZE_X_, vertex_idx(1, 0) + _step_length);
        // X + now is the front side : (p1 -- p2 -- p6 -- p5) face
        // ############################################################################################################

        collide = false;
        for(id_x = vertex_idx(1, 0); id_x <= x_up; id_x++ )
        {   
            if( collide == true) 
                break;
            
            for(id_y = vertex_idx(1, 1); id_y <= vertex_idx(2, 1); id_y++ ) // P1P2
            {
                if( collide == true) 
                    break;
                tem_pos = MapGridIndex2Coordinate({id_x,id_y});
                bool occupy = ( HasObstacle( (int64_t)id_x, (int64_t)id_y) || BeyondBoundary(tem_pos));      
                if(occupy) // the voxel is occupied
                {   
                    collide = true;
                    break;
                }

            }
        }

        if(collide)
        {
            vertex_idx(1, 0) = std::max(id_x-2, vertex_idx(1, 0)); 
            vertex_idx(2, 0) = std::max(id_x-2, vertex_idx(2, 0)); 
            // vertex_idx(5, 0) = std::max(id_x-2, vertex_idx(5, 0)); 
            // vertex_idx(4, 0) = std::max(id_x-2, vertex_idx(4, 0)); 
        }
        else
            vertex_idx(1, 0) = vertex_idx(2, 0) = id_x - 1;    
            // vertex_idx(0, 0) = vertex_idx(1, 0) = vertex_idx(5, 0) = vertex_idx(4, 0) = id_x - 1;    

        // X- now is the back side : (p4 -- p3 -- p7 -- p8) face
        // ############################################################################################################
        collide = false;
        for(id_x = vertex_idx(0, 0); id_x >= x_lo; id_x-- )
        {   
            if( collide == true) 
                break;
            
            for(id_y = vertex_idx(0, 1); id_y <= vertex_idx(3, 1); id_y++ ) // P4P3
            {
                if( collide == true) 
                    break;
                tem_pos = MapGridIndex2Coordinate({id_x,id_y});
                bool occupy = ( HasObstacle( (int64_t)id_x, (int64_t)id_y) || BeyondBoundary(tem_pos));   
                if(occupy) // the voxel is occupied
                {   
                    collide = true;
                    break;
                }
                
            }
        }

        if(collide)
        {
            vertex_idx(0, 0) = std::min(id_x+2, vertex_idx(0, 0)); 
            vertex_idx(3, 0) = std::min(id_x+2, vertex_idx(3, 0)); 
            // vertex_idx(6, 0) = std::min(id_x+2, vertex_idx(6, 0)); 
            // vertex_idx(7, 0) = std::min(id_x+2, vertex_idx(7, 0)); 
        }
        else
            vertex_idx(0, 0) = vertex_idx(3, 0) = id_x + 1;
            // vertex_idx(3, 0) = vertex_idx(2, 0) = vertex_idx(6, 0) = vertex_idx(7, 0) = id_x + 1;

        if(vertex_idx_lst == vertex_idx)  // 膨胀one step,前后无变化,达到最大状态,跳出循环
            break;

        vertex_idx_lst = vertex_idx;

        Eigen::MatrixXd vertex_coord = Eigen::MatrixXd::Zero(8,3);
        for(int i = 0; i < 4; i++)
        {   
            int index_x = std::max(std::min(vertex_idx(i, 0), MAP_GRID_SIZE_X_ - 1), 0);  // 这里为什么是_max_x_id-1和0? --control point size
            int index_y = std::max(std::min(vertex_idx(i, 1), MAP_GRID_SIZE_Y_ - 1), 0);
            // int index_z = max(min(vertex_idx(i, 2), _max_z_id - 1), 0);

            Eigen::Vector2i index(index_x, index_y);
            Eigen::Vector2d pos = MapGridIndex2Coordinate(index);
            vertex_coord.row(i).head(2) = pos;
        }

        // 使用vertex_idx继续迭代, 这里vertex_coord只是为了计算cubeMax(每次迭代后的cube)
        cubeMax.setVertex(vertex_coord, MAP_GRID_RESOLUTION_);  // 将从collision_map->GridIndexToLocation(index)获得的顶点pos放入栅格中心???????????
        if( isContains(lstcube, cubeMax))  // 剪枝
            return std::make_pair(lstcube, false);

        iter ++;
    }

    return std::make_pair(cubeMax, true);   // 膨胀前后无变化,则达到最大状态    

}

bool HybridAStar::isContains(const Cube &cube1, const Cube &cube2){
    if( (cube1.vertex(1, 0) >= cube2.vertex(1, 0)) && (cube1.vertex(1, 1) <= cube2.vertex(1, 1)) &&
        (cube1.vertex(3, 0) <= cube2.vertex(3, 0)) && (cube1.vertex(3, 1) >= cube2.vertex(3, 1)) )
        return true;
    else
        return false; 
}

void HybridAStar::timeAllocation(std::vector<Cube> & corridor, const Vec3d& start_pt_, const Vec3d& end_pt_){
    std::vector<Vec3d> points;
    points.push_back (start_pt_);
    // 计算出的corridor的特点:第一个cube的边界点为第二个cube的center
    for(int i = 1; i < (int)corridor.size(); i++)
        points.push_back(corridor[i].center);

    points.push_back (end_pt_);
    double _Vel = _MAX_Vel * 0.6;
    double _Acc = _MAX_Acc * 0.6;

    for (int k = 0; k < (int)points.size(); k++)
    {
        double dtxyz;
        Vec3d p0 = points[k];
        Vec3d p1 = points[k + 1];
        Vec3d d = p1 - p0;
        Vec3d v0(0.0, 0.0, 0.0);

        // if (k == 0)
        //     v0 = _start_vel;    // _start_vel从odom中获得,为起始点的速度

        double D = d.norm();    // 相邻两点的距离
        double V0 = v0.dot(d / D);  // V0的含义???  V0 > 0:速度方向与目标点方向相同, V0 < 0:速度方向与目标点方向相反
        double aV0 = fabs(V0);

        double acct = (_Vel - V0) / _Acc * ((_Vel > V0) ? 1 : -1);  // 加速时间
        double accd = V0 * acct + (_Acc * acct * acct / 2) * ((_Vel > V0) ? 1 : -1);  // 加速位移
        double dcct = _Vel / _Acc;  // 减速时间
        double dccd = _Acc * dcct * dcct / 2;  // 减速位移

        if (D < aV0 * aV0 / (2 * _Acc))    // 两点之间距离小于加速距离, 这行写错了吧???, 测试结果:一直不执行
        {
            double t1 = (V0 < 0) ? 2.0 * aV0 / _Acc : 0.0;
            double t2 = aV0 / _Acc;
            dtxyz = t1 + t2;
        }
        else if (D < accd + dccd)    // 两点之间距离小于加速距离+减速距离
        {
            double t1 = (V0 < 0) ? 2.0 * aV0 / _Acc : 0.0;
            double t2 = (-aV0 + sqrt(aV0 * aV0 + _Acc * D - aV0 * aV0 / 2)) / _Acc;
            double t3 = (aV0 + _Acc * t2) / _Acc;
            dtxyz = t1 + t2 + t3;  
        }
        else    // 正常情况,两点之间距离=加速距离+匀速距离+减速距离
        {
            double t1 = acct;
            double t2 = (D - accd - dccd) / _Vel;
            double t3 = dcct;
            dtxyz = t1 + t2 + t3;
        }
        corridor[k].t = dtxyz;  // 一共points.size()-1条轨迹
    }
}

bool HybridAStar::getOccupancy(const Vec3d& pos){
    if(BeyondBoundary(pos.head(2))) return -1;

    Vec2i id = Coordinate2MapGridIndex(pos.head(2));
    return HasObstacle(id);
}
bool HybridAStar::getOccupancy(const Eigen::Vector2d& pos){
    if(BeyondBoundary(pos)) return -1;

    Vec2i id = Coordinate2MapGridIndex(pos);
    return HasObstacle(id);
}

//将Hybrid A*计算的轨迹结果，按照行驶的正反方向切换，分割为数段，
//并完善轨迹的静态、动态信息
bool HybridAStar::TrajectoryPartition(
    const HybridAStartResult& result,
    std::vector<HybridAStartResult>* partitioned_result) {
  const auto& x = result.x;
  const auto& y = result.y;
  const auto& phi = result.phi;
  if (x.size() != y.size() || x.size() != phi.size()) {
    std::cout << "states sizes are not equal when do trajectory partitioning of "
              "Hybrid A Star result" << std::endl;
    return false;
  }

  size_t horizon = x.size();
  partitioned_result->clear();
  partitioned_result->emplace_back();
  auto* current_traj = &(partitioned_result->back());
  double heading_angle = phi.front();
  const Vec2d init_tracking_vector(x[1] - x[0], y[1] - y[0]);
  double tracking_angle = init_tracking_vector.Angle();
  bool current_gear =
      std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
      (M_PI_2);
//此时的result只有路径静态信息，x,y,phi
//将Hybrid A*计算的轨迹结果，按照行驶的正反方向切换，分割为数段
  for (size_t i = 0; i < horizon - 1; ++i) {
    heading_angle = phi[i];
    const Vec2d tracking_vector(x[i + 1] - x[i], y[i + 1] - y[i]);
    tracking_angle = tracking_vector.Angle();
    bool gear =
        std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
        (M_PI_2);
    if (gear != current_gear) {
      current_traj->x.push_back(x[i]);
      current_traj->y.push_back(y[i]);
      current_traj->phi.push_back(phi[i]);
      partitioned_result->emplace_back();
      current_traj = &(partitioned_result->back());
      current_gear = gear;
    }
    current_traj->x.push_back(x[i]);
    current_traj->y.push_back(y[i]);
    current_traj->phi.push_back(phi[i]);
  }
  current_traj->x.push_back(x.back());
  current_traj->y.push_back(y.back());
  current_traj->phi.push_back(phi.back());

  const auto start_timestamp = std::chrono::system_clock::now();

  // Retrieve v, a and steer from path
  for (auto& result : *partitioned_result) {
    //根据result中的静态信息x,y,phi，利用相邻点、逐点求动态信息v,a,steer
    if (!GenerateSpeedAcceleration(&result)) {
    ROS_ERROR("GenerateSpeedAcceleration fail");
    return false;
    }
  }
  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  std::cout << "speed profile total time: " << diff.count() * 1000.0 <<" ms."<< std::endl;
  return true;
}

bool HybridAStar::GenerateSpeedAcceleration(HybridAStartResult* result) {
  // Sanity Check
  delta_t_ = 0.5;
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    ROS_ERROR("result size check when generating speed and acceleration fail"); 
    return false;
  }
  const size_t x_size = result->x.size();

  // load velocity from position
  // initial and end speed are set to be zeros
  result->v.push_back(0.0);
  for (size_t i = 1; i + 1 < x_size; ++i) {
    double discrete_v = (((result->x[i + 1] - result->x[i]) / delta_t_) * // delta_t_ = 0.5
                             std::cos(result->phi[i]) +
                         ((result->x[i] - result->x[i - 1]) / delta_t_) *
                             std::cos(result->phi[i])) /
                            2.0 +  //上面是x方向上，利用连续3点的坐标求中间点的速度，平均速度
                        (((result->y[i + 1] - result->y[i]) / delta_t_) *
                             std::sin(result->phi[i]) +
                         ((result->y[i] - result->y[i - 1]) / delta_t_) *
                             std::sin(result->phi[i])) /
                            2.0;  //上面是y方向上，利用连续3点的坐标求中间点的速度，平均速度
    result->v.push_back(discrete_v);
  }
  result->v.push_back(0.0);

  // load acceleration from velocity
  for (size_t i = 0; i + 1 < x_size; ++i) {
    const double discrete_a = (result->v[i + 1] - result->v[i]) / delta_t_;
    result->a.push_back(discrete_a);
  }

  // load steering from phi
  for (size_t i = 0; i + 1 < x_size; ++i) {
    double discrete_steer = (result->phi[i + 1] - result->phi[i]) *
                            wheel_base_ / move_step_size_;
    if (result->v[i] > 0.0) {
      discrete_steer = std::atan(discrete_steer);
    } else {
      discrete_steer = std::atan(-discrete_steer);
    }
    result->steer.push_back(discrete_steer);
  }
  return true;
}

