// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>

namespace opt_planner
{
   
  // SECTION interfaces for setup and query

  OPTPlannerManager::OPTPlannerManager() {}

  OPTPlannerManager::~OPTPlannerManager() { std::cout << "des manager" << std::endl; }

  void OPTPlannerManager::initPlanModules(ros::NodeHandle &nh ,PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */
    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    // nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);

    local_data_.traj_id_ = 0;
    // hybridAStar_flow_.reset(new HybridAStarFlow);
    hybridAStar_flow_ = std::make_shared<HybridAStarFlow>(nh);
    // hybridAStar_flow_->init(nh); // Hybrid A* initial
    // hybridAStar_flow_.init(nh); // Hybrid A* initial

    bspline_optimizer_rebound_.reset(new BsplineOptimizer); 

    bspline_optimizer_rebound_->setParam(nh,hybridAStar_flow_);

    // bspline_optimizer_rebound_->setEnvironment(grid_map_);
    // bspline_optimizer_rebound_->a_star_.reset(new AStar);
    // bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
    plan_success_ = false;

    plan_pub_bspline=nh.advertise<nav_msgs::Path>("plan_bspline", 1);
    plan_pub_dl=nh.advertise<nav_msgs::Path>("plan_dl_path", 1);
  }

  void OPTPlannerManager::run(){
    /*** STEP 1: hybrid A* INIT ***/
    if(!hybridAStar_flow_->Run()){
      return;
    }
    // static int vis_id = 0;

    /*** STEP 2: OPTIMIZE ***/
    
    bool flag_success = false;
    if(hybridAStar_flow_->hasStartEndPt()){
      hybridAStar_flow_->resetPt();
      cout << "STEP 2: OPTIMIZE : "<< endl;
      auto patition_trajectory = hybridAStar_flow_->getPatitionTrajectory();
      const auto corridors = hybridAStar_flow_->GetCorridors();
      // const auto all_corridor = hybridAStar_flow_->GetCorridor();
      auto init_pt = hybridAStar_flow_->GetStartEnd();
      
      vector<Eigen::Vector3d> start_end_derivatives, path_smooth_set;
      Eigen::MatrixXd traj_ctrl_pts;
      double total_opt_time = 0.0;
      

      PathSmooth_set.clear();
      BSplineSmooth_set.clear();
      smooth_path_point_plot.clear();
      HybridAS_path_points_plot.clear();
      Bspline_path_points_plot.clear();

      dl_path_points_plot.clear();
      Dl_pathSmooth_set.clear();
      for(int i = 0;i<patition_trajectory.size();i++){
          if (patition_trajectory[i].x.size() != patition_trajectory[i].y.size() || patition_trajectory[i].x.size() != patition_trajectory[i].phi.size()) {
            ROS_ERROR ("states sizes are not equal when do trajectory partitioning of "
                      "Hybrid A Star result") ;
            return;
          }
          if(patition_trajectory[i].x.size() < 2){
            ROS_ERROR ("reference points size smaller than two, smoother early "
                      "returned") ;
            return;
          }
          std::cout<<"--------------"<<std::endl;
          std::cout<<"trajectory :"<< i << std::endl;
          if(patition_trajectory[i].x.size() < 4){
            for(int j = 0; j < patition_trajectory[i].x.size(); j++){
              Eigen::Vector3d point3D(patition_trajectory[i].x[j],patition_trajectory[i].y[j],patition_trajectory[i].phi[j]);
              PathSmooth_set.emplace_back(move(point3D));
            }
            ROS_WARN ("reference points size smaller than four, smoother next ") ;
            continue;
          }
          if(patition_trajectory[i].x.size() <= 5){
            ADEBUG << "interpolated_warm_start_path smaller than 4, can't enforce "
                "initial zero kappa";
            enforce_initial_kappa_ = false;
          }
          else{
            enforce_initial_kappa_ = true;
          }
          
          //path smooth
          cout<<"path smooth"<<endl;
          path_smooth_set.clear();
          Timer time_bef_pathSmooth;
          if(!PathSmoothAlgorithm(patition_trajectory[i], corridors, i, path_smooth_set)){
            ADEBUG << "PathSmoothAlgorithm "
                "fail!";
            return;
          }
          std::cout << "Time consume in path smooth is: " << time_bef_pathSmooth.End() <<" ms"<< std::endl;
          
          //bspline optimize
          vector<Eigen::Vector3d> point_set;
          double start_angle, end_angle, distance;
          int sample_count = 2; //隔sample_count-1个原始点进行采样
          start_end_derivatives.clear();
          
          getSample(patition_trajectory[i], point_set, sample_count, start_angle, end_angle, distance);
          if(point_set.size() < 4) {cout<<"point_set.size() < 4, skip!!!"<<endl;continue;} //防止参考点数量太少

          // double ts = 0.1; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
          double ts = distance > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
          // double ts =  pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 ; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
          
          //初始化起点和终点的速度和加速度
          start_end_derivatives.push_back({cos(start_angle), sin(start_angle), 0});
          start_end_derivatives.push_back({cos(end_angle), sin(end_angle), 0});
          start_end_derivatives.push_back(Eigen::Vector3d::Zero());
          start_end_derivatives.push_back(Eigen::Vector3d::Zero());
          
          Eigen::MatrixXd ctrl_pts;
          UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
          bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

          Timer time_bef_optimization;
          bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts, corridors[i].second);
          total_opt_time += time_bef_optimization.End();
          std::cout << "Time consume in optimization is: " << time_bef_optimization.End() <<" ms"<< std::endl;
          if (!flag_step_1_success)
          {
            // continous_failures_count_++;
            ROS_ERROR("OPTIMIZE failed!");
            std::cout<< "OPTIMIZE failed in iteration: " <<i<<std::endl;
            return;
          }

          UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
          
          double tn = pos.getTimeSum();
          for (double t = 0; t <= tn; t += 0.01) {
              Eigen::Vector3d pt = pos.evaluateDeBoor(t);
              
              BSplineSmooth_set.emplace_back(pt);
          }
          

          
        }
      double b_spline_length = 0;
      std::vector<std::pair<double, double>> bspline;
      for(int i = 0; i < BSplineSmooth_set.size();i++){
        if(i < BSplineSmooth_set.size() - 1){
          b_spline_length += (BSplineSmooth_set[i+1].head(2) - BSplineSmooth_set[i].head(2)).norm();
        }
        bspline.emplace_back(BSplineSmooth_set[i](0),BSplineSmooth_set[i](1));
      }
      std::cout << "length in optimization is: " << b_spline_length <<" m"<< std::endl;
      // std::vector<double> new_y(BSplineSmooth_set.size(), 0.0);
      // std::vector<double> new_x(BSplineSmooth_set.size(), 0.0);
      // std::vector<double> x_data, y_data;
      // std::vector<std::pair<double, double>> dl_path;
      // // for(int i = 0; i < BSplineSmooth_set.size(); i++){
      // //   new_x.emplace_back(BSplineSmooth_set[i][0]);
      // // }
      
      // for(int i = 0; i < PathSmooth_set.size();i++){
      //   x_data.emplace_back(PathSmooth_set[i](0));
      //   y_data.emplace_back(PathSmooth_set[i](1));
      // }
      // CubicSpline<double> my_data( x_data, y_data, 2);
      // new_y = my_data.interpolate( new_x );
      // for(int i = 0; i < new_y.size(); i++){
      //   Dl_pathSmooth_set.emplace_back(new_x[i],new_y[i],0.0);
      //   dl_path.emplace_back(new_x[i], new_y[i]);
      // }
      SetPathProfile(bspline, &Bspline_path_points_plot);
      // SetPathProfile(dl_path, &dl_path_points_plot);
      double dl_spline_length = 0;
      for(int i = 0; i < PathSmooth_set.size();i++){
        if(i < PathSmooth_set.size() - 1){
          dl_spline_length += (PathSmooth_set[i+1].head(2) - PathSmooth_set[i].head(2)).norm();
        }
        // bspline.emplace_back(PathSmooth_set[i](0),PathSmooth_set[i](1));
      }
      std::cout << "length in optimization is: " << dl_spline_length <<" m"<< std::endl;

      publishPlan_bspline(BSplineSmooth_set, plan_pub_bspline);
      publishPlan_bspline(PathSmooth_set, plan_pub_dl);
      // visualization_->displaySmoothPathList(PathSmooth_set, 0);
      PlotCurvature();
    }
    
  }
  pair<bool,bool> OPTPlannerManager::CheckGear(const struct HybridAStartResult &trajectory) {
  if(trajectory.x.size() < 2) {
    return {false, false};
  }
  double init_heading_angle = trajectory.phi[0];
  const common::math::Vec2d init_tracking_vector(trajectory.x[1] - trajectory.x[0],
                                   trajectory.y[1] - trajectory.y[0]);
  double init_tracking_angle = init_tracking_vector.Angle();

  int n  = trajectory.phi.size();

  double end_heading_angle = trajectory.phi[n -1];
  const common::math::Vec2d end_tracking_vector(trajectory.x[n-1] - trajectory.x[n -2],
                                   trajectory.y[n - 1] - trajectory.y[n - 2]);
  double end_tracking_angle = end_tracking_vector.Angle();
  bool ini_gear = std::abs(common::math::NormalizeAngle(init_tracking_angle - init_heading_angle)) <
         M_PI_2;
  bool end_gear = std::abs(common::math::NormalizeAngle(end_tracking_angle - end_heading_angle)) <
         M_PI_2;
  return {ini_gear, end_gear};
}

  void OPTPlannerManager::AdjustStartEndHeading(
      const struct HybridAStartResult &trajectory,
      vector<Eigen::Vector2d>& point_set) {
    // Set initial heading and bounds
    const double initial_heading = trajectory.phi[0];
    const double end_heading = trajectory.phi[trajectory.phi.size() - 1];

    // Adjust the point position to have heading by finite element difference of
    // the point and the other point equal to the given warm start initial or end
    // heading
    const double first_to_second_dx = point_set[1](0) - point_set[0](0);
    const double first_to_second_dy =
        point_set[1](1) - point_set[0](1);
    const double first_to_second_s =
        std::sqrt(first_to_second_dx * first_to_second_dx +
                  first_to_second_dy * first_to_second_dy);
    common::math::Vec2d first_point(point_set[0](0), point_set[0](1));
    common::math::Vec2d initial_vec(first_to_second_s, 0);
    initial_vec.SelfRotate(gear_.first ? initial_heading
                                : common::math::NormalizeAngle(initial_heading + M_PI));
    initial_vec += first_point;
    point_set[1] = Eigen::Vector2d(initial_vec.x(), initial_vec.y());

    const size_t path_size = point_set.size();
    const double second_last_to_last_dx =
        point_set[path_size - 1](0) - point_set[path_size - 2](0);
    const double second_last_to_last_dy =
        point_set[path_size - 1](1) - point_set[path_size - 2](1);
    const double second_last_to_last_s =
        std::sqrt(second_last_to_last_dx * second_last_to_last_dx +
                  second_last_to_last_dy * second_last_to_last_dy);
    common::math::Vec2d last_point(point_set[path_size - 1](0),
                    point_set[path_size - 1](1));
    common::math::Vec2d end_vec(second_last_to_last_s, 0);
    end_vec.SelfRotate(gear_.first ? common::math::NormalizeAngle(end_heading + M_PI) : end_heading);
    end_vec += last_point;
    point_set[path_size - 2] = Eigen::Vector2d(end_vec.x(), end_vec.y());
  }

  bool OPTPlannerManager::GenerateInitialBounds(const std::vector<Eigen::Vector2d> &raw_point_set,
                             std::vector<double>* initial_bounds){
    CHECK_NOTNULL(initial_bounds);
    initial_bounds->clear();
    const double kEpislon = 1e-8;
    // std::vector<double> default_bounds(raw_point_set.size(), 2);
    // *initial_bounds = std::move(default_bounds);
    // TODO(Jinyun): refine obstacle formulation and speed it up
    for (const auto& path_point : raw_point_set) {
      double min_bound = std::numeric_limits<double>::infinity();
      for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
        for (const common::math::LineSegment2d& linesegment : obstacle_linesegments) {
          min_bound =
              std::min(min_bound,
                      linesegment.DistanceTo({path_point(0), path_point(1)}));
        }
      }
      min_bound -= 0.41;
      min_bound = min_bound < kEpislon ? 0.0 : min_bound;
      initial_bounds->push_back(min_bound);
    }
    return true;
  }


  bool OPTPlannerManager::SmoothPath(const std::vector<Eigen::Vector2d> &raw_point_set,const std::vector<double>& bounds,
  std::vector<common::PathPoint>& smoothed_path_points){
    // TODO(Jinyun): move to confs
    std::vector<std::pair<double, double>> smoothed_point2d;
    std::vector<double> flexible_bounds = bounds;
    std::vector<size_t> colliding_point_index;
    const size_t max_iteration_num = 50;
    size_t counter = 0;
    bool is_collision_free = false;
    while (!is_collision_free) {
      if (counter > max_iteration_num) {
        cout << "Maximum iteration reached, path smoother early stops" << endl;
        return true;
      }
      // cout<<"flexible_bounds: " << flexible_bounds.size()<<endl;
      // cout<<"colliding_point_index: "<< colliding_point_index.size()<<endl;
      AdjustPathBounds(colliding_point_index, &flexible_bounds);

      std::vector<double> opt_x;
      std::vector<double> opt_y;
      if (!SqpWithOsqp(raw_point_set,flexible_bounds,&opt_x, &opt_y)) {
        cout << "Smoothing path fails"<<endl;
        return false;
      }

      if (opt_x.size() < 2 || opt_y.size() < 2) {
        cout << "Return by fem_pos_smoother is wrong. Size smaller than 2 "<<endl;
        return false;
      }

      CHECK_EQ(opt_x.size(), opt_y.size());

      size_t point_size = opt_x.size();
      smoothed_point2d.clear();
      smoothed_path_points.clear();
      for (size_t i = 0; i < point_size; ++i) {
        smoothed_point2d.emplace_back(opt_x[i], opt_y[i]);
      }

      if (!SetPathProfile(smoothed_point2d, &smoothed_path_points)) {
        AERROR << "Set path profile fails";
        return false;
      }
      input_colliding_point_index_.clear();
      is_collision_free =
          CheckCollisionAvoidance(smoothed_path_points, &colliding_point_index);
      // is_collision_free = true;
      // cout << "loop iteration number is " << counter;
      ++counter;
    }
    
    return true;
  }
  void OPTPlannerManager::AdjustPathBounds(const std::vector<size_t>& colliding_point_index,
                  std::vector<double>* bounds){
    CHECK_NOTNULL(bounds);
    const double collision_decrease_ratio = 0.9;//0.9

    
    for (const auto index : colliding_point_index) {
      bounds->at(index) *= collision_decrease_ratio;
      
      // cout<<"colliding:"<<endl;
      // cout<<index<<endl;
    }

    // Anchor the end points to enforce the initial end end heading continuity and
    // zero kappa
    bounds->at(0) = 0.0;
    bounds->at(1) = 0.0;
    bounds->at(bounds->size() - 1) = 0.0;
    bounds->at(bounds->size() - 2) = 0.0;
    if (enforce_initial_kappa_) {
      bounds->at(2) = 0.0;
    }

  }
  bool OPTPlannerManager::SetPathProfile(const std::vector<std::pair<double, double>>& point2d, std::vector<common::PathPoint>* raw_path_points){
    CHECK_NOTNULL(raw_path_points);
    raw_path_points->clear();
    // Compute path profile
    std::vector<double> headings;
    std::vector<double> kappas;
    std::vector<double> dkappas;
    std::vector<double> accumulated_s;
    if (!common::math::DiscretePointsMath::ComputePathProfile(
            point2d, &headings, &accumulated_s, &kappas, &dkappas)) {
      return false;
    }
    CHECK_EQ(point2d.size(), headings.size());
    CHECK_EQ(point2d.size(), kappas.size());
    CHECK_EQ(point2d.size(), dkappas.size());
    CHECK_EQ(point2d.size(), accumulated_s.size());

    // Load into path point
    size_t points_size = point2d.size();
    for (size_t i = 0; i < points_size; ++i) {
      common::PathPoint path_point;
      path_point.x = point2d[i].first;
      path_point.y = point2d[i].second;
      path_point.theta = headings[i];
      path_point.s = accumulated_s[i];
      path_point.kappa = (kappas[i]);
      path_point.dkappa = (dkappas[i]);
      path_point.lane_id = path_id;
      raw_path_points->push_back(std::move(path_point));
    }
    return true;
  }

  bool OPTPlannerManager::CheckCollisionAvoidance(const std::vector<common::PathPoint> & path_points, 
            std::vector<size_t>* colliding_point_index){
    CHECK_NOTNULL(colliding_point_index);
    colliding_point_index->clear();
    size_t path_points_size = path_points.size();
    for (size_t i = 0; i < path_points_size; ++i) {
      // Skip checking collision for thoese points colliding originally
      bool skip_checking = false;
      for (const auto index : input_colliding_point_index_) {
        if (i == index) {
          skip_checking = true;
          break;
        }
      }
      if (skip_checking) {
        continue;
      }

      const double heading = gear_.first
                                ? path_points[i].theta
                                : common::math::NormalizeAngle(path_points[i].theta + M_PI);
      common::math::Box2d ego_box(
          {path_points[i].x + center_shift_distance_ * std::cos(heading),
          path_points[i].y + center_shift_distance_ * std::sin(heading)},
          heading, ego_length_, ego_width_);

      bool is_colliding = false;
      for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
        for (const common::math::LineSegment2d& linesegment : obstacle_linesegments) {
          if (ego_box.HasOverlap(linesegment)) {
            colliding_point_index->push_back(i);
            ADEBUG << "point at " << i << "collied with LineSegment ";
                  // << linesegment.DebugString();
            // cout << "point at " << i << "collied with LineSegment "<<endl;  
            is_colliding = true;
            break;
          }
        }
        if (is_colliding) {
          break;
        }
      }
    }

    if (!colliding_point_index->empty()) {
      return false;
    }
    return true;
  }

  bool OPTPlannerManager::SqpWithOsqp(const std::vector<Eigen::Vector2d> &raw_point_set,const std::vector<double>& bounds,
    std::vector<double>* opt_x, std::vector<double>* opt_y){
    
    if (opt_x == nullptr || opt_y == nullptr) {
      AERROR << "opt_x or opt_y is nullptr";
      return false;
    }
    std::vector<std::pair<double, double>> ref_points;
    for(int i = 0; i < raw_point_set.size();i++){
      ref_points.push_back(make_pair(raw_point_set[i](0), raw_point_set[i](1)));
    }
    // sqp_optimizer->set_weight_fem_pos_deviation(1e8);
    // sqp_optimizer->set_weight_path_length(1.0);
    // sqp_optimizer->set_weight_ref_deviation(1e3);
    // sqp_optimizer->set_weight_curvature_constraint_slack_var(
    //     1e8);

    // sqp_optimizer->set_curvature_constraint(0.2);

    // solver.set_sqp_sub_max_iter(100);
    // solver.set_sqp_ftol(config_.sqp_ftol());
    // solver.set_sqp_pen_max_iter(config_.sqp_pen_max_iter());
    // solver.set_sqp_ctol(config_.sqp_ctol());

    // sqp_optimizer->set_max_iter(500);
    // sqp_optimizer->set_time_limit(0.0);
    // sqp_optimizer->set_verbose(false);
    // sqp_optimizer->set_scaled_termination(true);
    // sqp_optimizer->set_warm_start(true);

    // sqp_optimizer->set_ref_points(ref_points);
    // sqp_optimizer->set_bounds_around_refs(bounds);

    common::math::FemPosDeviationSqpOsqpInterface solver;

    solver.set_weight_fem_pos_deviation(1e8);
    solver.set_weight_path_length(1.0);
    solver.set_weight_ref_deviation(1e1);
    solver.set_weight_curvature_constraint_slack_var(
        1e8);

    solver.set_curvature_constraint(0.1);

    // solver.set_sqp_sub_max_iter(100);
    // solver.set_sqp_ftol(config_.sqp_ftol());
    // solver.set_sqp_pen_max_iter(config_.sqp_pen_max_iter());
    // solver.set_sqp_ctol(config_.sqp_ctol());

    solver.set_max_iter(500);
    solver.set_time_limit(0.0);
    solver.set_verbose(false);
    solver.set_scaled_termination(true);
    solver.set_warm_start(true);

    solver.set_ref_points(ref_points);
    solver.set_bounds_around_refs(bounds);

    if (!solver.Solve()) {
      return false;
    }

    std::vector<std::pair<double, double>> opt_xy = solver.opt_xy();

    // TODO(Jinyun): unify output data container
    opt_x->resize(opt_xy.size());
    opt_y->resize(opt_xy.size());
    for (size_t i = 0; i < opt_xy.size(); ++i) {
      (*opt_x)[i] = opt_xy[i].first;
      (*opt_y)[i] = opt_xy[i].second;
    }
    return true;

  }
  void OPTPlannerManager::publishPlan_bspline(const vector<Eigen::Vector3d>& path, ros::Publisher pub) {
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
    nav_path.header.stamp = ros::Time::now();

    pub.publish(nav_path);
}

bool OPTPlannerManager::PathSmoothAlgorithm(const HybridAStartResult &trajectory, const  std::vector<std::pair<VecCube, VecCube>>& corridors, int count, vector<Eigen::Vector3d>& path_smooth_set){
    std::vector<Eigen::Vector2d> point_set;
    std::vector<pair<double, double>> point_pair;
    // Set gear of the trajectory //向前为正
    gear_ = CheckGear(trajectory);
    // Set obstacle in form of linesegments
    std::vector<std::vector<common::math::LineSegment2d>> obstacles_linesegments_vec;
    for (const auto& corridor : corridors[count].first) {
      std::vector<common::math::LineSegment2d> obstacle_linesegments;
      common::math::Vec2d x1(corridor.box[0].second, corridor.box[1].first);
      common::math::Vec2d x2(corridor.box[0].second, corridor.box[1].second);
      common::math::Vec2d x3(corridor.box[0].first, corridor.box[1].second);
      common::math::Vec2d x4(corridor.box[0].first, corridor.box[1].first);
      
      obstacle_linesegments.emplace_back(common::math::LineSegment2d(x1,x2));
      obstacle_linesegments.emplace_back(common::math::LineSegment2d(x2,x3));
      obstacle_linesegments.emplace_back(common::math::LineSegment2d(x3,x4));
      obstacle_linesegments.emplace_back(common::math::LineSegment2d(x4,x1));
      
      obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
    }
    obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);
    for(int i = 0; i < trajectory.x.size(); i++){
      Eigen::Vector2d point2D(trajectory.x[i],trajectory.y[i]);
      point_set.emplace_back(move(point2D));
      point_pair.emplace_back(trajectory.x[i],trajectory.y[i]);
    }
    std::vector<common::PathPoint> rawHybridAS_path_points;
    SetPathProfile(point_pair, &rawHybridAS_path_points);
    HybridAS_path_points_plot.insert(HybridAS_path_points_plot.end(), rawHybridAS_path_points.begin(), rawHybridAS_path_points.end());
    AdjustStartEndHeading(trajectory, point_set);
    // if (!SetPathProfile(interpolated_warm_start_point2ds,
    //               &interpolated_warm_start_path)) {
    //   AERROR << "Set path profile fails";
    //   return ;
    // }
    std::vector<double> bounds;
    if (!GenerateInitialBounds(point_set, &bounds)) {
      AERROR << "Generate initial bounds failed, path point to close to obstacle";
      return false;
    }
    // input_colliding_point_index_.clear();
    // if (!CheckCollisionAvoidance(interpolated_warm_start_path,
    //                             &input_colliding_point_index_)) {
    //   AERROR << "Interpolated input path points colliding with obstacle";
    //   // if (!ReAnchoring(colliding_point_index, &interpolated_warm_start_path)) {
    //   //   AERROR << "Fail to reanchor colliding input path points";
    //   //   return false;
    //   // }
    // }
    // std::vector<Eigen::Vector2d> smooth_point_set;
    std::vector<common::PathPoint> smooth_path_point;
    // CHECK_NOTNULL(sqp_optimizer);
    // sqp_optimizer.reset(new common::math::FemPosDeviationSqpOsqpInterface);
    if (!SmoothPath(point_set, bounds,
                    smooth_path_point)) {
      AERROR<<"SmoothPath failed!";
      return false;
    }
    smooth_path_point_plot.insert(smooth_path_point_plot.end(),smooth_path_point.begin(),smooth_path_point.end());
    for (size_t i = 0; i < smooth_path_point.size(); i++) {
      // smooth_point_set.emplace_back(smooth_path_point[i].x, smooth_path_point[i].y);
      Eigen::Vector3d point3D(smooth_path_point[i].x, smooth_path_point[i].y, smooth_path_point[i].theta);
      path_smooth_set.emplace_back(move(point3D));
    }
    PathSmooth_set.insert(PathSmooth_set.end(), path_smooth_set.begin(), path_smooth_set.end());     
    return true;                                   
  }
  void OPTPlannerManager::PlotCurvature(){
    if(HybridAS_path_points_plot.size() != smooth_path_point_plot.size()) {
      AERROR<<"HybridAS_path_points_plot != smooth_path_point_plot!";
      return;
    }
    int n = HybridAS_path_points_plot.size();
    std::vector<double> t(n);
    std::vector<double> x1(n);
    std::vector<double> x2(n);
    std::vector<double> x3;
    
    for(int i = 5; i < n-5; i++){
      // t[i] = smooth_path_point_plot[i].s;
      x1[i] = HybridAS_path_points_plot[i].kappa;
      x2[i] = smooth_path_point_plot[i].kappa;
    }
    int j = (int)(Bspline_path_points_plot.size() / HybridAS_path_points_plot.size());
    for(int i = 5; i < Bspline_path_points_plot.size()-5; i += j){
      x3.push_back(Bspline_path_points_plot[i].kappa);
    }
    cout<<(HybridAS_path_points_plot.size())<<endl;
    cout<<(x3.size())<<endl;
    // plt::subplot(2,1,1);
    // plt::title("Curvature Smooth");
		// plt::xlabel("s");
		plt::ylabel("Curvature");
		plt::named_plot("HybridAStar",x1);
		plt::named_plot("DL-IAPS",x2);
    // plt::legend();
    // plt::subplot(2,1,2);
    // plt::xlabel("t");
		// plt::ylabel("curvature");
    plt::named_plot("our",x3);
		plt::legend();
		// plt::save("1PathSmooth.png");
		plt::show();
  }
  void OPTPlannerManager::getSample(const HybridAStartResult& trajectory, vector<Eigen::Vector3d>& point_set, 
                int sample_count, double& start_angle, double& end_angle, double& total_distance){
    if(point_set.size() == 4 ) sample_count = 1;

    point_set.emplace_back(trajectory.x.front(),trajectory.y.front(), trajectory.phi.front());
    for(int j = 1; j < trajectory.x.size()-1; j+=sample_count){
          Eigen::Vector3d point3D(trajectory.x[j],trajectory.y[j], trajectory.phi[j]);
          point_set.emplace_back(move(point3D));
        }
    point_set.emplace_back(trajectory.x.back(),trajectory.y.back(), trajectory.phi.back());

    double sum = 0;
    for(long j = 1; j < point_set.size(); j++){
          sum += (point_set[j].head(2) - point_set[j-1].head(2)).norm();
    }
    // cout<<"The average distance between 2 points: "<<sum/(point_set.size()-1)<<endl;
    //一段路径出入射角度保持一致
    //CheckGear()函数在PathSmoothAlgorithm()中调用，如果没有调用PathSmoothAlgorithm()则gear为初始值！！！！
    start_angle = gear_.first ? point_set[0](2) : common::math::NormalizeAngle(point_set[0](2) + M_PI);
    end_angle = gear_.second ? point_set[point_set.size()-1](2) : common::math::NormalizeAngle(point_set[point_set.size()-1](2) + M_PI);
    //一段路径起点到终点的位移
    total_distance = (point_set.back().head(2) - point_set.front().head(2)).norm();
    cout<<"s: "<< total_distance <<endl;
}


 

} // namespace opt_planner
