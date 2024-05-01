#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
// #include <ego_planner/DataDisp.h>
// #include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
#include <ros/ros.h>
#include <plan_manage/planning_visualization.h>
#include <hybrid_a_star/hybrid_a_star_flow.h>
#include <hybrid_a_star/type.h>
#include <hybrid_a_star/timer.h>
#include <math/fem_pos_deviation_sqp_osqp_interface.h>
#include <log.h>
#include <math/line_segment2d.h>
#include <point.h>
#include <math/discrete_points_math.h>
#include <math/box2d.h>
#include <plan_manage/CubicSpline.h>

#include <matplotlibcpp.h>
namespace plt = matplotlibcpp;
namespace opt_planner
{
  
  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class OPTPlannerManager
  {
    // SECTION stable
  public:
    OPTPlannerManager();
    ~OPTPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void run();
    /* main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    // bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    // void initPlanModules(ros::NodeHandle &nh);
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);

    PlanParameters pp_;
    LocalTrajData local_data_;
    // GlobalTrajData global_data_;
    // GridMap::Ptr grid_map_;


  private:
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    HybridAStarFlow::Ptr hybridAStar_flow_;
    BsplineOptimizer::Ptr bspline_optimizer_rebound_;
    
    std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec_;

    bool plan_success_;
    int continous_failures_count_{0};


    bool PathSmoothAlgorithm(const HybridAStartResult &trajectory, const  std::vector<std::pair<VecCube, VecCube>>& corridors, int count, vector<Eigen::Vector3d>& path_smooth_set);

    pair<bool,bool> CheckGear(const struct HybridAStartResult &trajectory);

    void AdjustStartEndHeading(
    const struct HybridAStartResult &trajectory, vector<Eigen::Vector2d>& point_set);
    
    bool SmoothPath(const std::vector<Eigen::Vector2d> &raw_point_set,const std::vector<double>& bounds,
                    std::vector<common::PathPoint>& smoothed_path_points);

    bool SqpWithOsqp(const std::vector<Eigen::Vector2d> &raw_point_set,const std::vector<double>& bounds,
                   std::vector<double>* opt_x, std::vector<double>* opt_y);
    
    void AdjustPathBounds(const std::vector<size_t>& colliding_point_index,
                  std::vector<double>* bounds);
    bool GenerateInitialBounds(const std::vector<Eigen::Vector2d> &raw_point_set,
                             std::vector<double>* initial_bounds);
    bool SetPathProfile(const std::vector<std::pair<double, double>>& point2d, std::vector<common::PathPoint>* raw_path_points);
    bool CheckCollisionAvoidance(const std::vector<common::PathPoint> & path_points, std::vector<size_t>* colliding_point_index);

    void getSample(const HybridAStartResult& trajectory, vector<Eigen::Vector3d>& point_set, 
                int sample_count, double& start_angle, double& end_angle, double& total_distance);
    void publishPlan_bspline(const vector<Eigen::Vector3d>& path, ros::Publisher pub);

    void PlotCurvature();


    // gear DRIVE as true and gear REVERSE as false
    pair<bool, bool> gear_ = {false, false};
    bool enforce_initial_kappa_ = true;
    std::vector<size_t> input_colliding_point_index_;
    vector<Eigen::Vector3d> PathSmooth_set, BSplineSmooth_set;
    vector<Eigen::Vector3d> Dl_pathSmooth_set;
    // vehicle_param
    double ego_length_ = 2.44;
    double ego_width_ = 0.82;
    double back_edge_to_center = ego_length_/2.0;
    double center_shift_distance_ = ego_length_ / 2.0 - back_edge_to_center;
    ros::Publisher plan_pub_bspline;
    ros::Publisher plan_pub_dl;
    std::vector<common::PathPoint> smooth_path_point_plot;
    std::vector<common::PathPoint> HybridAS_path_points_plot;
    std::vector<common::PathPoint> Bspline_path_points_plot;
    std::vector<common::PathPoint> dl_path_points_plot;
    int path_id;
    double ts_;
    
    // void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);

    // void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
    //                     double &time_inc);

    // bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    typedef std::shared_ptr<OPTPlannerManager> Ptr;

    // !SECTION
  };
} // namespace opt_planner

#endif