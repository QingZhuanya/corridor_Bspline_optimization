
#ifndef HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H
#define HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H
// #include <bspline_opt/bspline_optimizer.h>
// #include <bspline_opt/uniform_bspline.h>
#include <hybrid_a_star/hybrid_a_star.h>
#include <hybrid_a_star/costmap_subscriber.h>
#include <hybrid_a_star/init_pose_subscriber.h>
#include <hybrid_a_star/goal_pose_subscriber.h>

#include <ros/ros.h>
#include <math/vec2d.h>
#include <math/math_utils.h>
class HybridAStarFlow {
public:
    typedef std::shared_ptr<HybridAStarFlow> Ptr;

    ~HybridAStarFlow();
    // HybridAStarFlow();
    HybridAStarFlow() = default;

    explicit HybridAStarFlow(ros::NodeHandle &nh); //it is valuable onle if explicit ues
    bool Run();

    Eigen::Vector3d getStartEndDistance() ;
    std::vector<Eigen::Vector2d> getPointSet() ;
    std::vector<HybridAStartResult> getPatitionTrajectory() ;

    double getMapResolution(){
        return map_resolution;
    }
    bool hasStartEndPt(){
        return (has_startPt_ && has_endPt_);
    }
    void resetPt(){
        has_startPt_ = false;
        has_endPt_ = false;
    }
    bool get2DOccupancy(const Eigen::Vector2d &pos){
        return kinodynamic_astar_searcher_ptr_->getOccupancy(pos);
    }
    bool get3DOccupancy(const Vec3d &pos){
        return kinodynamic_astar_searcher_ptr_->getOccupancy(pos);
    }

    std::vector<std::pair<VecCube,VecCube>> GetCorridors() {
    return corridors;
    }
    VecCube GetCorridor()  {
    return all_corridor;
    }
    std::pair<Eigen::Vector3d, Eigen::Vector3d> GetStartEnd(){return { start_pt_,  end_pt_};}
private:
    void InitPoseData();

    void ReadData();

    bool HasStartPose();

    bool HasGoalPose();

    void PublishPath(const VectorVec3d &path);

    void PublishSearchedTree(const VectorVec4d &searched_tree);

    void PublishVehiclePath(const VectorVec3d &path, double width,
                            double length, unsigned int vehicle_interval);

    void PublishWayPoint(const VectorVec3d &path);

    void PublishCorridor(const std::vector<Cube> &corridor);

    void DataTransform(const VectorVec3d& raw_path, HybridAStartResult* result);

    void LoadHybridAstarResultInEigen(HybridAStartResult* result,
                                    Eigen::MatrixXd* xWS, Eigen::MatrixXd* uWS);
    
    // bool GenerateDecoupledTraj(
    //   const Eigen::MatrixXd& xWS, const double init_a, const double init_v,
    //   const std::vector<std::vector<common::math::Vec2d>>&
    //       obstacles_vertices_vec,
    //   Eigen::MatrixXd* state_result_dc, Eigen::MatrixXd* control_result_dc,
    //   Eigen::MatrixXd* time_result_dc);
    bool GenerateDecoupledTraj(
      const Eigen::MatrixXd& xWS, const double init_a, const double init_v,
    //   const std::vector<std::vector<common::math::Vec2d>>&
    //       obstacles_vertices_vec,
      Eigen::MatrixXd* state_result_dc, Eigen::MatrixXd* control_result_dc,
      Eigen::MatrixXd* time_result_dc);

    
private:
    std::shared_ptr<HybridAStar> kinodynamic_astar_searcher_ptr_;
    std::shared_ptr<CostMapSubscriber> costmap_sub_ptr_;
    std::shared_ptr<InitPoseSubscriber2D> init_pose_sub_ptr_;
    std::shared_ptr<GoalPoseSubscriber2D> goal_pose_sub_ptr_;

    ros::Publisher path_pub_;
    ros::Publisher searched_tree_pub_;
    ros::Publisher vehicle_path_pub_;
    ros::Publisher way_point_pub_;
    ros::Publisher corridor_pub_;

    std::deque<geometry_msgs::PoseWithCovarianceStampedPtr> init_pose_deque_;
    std::deque<geometry_msgs::PoseStampedPtr> goal_pose_deque_;
    std::deque<nav_msgs::OccupancyGridPtr> costmap_deque_;

    geometry_msgs::PoseWithCovarianceStampedPtr current_init_pose_ptr_;
    geometry_msgs::PoseStampedPtr current_goal_pose_ptr_;
    nav_msgs::OccupancyGridPtr current_costmap_ptr_;

    ros::Time timestamp_;

    bool has_map_{};
    bool has_startPt_{}, has_endPt_{};
    bool hybrid_AStar_plan_{};

    double map_resolution;
    std::vector<Eigen::Vector2d> point_set;
    VectorVec3d path;

    std::vector<std::pair<VecCube, VecCube>> corridors;
    VecCube all_corridor;
    std::vector<HybridAStartResult> partition_trajectories;

    Vec3d start_pt_;
    Vec3d end_pt_;
};

#endif //HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H
