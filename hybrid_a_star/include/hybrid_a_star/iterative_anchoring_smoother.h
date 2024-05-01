// #pragma once
// #include <utility>
// #include <vector>
// // #include <point.h>
// class IterativeAnchoringSmoother {
//  public:
//   IterativeAnchoringSmoother(
//       const PlannerOpenSpaceConfig& planner_open_space_config);

//   ~IterativeAnchoringSmoother() = default;

//   bool Smooth(const Eigen::MatrixXd& xWS, const double init_a,
//               const double init_v,
//               const std::vector<std::vector<common::math::Vec2d>>&
//                   obstacles_vertices_vec,
//               DiscretizedTrajectory* discretized_trajectory);

//  private:
//   void AdjustStartEndHeading(
//       const Eigen::MatrixXd& xWS,
//       std::vector<std::pair<double, double>>* const point2d);

//   bool ReAnchoring(const std::vector<size_t>& colliding_point_index,
//                    DiscretizedPath* path_points);

//   bool GenerateInitialBounds(const DiscretizedPath& path_points,
//                              std::vector<double>* initial_bounds);

//   bool SmoothPath(const DiscretizedPath& raw_path_points,
//                   const std::vector<double>& bounds,
//                   DiscretizedPath* smoothed_path_points);

//   bool CheckCollisionAvoidance(const DiscretizedPath& path_points,
//                                std::vector<size_t>* colliding_point_index);

//   void AdjustPathBounds(const std::vector<size_t>& colliding_point_index,
//                         std::vector<double>* bounds);

//   bool SetPathProfile(const std::vector<std::pair<double, double>>& point2d,
//                       DiscretizedPath* raw_path_points);

//   bool CheckGear(const Eigen::MatrixXd& xWS);

//   bool SmoothSpeed(const double init_a, const double init_v,
//                    const double path_length, SpeedData* smoothed_speeds);

//   bool CombinePathAndSpeed(const DiscretizedPath& path_points,
//                            const SpeedData& speed_points,
//                            DiscretizedTrajectory* discretized_trajectory);

//   void AdjustPathAndSpeedByGear(DiscretizedTrajectory* discretized_trajectory);

//   bool GenerateStopProfileFromPolynomial(const double init_acc,
//                                          const double init_speed,
//                                          const double stop_distance,
//                                          SpeedData* smoothed_speeds);

//   bool IsValidPolynomialProfile(const QuinticPolynomialCurve1d& curve);

//   // @brief: a helper function on discrete point heading adjustment
//   double CalcHeadings(const DiscretizedPath& path_points, const size_t index);

//  private:
//   // vehicle_param
//   double ego_length_ = 0.0;
//   double ego_width_ = 0.0;
//   double center_shift_distance_ = 0.0;

//   std::vector<std::vector<common::math::LineSegment2d>>
//       obstacles_linesegments_vec_;

//   std::vector<size_t> input_colliding_point_index_;

//   bool enforce_initial_kappa_ = true;

//   // gear DRIVE as true and gear REVERSE as false
//   bool gear_ = false;

// //   PlannerOpenSpaceConfig planner_open_space_config_;
// };

