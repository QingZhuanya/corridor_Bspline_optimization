

#ifndef COMMON_POINY_H
#define COMMON_POINY_H
#include <iostream>
#include <vector>
#include <Eigen/Core>

namespace common
{
    struct PathPoint {
	// coordinates
	double x;
	double y;
	double z;
	
	// direction on the x-y plane
	double theta;
	// curvature on the x-y planning
	double kappa;
	// accumulated distance from beginning of the path
	double s;
	
	// derivative of kappa w.r.t s.
	double dkappa;
	// derivative of derivative of kappa w.r.t s.
	double ddkappa;
	// The lane ID where the path point is on
	std::string lane_id;
	
	// derivative of x and y w.r.t parametric parameter t in CosThetareferenceline
	double x_derivative;
	double y_derivative;
};

struct TrajectoryPoint {
    PathPoint path_point ;
    double v ;  // in [m/s]
    double a ;
    double relative_time ;
    double da;
    double steer ;
};
} // namespace common



#endif //COMMON_POINY_H