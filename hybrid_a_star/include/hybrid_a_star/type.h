

#ifndef HYBRID_A_STAR_TYPE_H
#define HYBRID_A_STAR_TYPE_H
#include <iostream>
#include <vector>
#include <Eigen/Core>

template<int dim>
using TypeVectorVecd = typename std::vector<Eigen::Matrix<double, dim, 1>,
        Eigen::aligned_allocator<Eigen::Matrix<double, dim, 1>>>;

typedef TypeVectorVecd<4> VectorVec4d;
typedef TypeVectorVecd<3> VectorVec3d;
typedef TypeVectorVecd<2> VectorVec2d;

// typedef typename Eigen::Vector2d Vec2d;
typedef typename Eigen::Vector3d Vec3d;
typedef typename Eigen::Vector4d Vec4d;

typedef typename Eigen::Vector2i Vec2i;
typedef typename Eigen::Vector3i Vec3i;

typedef typename Eigen::Matrix2d Mat2d;
typedef typename Eigen::Matrix3d Mat3d;

typedef typename Eigen::MatrixXd MatXd;
typedef typename Eigen::VectorXd VecXd;

struct HybridAStartResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> steer;
  std::vector<double> accumulated_s;
};

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

struct Cube;

struct Cube
{     
      //Eigen::Vector3d p1(xl,yl), p2(xu,yl), p3(xu,yu), p4(xl,yu)   // the 4 vertex of a cube 
      Eigen::MatrixXd vertex;
      Eigen::Vector3d center; // the center of the cube
      bool valid;    // indicates whether this cube should be deleted
      int index;
      double t; // time allocated to this cube
      std::vector< std::pair<double, double> > box;
/*
           P4------------P3 
           /            /               ^
          /            /                | y
        P1------------P2                |
                                        |
                                        /--------> x


*/                                                                                 

      // create a cube using 8 vertex and the center point
      Cube( Eigen::MatrixXd vertex_, Eigen::Vector3d center_)
      {
            vertex = vertex_;
            center = center_;
            valid = true;
            t = 0.0;
            box.resize(2);
      }

      // create a inscribe cube of a ball using the center point and the radius of the ball
      void setVertex( Eigen::MatrixXd vertex_, double resolution_)
      {     
            vertex = vertex_;
            vertex(0,1) -= resolution_ / 2.0;
            vertex(1,1) -= resolution_ / 2.0;


            vertex(2,1) += resolution_ / 2.0;
            vertex(3,1) += resolution_ / 2.0;


            vertex(1,0) += resolution_ / 2.0;
            vertex(2,0) += resolution_ / 2.0;


            vertex(0,0) -= resolution_ / 2.0;
            vertex(3,0) -= resolution_ / 2.0;
            
            // vertex = vertex_;
            // vertex(0,1) -= resolution_ / 2.0;
            // vertex(3,1) -= resolution_ / 2.0;


            // vertex(1,1) += resolution_ / 2.0;
            // vertex(2,1) += resolution_ / 2.0;


            // vertex(0,0) += resolution_ / 2.0;
            // vertex(1,0) += resolution_ / 2.0;


            // vertex(2,0) -= resolution_ / 2.0;
            // vertex(3,0) -= resolution_ / 2.0;
            
            setBox();
      }
      
      void setBox()
      {
            box.clear();
            box.resize(2);
            box[0] = std::make_pair( vertex(0, 0), vertex(1, 0) );//xlow, xhigh
            box[1] = std::make_pair( vertex(1, 1), vertex(2, 1) );//ylow, yhigh
            // box[0] = std::make_pair( vertex(3, 0), vertex(0, 0) );//xlow, xhigh
            // box[1] = std::make_pair( vertex(0, 1), vertex(1, 1) );//ylow, yhigh

      }

      void printBox()
      {
            std::cout<<"center of the cube: \n"<<center<<std::endl;
            std::cout<<"vertex of the cube: \n"<<vertex<<std::endl;
      }

      Cube()
      {  
         center = Eigen::VectorXd::Zero(3);
         vertex = Eigen::MatrixXd::Zero(8, 3);

         valid = true;
         t = 0.0;
         box.resize(2);
      }
      void clear(){
            center = Eigen::VectorXd::Zero(3);
            vertex = Eigen::MatrixXd::Zero(8, 3);

            valid = true;
            t = 0.0;
            box.resize(2);
      }
      ~Cube(){}
};

using VecCube = typename std::vector<Cube>;
#endif //HYBRID_A_STAR_TYPE_H