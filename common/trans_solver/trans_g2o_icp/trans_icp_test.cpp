#include <Eigen/Core>
#include <random>
#include <iostream>
#include <stdint.h>

#include "trans_solver_6d_icp.hpp"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

using namespace Eigen;
using namespace std;
using namespace g2o;

// sampling distributions
  class Sample
  {

    static default_random_engine gen_real;
    static default_random_engine gen_int;
  public:
    static int uniform(int from, int to);

    static double uniform();

    static double gaussian(double sigma);
  };


  default_random_engine Sample::gen_real;
  default_random_engine Sample::gen_int;

  int Sample::uniform(int from, int to)
  {
    uniform_int_distribution<int> unif(from, to);
    int sam = unif(gen_int);
    return  sam;
  }

  double Sample::uniform()
  {
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    double sam = unif(gen_real);
    return  sam;
  }

  double Sample::gaussian(double sigma)
  {
    std::normal_distribution<double> gauss(0.0, sigma);
    double sam = gauss(gen_real);
    return  sam;
  }


//
// set up simulated system with noise, optimize it
//

int main()
{
  // create data
  double euc_noise = 0.01;       // noise in position, m
  //  double outlier_ratio = 0.1;
  Isometry3d pose ;
  Quaterniond q(Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX())) ;
  //q.setIdentity() ;
  pose = q ;
  Vector3d t(1, 0, 0) ;
  pose.translation() = t ;
  int N = 1000 ;

  pcl::PointCloud<pcl::PointXYZ> src_points, tar_points ;
  for (size_t i=0;i<N; ++i)
  {
	Vector3d pos1 ;	
	double x, y, z  ;
	x = (Sample::uniform()-0.5)*3 ;
	y = Sample::uniform()-0.5 ;
	z = Sample::uniform()-0.5 ;
	if(i < N/3)
		pos1 = Eigen::Vector3d(x, y, 10) ;
	else if (i < 2*N/3)
		pos1 = Eigen::Vector3d(x, -x, z) ;
	else
		pos1 = Eigen::Vector3d(x, x, z) ;
	
    Vector3d pos0 = pose * pos1 ;	
	pcl::PointXYZ p1(pos1[0], pos1[1], pos1[2]) ;
	pcl::PointXYZ p0(pos0[0], pos0[1], pos0[2]) ;
    tar_points.push_back(p0) ;
	src_points.push_back(p1) ;
  }

   // icp solver
   Isometry3d pose0 ;
   pose0.setIdentity() ;

   auto icp = dynamic_cast<TransSolverBase<pcl::PointXYZ, Eigen::Isometry3d>*>(new TransSolver6DICP<pcl::PointXYZ>()) ;
	
   icp->set_source_pc(src_points.makeShared()) ;
   icp->set_target_pc(tar_points.makeShared()) ;
   icp->set_initial_trans(pose0) ;

   icp->run() ;

   std::cout << "final result " << std::endl ;
   std::cout << g2o::internal::toVectorMQT(icp->get_trans()) << std::endl ;

  return 0 ;

}
