#include <Eigen/Core>
#include <random>
#include <iostream>
#include <stdint.h>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "lidar_feature_plane.hpp"


using namespace Eigen;
using namespace std;

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

void generate_point_cloud(const Isometry3d& pose, 
        const Isometry3d& trans_src, 
        const Isometry3d& trans_tgt,
        pcl::PointCloud<PCLVelodyne>& src_points,
        pcl::PointCloud<PCLVelodyne>& tar_points,
        size_t N)
{
  // simulate point cloud 
  for (size_t i=0;i<N; ++i)
  {
	Vector3d pos_g ;	
	double x, y, z  ;
	x = (Sample::uniform()-0.5)*3 ;
	y = Sample::uniform()-0.5 ;
	z = Sample::uniform()-0.5 ;
	if(i < N/2)
		pos_g = Eigen::Vector3d(x, y, 0) ;
	else
		pos_g = Eigen::Vector3d(0, y, z) ;

	Vector3d pos_tgt_b = trans_tgt.inverse()*pos_g ;
	Vector3d pos_tgt_l = pose.inverse()*pos_tgt_b ;
	
	Vector3d pos_src_b = trans_src.inverse()*pos_g ;
	Vector3d pos_src_l = pose.inverse()*pos_src_b ;
	
	PCLVelodyne p0;
	p0.x = pos_tgt_l[0];
	p0.y = pos_tgt_l[1];
	p0.z = pos_tgt_l[2];
	PCLVelodyne p1;
	p1.x = pos_src_l[0];
	p1.y = pos_src_l[1]; 
	p1.z = pos_src_l[2];
	tar_points.push_back(p0) ;
	src_points.push_back(p1) ;
  }

   if(true)
   {
		Vector3d pos_g = Vector3d(0, 1, 0) ;
		Vector3d pos_tgt_b = trans_tgt.inverse()*pos_g ;
		Vector3d pos_tgt_l = pose.inverse()*pos_tgt_b ;
		
		Vector3d pos_src_b = trans_src.inverse()*pos_g ;
		Vector3d pos_src_l = pose.inverse()*pos_src_b ;
		
		//std::cout << "pos in target lidar frame " << std::endl ;
		//std::cout << pos_tgt_l << std::endl ;
		//std::cout << "pose in source lidar frame " << std::endl ;
		//std::cout << pos_src_l << std::endl ;
   }


}



//
// set up simulated system with noise, optimize it
//

int main(int argc, char* argv[])
{
    size_t N = 1000 ;
    if(argc > 1)
        N = atoi(argv[1]) ;
    printf("%zu points per cloud\n", N) ;

    // add noise
    double euc_noise = 0.01;       // noise in position, m
    //  double outlier_ratio = 0.1;
    
    // simulate calibration
    Isometry3d pose ;
    Quaterniond qx(Eigen::AngleAxisd(0.1*M_PI, Eigen::Vector3d::UnitX())) ;
    Quaterniond qy(Eigen::AngleAxisd(0.2*M_PI, Eigen::Vector3d::UnitY())) ;
    Quaterniond qz(Eigen::AngleAxisd(0.3*M_PI, Eigen::Vector3d::UnitZ())) ;
    Quaterniond q = qz * qy * qx   ;
    pose = q ;
    Vector3d t(0, 0, 0) ;
    pose.translation() = t ;
    
    // simulate global poses 
    Isometry3d trans_0, trans_1, trans_2 ;
    trans_0.setIdentity() ;
    trans_1 = qz ;
    trans_1.translation() = Vector3d(1, 0, 0) ;
    trans_2 = qy ;
    trans_2.translation() = Vector3d(2, 1, 0) ;
      
    /// simulate points
    pcl::PointCloud<PCLVelodyne> points_1, points_2, points_3, points_4 ;
    generate_point_cloud(pose, trans_1, trans_0, points_2, points_1, N) ;
    generate_point_cloud(pose, trans_2, trans_1, points_4, points_3, N) ;
   
    /// create plane feature class
    lcm::LCM lcm ;
    param_t param(&lcm) ;
    trans_t trans(lcm.getUnderlyingLCM(), &param) ;
    LidarFeatureBase<PCLVelodyne> * feat = new LidarUnorgPlane<PCLVelodyne>(&lcm, &trans, "features.plane") ;
    
    feat->extract(points_1) ;

    return 0 ;

}
