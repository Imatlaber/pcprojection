#include "pcl_viewer.hpp"
#include <Eigen/Core>
#include <random>
#include <iostream>
#include <stdint.h>
#include <array>

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

void generate_point_cloud(pcl::PointCloud<pcl::PointXYZ>& src_points, size_t N)
{
  // simulate point cloud 
  for (size_t i=0;i<N; ++i)
  {
	Vector3d pos_g;	
	double x, y, z  ;
	x = (Sample::uniform()-0.5)*3 ;
	y = Sample::uniform()-0.5 ;
	z = Sample::uniform()-0.5 ;
	if(i < N/2)
    {
		pos_g = Eigen::Vector3d(x, y, 0) ;

    }
	else
    {
		pos_g = Eigen::Vector3d(0, y, z) ;
    }

    pcl::PointXYZ p(pos_g[0], pos_g[1], pos_g[2]) ;
	src_points.push_back(p) ;
  }
}

int main(int argc, char * argv[])
{
    size_t N = 1000 ;
    if(argc > 1)
        N = atoi(argv[1]) ;
    printf("%zu points per cloud\n", N) ;

    // add noise
    /// simulate points
    pcl::PointCloud<pcl::PointXYZ> points ;
    generate_point_cloud(points, N) ;
    
    had::PCLViewer viewer ;
    viewer.initialize() ;
    
    std::array<int,3> color{{0, 255, 0}} ;
    viewer.add_cloud_to_viewer<pcl::PointXYZ>(points.makeShared(), color, "dummy") ;

    viewer.show() ;
    



    return 0 ;
}
