/* Copyright (c) AutoNavi - All Rights Reserved 
 * Author: shuo <shuo@alibaba-inc.com>
 * Description: 6DOD calibration solver
 */

#ifndef _TRANS_6D_CALIB_HPP_
#define _TRANS_6D_CALIB_HPP_

#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "../../common/trans_solver/trans_solver_base.hpp"
#include "type_calib.h"

template<class T>
class TransSolver6DCALIB : public TransSolverBase<T, Eigen::Isometry3d>
{
	public:
	TransSolver6DCALIB() ;

	bool solve(const pcl::PointCloud<T>& source_pc,
					 const std::vector<int>& source_indices,
					 const std::vector<Eigen::Matrix3d>& source_cov,
					 const pcl::PointCloud<T>& target_pc,
					 const std::vector<int> & target_indices,
					 const std::vector<Eigen::Matrix3d> & target_cov,
					 Eigen::Isometry3d& trans
					 ) ;


	bool run() override ; 
	void compute_correspondence(const pcl::PointCloud<T>& source_pc,
								const pcl::PointCloud<T>& target_pc,
								const Eigen::Isometry3d& trans,
								std::vector<int>& source_indices,
								std::vector<int>& target_indices) ;
	bool compute_covariances(const pcl::PointCloud<T>& pc, 
							 const pcl::KdTreeFLANN<T>& tree,
							 std::vector<Eigen::Matrix3d>& covs) ;
	void build_kdtree(const pcl::PointCloud<T>& pc, pcl::KdTreeFLANN<T>& tree) ;

	const Eigen::Isometry3d& get_trans() const override ;
	
	public:
	
	pcl::KdTreeFLANN<T> source_tree_ ;
	pcl::KdTreeFLANN<T> target_tree_ ;
	Eigen::Isometry3d trans_ ;
	int k_correspondences_ ;
	double calib_epsilon_ ;
	double dist_threshold_ ;
	double rotation_epsilon_ ;
	double transformation_epsilon_ ;
	int max_iterations_ ;
	
} ;



#endif // _TRANS_SOLVER_6D_CALIB_HPP_
