#include <algorithm>
#include <pcl/common/transforms.h>
#include "trans_solver_6d_pcl.hpp"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

template<class T>
TransSolver6DPCL<T>::TransSolver6DPCL():
k_correspondences_(20),
gpcl_epsilon_(1e-3),
dist_threshold_(25),
rotation_epsilon_(5e-3),
transformation_epsilon_(1e-3),
max_iterations_(200),
max_optimizer_iterations_(3)
{

}

template<class T>
bool TransSolver6DPCL<T>::run()
{
	//std::vector<int> source_indices ;
	//std::vector<int> target_indices ;
	//std::vector<Eigen::Matrix3d> source_covs ;
	//std::vector<Eigen::Matrix3d> target_covs ;

	//build_kdtree(*this->source_pc_, source_tree_) ;
	//build_kdtree(*this->target_pc_, target_tree_) ;
	//
	//if(!compute_covariances(*this->source_pc_, source_tree_, source_covs))
	//	return false ;
	//if(!compute_covariances(*this->target_pc_, target_tree_, target_covs)) 
	//	return false ;

	//bool converged = false ;

	const Eigen::Matrix4d& trans_d = this->trans0_.matrix() ;
	Eigen::Matrix4f trans_f = trans_d.cast<float>() ;
	
	typename pcl::PointCloud<T>::Ptr transformed_pc(new pcl::PointCloud<T>) ;
	gicp_.setInputSource(this->source_pc_) ;
	gicp_.setInputTarget(this->target_pc_) ;
	gicp_.setCorrespondenceRandomness(k_correspondences_) ;
	gicp_.setRotationEpsilon(rotation_epsilon_) ;
	gicp_.setTransformationEpsilon(transformation_epsilon_) ;
	gicp_.setMaximumIterations(max_iterations_) ;
	gicp_.align(*transformed_pc, trans_f) ;

		//gicp.setCorrespondenceRandomness(300);  
		//gicp.setRotationEpsilon(2e-6);
		//gicp.setTransformationEpsilon(1e-8);
		//gicp.setMaximumIterations(300);
	
		//cout << "EuclideanFitnessEpsilon: " << gicp.getEuclideanFitnessEpsilon()<< endl;
		//cout << "FitnessScore: " << gicp.getFitnessScore()<< endl;
		//cout << "RotationEpsilon: " << gicp.getRotationEpsilon() << endl;
		//cout << "TransformationEpsilon: " << gicp.getTransformationEpsilon() << endl;
		//transformation = gicp.getFinalTransformation();
		//transformation = transformation * init_transEIGtransport;   
		//cout << "transformation: \n" << transformation << endl;
		//Eigen::Matrix4f::Map(est_trans) = transformation.transpose(); 

	
	Eigen::Matrix4f trans = gicp_.getFinalTransformation() ;
	Eigen::Vector3f angles = trans.topLeftCorner<3,3>().eulerAngles(2, 1, 0) ;
	std::cout << "Final transformation " << std::endl ;
	std::cout << trans << std::endl ;
	std::cout << "rotation angles " << std::endl ;
	std::cout << angles << std::endl ;

	return true ;
}

template<class T>
const Eigen::Isometry3d& TransSolver6DPCL<T>::get_trans() const
{
	return trans_ ;
}

template<class T>
bool TransSolver6DPCL<T>::solve(const pcl::PointCloud<T>& source_pc,
					 const std::vector<int>& source_indices,
					 const std::vector<Eigen::Matrix3d>& source_cov,
					 const pcl::PointCloud<T>& target_pc,
					 const std::vector<int> & target_indices,
					 const std::vector<Eigen::Matrix3d> & target_cov,
					 Eigen::Isometry3d& trans
					 ) 
{

	return true ;
}


template<class T>
void TransSolver6DPCL<T>::compute_correspondence(const pcl::PointCloud<T>& source_pc,
												 const pcl::PointCloud<T>& target_pc,   
												 const Eigen::Isometry3d& trans,
													std::vector<int>& source_indices,
													std::vector<int>& target_indices)
{
}

template<class T>
bool TransSolver6DPCL<T>::compute_covariances(const pcl::PointCloud<T>& pc,
												const pcl::KdTreeFLANN<T>& tree,
												std::vector<Eigen::Matrix3d>& covs)
{

	return true ;
}

template<class T>
void TransSolver6DPCL<T>::build_kdtree(const pcl::PointCloud<T>& pc, pcl::KdTreeFLANN<T>& tree) 
{	
	tree.setInputCloud(pc.makeShared()) ;
}

template class TransSolver6DPCL<pcl::PointXYZ> ;
