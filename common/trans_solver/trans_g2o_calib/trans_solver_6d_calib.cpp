#include <algorithm>
#include <pcl/common/transforms.h>
#include "trans_solver_6d_calib.hpp"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

template<class T>
TransSolver6DCALIB<T>::TransSolver6DCALIB():
k_correspondences_(20),
calib_epsilon_(1e-3),
dist_threshold_(25),
rotation_epsilon_(2e-3),
transformation_epsilon_(5e-4),
max_iterations_(200)
{

}

template<class T>
bool TransSolver6DCALIB<T>::run()
{
	std::vector<int> source_indices ;
	std::vector<int> target_indices ;
	std::vector<Eigen::Matrix3d> source_covs ;
	std::vector<Eigen::Matrix3d> target_covs ;

	build_kdtree(*this->source_pc_, source_tree_) ;
	build_kdtree(*this->target_pc_, target_tree_) ;
	
	if(!compute_covariances(*this->source_pc_, source_tree_, source_covs))
		return false ;
	if(!compute_covariances(*this->target_pc_, target_tree_, target_covs)) 
		return false ;

	bool converged = false ;
	int cnt = 0 ;
	Eigen::Isometry3d trans = this->trans0_, prev_trans = this->trans0_ ;
	while(!converged)
	{
		compute_correspondence(*this->source_pc_, *this->target_pc_, trans, source_indices, target_indices) ;	
		printf("----------- calib iteration %d ----------- \n", cnt) ; 
		solve(*this->source_pc_, source_indices, source_covs, *this->target_pc_, target_indices, target_covs, trans) ;
	
		const Eigen::Matrix4d& previous_transformation = prev_trans.matrix() ;
		const Eigen::Matrix4d& transformation = trans.matrix() ;

		/* compute the delta from this iteration */
		double delta = 0.;
		for(int k = 0; k < 4; k++) {
		  for(int l = 0; l < 4; l++) {
		    double ratio = 1;
		    if(k < 3 && l < 3) // rotation part of the transform
		      ratio = 1./rotation_epsilon_;
		    else
		      ratio = 1./transformation_epsilon_;
		    double c_delta = ratio*fabs(previous_transformation(k,l) - transformation(k,l));
		    if(c_delta > delta)
		      delta = c_delta;
		  }
		}
		
		prev_trans = trans ;
		++cnt ;
		if(delta < 1 || cnt > max_iterations_)
			converged = true ;
	}

	trans_ = trans ;
	return true ;
}

template<class T>
const Eigen::Isometry3d& TransSolver6DCALIB<T>::get_trans() const
{
	return trans_ ;
}

template<class T>
bool TransSolver6DCALIB<T>::solve(const pcl::PointCloud<T>& source_pc,
					 const std::vector<int>& source_indices,
					 const std::vector<Eigen::Matrix3d>& source_cov,
					 const pcl::PointCloud<T>& target_pc,
					 const std::vector<int> & target_indices,
					 const std::vector<Eigen::Matrix3d> & target_cov,
					 Eigen::Isometry3d& trans
					 ) 
{
	g2o::SparseOptimizer optimizer;
	optimizer.setVerbose(true);
	
	// variable-size block solver
	g2o::BlockSolverX::LinearSolverType * linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
	g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);
	
	// add vertex
	Vertex_calib * v = new Vertex_calib() ;
	v->setEstimate(trans) ;
	v->setId(0) ;
	optimizer.addVertex(v) ; 
	
	// add edges
	Cache_calib cache ;
	printf("calib\n") ;
	cache.move = *((Eigen::Isometry3d*)(this->data_)) ;
	int N = source_indices.size() ;
	for(int i=0; i<N; ++i)
	{
	  Edge_calib * e = new Edge_calib() ;
	  e->setVertex(0, v) ;
	  Meas_calib meas ;
	  const T& p0 = source_pc[source_indices[i]] ;
	  const T& p1 = target_pc[target_indices[i]] ;
	  meas.p0 = Eigen::Vector3d(p0.x, p0.y, p0.z) ;
	  meas.p1 = Eigen::Vector3d(p1.x, p1.y, p1.z) ;
	  meas.cov0 = source_cov[source_indices[i]] ;
	  meas.cov1 = target_cov[target_indices[i]] ;
	  meas.usr_data = (void*)&cache ;
	  e->setMeasurement(meas) ;	
	  e->information() = Eigen::Matrix3d::Identity() ;
	  optimizer.addEdge(e) ;
	}
	
	// optimize
	optimizer.initializeOptimization() ;
	optimizer.computeActiveErrors() ;
	optimizer.setVerbose(true) ;
	optimizer.optimize(10) ;
	
	v = dynamic_cast<Vertex_calib*>(optimizer.vertex(0)) ;
	//std::cout << "number of edges " << v->edges().size() << std::endl ;
	trans = v->estimate() ;
	g2o::Vector6d est_vec = g2o::internal::toVectorMQT(trans) ;
	std::cout << "estimate vec " << std::endl ;
	std::cout << est_vec << std::endl ;

	return true ;
}


template<class T>
void TransSolver6DCALIB<T>::compute_correspondence( const pcl::PointCloud<T>& source_pc,
													const pcl::PointCloud<T>& target_pc,
													const Eigen::Isometry3d& trans,
													std::vector<int>& source_indices,
													std::vector<int>& target_indices)
{
	int N = this->source_pc_->size() ;
	source_indices.resize(N) ;
	target_indices.resize(N) ;
	for(int i=0; i<N; ++i)
	{
		source_indices[i] = i ;
		target_indices[i] = i ;
	}
	//std::vector<int> nn_indices (1);
	//std::vector<float> nn_dists_sq (1);
	//size_t N = std::max(source_pc.size(), target_pc.size()) ;
	//source_indices.resize(N) ;
	//target_indices.resize(N) ;
	//pcl::PointCloud<T> target_transformed_pc ;
	//Eigen::Matrix4f trans_mat = trans.matrix().cast<float>() ;

	//pcl::transformPointCloud(target_pc, target_transformed_pc, trans_mat) ;
	//size_t cnt = 0 ;
    //for (size_t i = 0; i < N; i++)
    //{
    //  const T& query = target_transformed_pc[i] ;
	//  if(source_tree_.nearestKSearch(query, 1, nn_indices, nn_dists_sq))
	//  {
	//	printf("error: kdtree search failed \n") ;
    //    return;
    //  }
    //  
    //  // Check if the distance to the nearest neighbor is smaller than the user imposed threshold
    //  if (nn_dists_sq[0] < dist_threshold_)
    //  {
    //    target_indices[cnt] = static_cast<int> (i);
    //    source_indices[cnt] = nn_indices[0];
    //    cnt++;
    //  }
    //}
    //// Resize to the actual number of valid correspondences
    //source_indices.resize(cnt); target_indices.resize(cnt);
}

template<class T>
bool TransSolver6DCALIB<T>::compute_covariances(const pcl::PointCloud<T>& pc,
												const pcl::KdTreeFLANN<T>& tree,
												std::vector<Eigen::Matrix3d>& covs)
{
	size_t N = pc.size() ;
	if (k_correspondences_ > int (N))
	{
	  printf("error: requested correspondence %d is greater than number of points %zu ", k_correspondences_, N) ;
	  return false ;
	}
	
	Eigen::Vector3d mean;
	std::vector<int> nn_indecies; nn_indecies.reserve (k_correspondences_);
	std::vector<float> nn_dist_sq; nn_dist_sq.reserve (k_correspondences_);
	
	// We should never get there but who knows
	if(covs.size () < N)
		covs.resize(N) ;
	
	typename pcl::PointCloud<T>::const_iterator points_iterator = pc.begin();
	std::vector<Eigen::Matrix3d>::iterator matrices_iterator = covs.begin();
	for(;points_iterator != pc.end (); ++points_iterator, ++matrices_iterator)
	{
		const T &query_point = *points_iterator;
		Eigen::Matrix3d &cov = *matrices_iterator;
		// Zero out the cov and mean
		cov.setZero ();
		mean.setZero ();
		
		// Search for the K nearest neighbours
		tree.nearestKSearch(query_point, k_correspondences_, nn_indecies, nn_dist_sq);
		
		// Find the covariance matrix
		for(int j = 0; j < k_correspondences_; j++) 
		{
		  const T &pt = pc[nn_indecies[j]];
		  
		  mean[0] += pt.x;
		  mean[1] += pt.y;
		  mean[2] += pt.z;
		  
		  cov(0,0) += pt.x*pt.x;
		  
		  cov(1,0) += pt.y*pt.x;
		  cov(1,1) += pt.y*pt.y;
		  
		  cov(2,0) += pt.z*pt.x;
		  cov(2,1) += pt.z*pt.y;
		  cov(2,2) += pt.z*pt.z;    
		}
		
		mean /= static_cast<double> (k_correspondences_);
		// Get the actual covariance
		for (int k = 0; k < 3; k++)
		  for (int l = 0; l <= k; l++) 
		  {
		    cov(k,l) /= static_cast<double> (k_correspondences_);
		    cov(k,l) -= mean[k]*mean[l];
		    cov(l,k) = cov(k,l);
		  }
		
		// Compute the SVD (covariance matrix is symmetric so U = V')
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU);
		cov.setZero ();
		Eigen::Matrix3d U = svd.matrixU ();
		// Reconstitute the covariance matrix with modified singular values using the column     // vectors in V.
		for(int k = 0; k < 3; k++) 
		{
		  Eigen::Vector3d col = U.col(k);
		  double v = 1.; // biggest 2 singular values replaced by 1
		  if(k == 2)   // smallest singular value replaced by calib_epsilon
		  {
		    v = calib_epsilon_;
			//std::cout << "normal " << col.transpose() << std::endl ;	
		  }
		  cov+= v * col * col.transpose(); 
		}
	}

	return true ;
}

template<class T>
void TransSolver6DCALIB<T>::build_kdtree(const pcl::PointCloud<T>& pc, pcl::KdTreeFLANN<T>& tree) 
{	
	tree.setInputCloud(pc.makeShared()) ;
}


template class TransSolver6DCALIB<pcl::PointXYZ>  ;












