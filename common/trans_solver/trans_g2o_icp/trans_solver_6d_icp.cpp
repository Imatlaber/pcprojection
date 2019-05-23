#include <algorithm>
#include <pcl/common/transforms.h>
#include "trans_solver_6d_icp.hpp"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

template<class T>
TransSolver6DICP<T>::TransSolver6DICP():
k_correspondences_(20),
gicp_epsilon_(1e-3),
dist_threshold_(25),
rotation_epsilon_(1e-3),
transformation_epsilon_(1e-3),
max_iterations_(200),
max_optimizer_iterations_(20)
{

}

template<class T>
bool TransSolver6DICP<T>::run()
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
		printf("----------- icp iteration %d ----------- \n", cnt) ; 
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
const Eigen::Isometry3d& TransSolver6DICP<T>::get_trans() const
{
	return trans_ ;
}

template<class T>
bool TransSolver6DICP<T>::solve(const pcl::PointCloud<T>& source_pc,
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
	Vertex_icp * v = new Vertex_icp() ;
	v->setEstimate(trans) ;
	v->setId(0) ;
	optimizer.addVertex(v) ; 
	
	// add edges
	Cache_icp cache ;
	int N = source_indices.size() ;
	for(int i=0; i<N; ++i)
	{
	  Edge_icp * e = new Edge_icp() ;
	  e->setVertex(0, v) ;
	  Meas_icp meas ;
	  const T& p1 = source_pc[source_indices[i]] ;
	  const T& p0 = target_pc[target_indices[i]] ;
	  meas.p1 = Eigen::Vector3d(p1.x, p1.y, p1.z) ;
	  meas.p0 = Eigen::Vector3d(p0.x, p0.y, p0.z) ;
	  meas.cov1 = source_cov[source_indices[i]] ;
	  meas.cov0 = target_cov[target_indices[i]] ;
	  meas.usr_data = (void*)&cache ;
	  e->setMeasurement(meas) ;	
	  e->information() = Eigen::Matrix3d::Identity() ;
	  optimizer.addEdge(e) ;
	}
	
	// optimize
	optimizer.initializeOptimization() ;
	optimizer.computeActiveErrors() ;
	optimizer.optimize(max_optimizer_iterations_) ;
	trans = v->estimate() ;
	
	//v = dynamic_cast<Vertex_icp*>(optimizer.vertex(0)) ;
	//std::cout << "number of edges " << v->edges().size() << std::endl ;
	//g2o::Vector6d est_vec = g2o::internal::toVectorMQT(trans) ;
	//std::cout << "estimate vec " << std::endl ;
	//std::cout << est_vec << std::endl ;

	return true ;
}


template<class T>
void TransSolver6DICP<T>::compute_correspondence(const pcl::PointCloud<T>& source_pc,
												 const pcl::PointCloud<T>& target_pc,   
												 const Eigen::Isometry3d& trans,
													std::vector<int>& source_indices,
													std::vector<int>& target_indices)
{
	/*
	int N = this->source_pc_->size() ;
	source_indices.resize(N) ;
	target_indices.resize(N) ;
	for(int i=0; i<N; ++i)
	{
		source_indices[i] = i ;
		target_indices[i] = i ;
	}
	*/
	std::vector<int> nn_indices (1);
	std::vector<float> nn_dists_sq (1);
	size_t N = std::max(source_pc.size(), target_pc.size()) ;
	source_indices.resize(N) ;
	target_indices.resize(N) ;
	pcl::PointCloud<T> source_transformed_pc ;
	Eigen::Matrix4f trans_mat = trans.matrix().cast<float>() ;

	pcl::transformPointCloud(source_pc, source_transformed_pc, trans_mat) ;
	size_t cnt = 0 ;
    for (size_t i = 0; i < N; i++)
    {
      const T& query = source_transformed_pc[i] ;
      //const T& target_query = target_pc[i] ;
	  //printf("source point %f %f %f \n", query.x, query.y, query.z) ;
	  //printf("transformed target point %f %f %f \n", source_query.x, source_query.y, source_query.z) ;
	  if(!target_tree_.nearestKSearch(query, 1, nn_indices, nn_dists_sq))
	  {
		//printf("error: kdtree search failed \n") ;
        continue;
      }
      
      // Check if the distance to the nearest neighbor is smaller than the user imposed threshold
      if (nn_dists_sq[0] < dist_threshold_)
      {
        source_indices[cnt] = static_cast<int> (i);
        target_indices[cnt] = nn_indices[0];
        cnt++;
      }
    }
    // Resize to the actual number of valid correspondences
    source_indices.resize(cnt); target_indices.resize(cnt);
	//printf("number of correspondence %zu \n", cnt) ;
}

template<class T>
bool TransSolver6DICP<T>::compute_covariances(const pcl::PointCloud<T>& pc,
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
		  if(k == 2)   // smallest singular value replaced by gicp_epsilon
		  {
		    v = gicp_epsilon_;
			//std::cout << "normal " << col.transpose() << std::endl ;	
		  }
		  cov+= v * col * col.transpose(); 
		}
	}

	return true ;
}

template<class T>
void TransSolver6DICP<T>::build_kdtree(const pcl::PointCloud<T>& pc, pcl::KdTreeFLANN<T>& tree) 
{	
	tree.setInputCloud(pc.makeShared()) ;
}


template class TransSolver6DICP<pcl::PointXYZ>  ;












