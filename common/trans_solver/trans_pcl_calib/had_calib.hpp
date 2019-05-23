/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */
#ifndef _IMPL_HAD_CALIB_HPP_
#define _IMPL_HAD_CALIB_HPP_

#include <pcl/registration/boost.h>
#include <pcl/registration/exceptions.h>
#include <boost/shared_ptr.hpp>
#include <array>
#include <unordered_map>
#include "had_calib.h"
#include "common/pcl_viewer/pcl_viewer.hpp"
#include "utilities/hadif/math_helper.hpp"

using namespace pcl ;

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::load_params()
{
    param_->setNamespace("calibration.gd") ;
    param_->getParam("draw_cloud_lidar", draw_cloud_lidar_, false) ; 
    param_->getParam("draw_normal_lidar", draw_normal_lidar_, false) ;
    param_->getParam("draw_cloud_world", draw_cloud_world_, false) ;
    param_->getParam("draw_cloud_world_final", draw_cloud_world_final_, false) ;
    param_->getParam("correspondence_type", correspondence_type_, 1) ;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::setInputCloud (
    const typename had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::PointCloudSourceConstPtr &cloud)
{
  setInputSource (cloud);
}

////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> 
template<typename PointT> void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeCovariances(typename pcl::PointCloud<PointT>::ConstPtr cloud, 
                                                                                    const typename pcl::search::KdTree<PointT>::Ptr kdtree,
                                                                                    MatricesVector& cloud_covariances) 
{
  if (k_correspondences_ > int (cloud->size ()))
  {
    PCL_ERROR ("[pcl::CalibGeneralizedIterativeClosestPoint::computeCovariances] Number or points in cloud (%lu) is less than k_correspondences_ (%lu)!\n", cloud->size (), k_correspondences_);
    return;
  }

  Eigen::Vector3d mean;
  std::vector<int> nn_indecies; nn_indecies.reserve (k_correspondences_);
  std::vector<float> nn_dist_sq; nn_dist_sq.reserve (k_correspondences_);

  // We should never get there but who knows
  if(cloud_covariances.size () < cloud->size ())
    cloud_covariances.resize (cloud->size ());

  typename pcl::PointCloud<PointT>::const_iterator points_iterator = cloud->begin ();
  MatricesVector::iterator matrices_iterator = cloud_covariances.begin ();
  for(;
      points_iterator != cloud->end ();
      ++points_iterator, ++matrices_iterator)
  {
    const PointT &query_point = *points_iterator;
    Eigen::Matrix3d &cov = *matrices_iterator;
    // Zero out the cov and mean
    cov.setZero ();
    mean.setZero ();

    // Search for the K nearest neighbours
    kdtree->nearestKSearch(query_point, k_correspondences_, nn_indecies, nn_dist_sq);
    
    // Find the covariance matrix
    for(int j = 0; j < k_correspondences_; j++) {
      const PointT &pt = (*cloud)[nn_indecies[j]];
      
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
    for(int k = 0; k < 3; k++) {
      Eigen::Vector3d col = U.col(k);
      double v = 1.; // biggest 2 singular values replaced by 1
      if(k == 2)   // smallest singular value replaced by gicp_epsilon
        v = gicp_epsilon_;
      cov+= v * col * col.transpose(); 
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> 
template<typename PointT> void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeCovariancesAndNormals(typename pcl::PointCloud<PointT>::ConstPtr cloud, 
                                                                                    const typename pcl::search::KdTree<PointT>::Ptr kdtree,
                                                                                    MatricesVector& cloud_covariances, 
                                                                                    std::vector<Eigen::Vector3f>& normals)
{
  if (k_correspondences_ > int (cloud->size ()))
  {
    PCL_ERROR ("[pcl::CalibGeneralizedIterativeClosestPoint::computeCovariances] Number or points in cloud (%lu) is less than k_correspondences_ (%lu)!\n", cloud->size (), k_correspondences_);
    return;
  }

  Eigen::Vector3d mean;
  std::vector<int> nn_indecies; nn_indecies.reserve (k_correspondences_);
  std::vector<float> nn_dist_sq; nn_dist_sq.reserve (k_correspondences_);

  // We should never get there but who knows
  size_t N = cloud->size() ;
  if(cloud_covariances.size () < cloud->size ())
  {
    cloud_covariances.resize (N);
    normals.resize(N) ;
  }

  typename pcl::PointCloud<PointT>::const_iterator points_iterator = cloud->begin ();
  MatricesVector::iterator matrices_iterator = cloud_covariances.begin ();
  std::vector<Eigen::Vector3f>::iterator normals_iterator = normals.begin() ;
  for(;
      points_iterator != cloud->end ();
      ++points_iterator, ++matrices_iterator, ++normals_iterator)
  {
    const PointT &query_point = *points_iterator;
    Eigen::Matrix3d &cov = *matrices_iterator;
    Eigen::Vector3f& normal = *normals_iterator ;
    // Zero out the cov and mean
    cov.setZero ();
    mean.setZero ();

    // Search for the K nearest neighbours
    kdtree->nearestKSearch(query_point, k_correspondences_, nn_indecies, nn_dist_sq);
    
    // Find the covariance matrix
    for(int j = 0; j < k_correspondences_; j++) {
      const PointT &pt = (*cloud)[nn_indecies[j]];
      
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
    for(int k = 0; k < 3; k++) {
      Eigen::Vector3d col = U.col(k);
      double v = 1.; // biggest 2 singular values replaced by 1
      if(k == 2)   // smallest singular value replaced by gicp_epsilon
      {
        v = gicp_epsilon_;
        normal = col.cast<float>() ;
      }
      cov+= v * col * col.transpose(); 
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> 
template<typename PointT> void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeCovariancesAndNormals(typename pcl::PointCloud<PointT>::ConstPtr cloud, 
                                                                                    const typename pcl::search::KdTree<PointT>::Ptr kdtree,
                                                                                    MatricesVector& cloud_covariances, 
                                                                                    std::vector<Eigen::Vector3f>& normals,
                                                                                    const std::vector<Eigen::Matrix3d>& cov_plane,
                                                                                    const std::vector<Eigen::Vector3d>& normal_plane)
{
    // We should never get there but who knows
    size_t N = cloud->size() ;
    if(cloud_covariances.size () < cloud->size ())
    {
      cloud_covariances.resize (N);
      normals.resize(N) ;
    }
    
    typename pcl::PointCloud<PointT>::const_iterator points_iterator = cloud->begin ();
    MatricesVector::iterator matrices_iterator = cloud_covariances.begin ();
    std::vector<Eigen::Vector3f>::iterator normals_iterator = normals.begin() ;
    for(;
        points_iterator != cloud->end ();
        ++points_iterator, ++matrices_iterator, ++normals_iterator)
    {
      const PointT &query_point = *points_iterator;
      Eigen::Matrix3d &cov = *matrices_iterator;
      Eigen::Vector3f& normal = *normals_iterator ;
      cov = cov_plane[query_point.id] ;
      normal = normal_plane[query_point.id].template cast<float>() ;
    }
}

    
////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeRDerivative(const Vector6d &x, const Eigen::Matrix3d &R, Vector6d& g) const
{
  Eigen::Matrix3d dR_dPhi;
  Eigen::Matrix3d dR_dTheta;
  Eigen::Matrix3d dR_dPsi;

  double phi = x[3], theta = x[4], psi = x[5];
  
  double cphi = cos(phi), sphi = sin(phi);
  double ctheta = cos(theta), stheta = sin(theta);
  double cpsi = cos(psi), spsi = sin(psi);
      
  dR_dPhi(0,0) = 0.;
  dR_dPhi(1,0) = 0.;
  dR_dPhi(2,0) = 0.;

  dR_dPhi(0,1) = sphi*spsi + cphi*cpsi*stheta;
  dR_dPhi(1,1) = -cpsi*sphi + cphi*spsi*stheta;
  dR_dPhi(2,1) = cphi*ctheta;

  dR_dPhi(0,2) = cphi*spsi - cpsi*sphi*stheta;
  dR_dPhi(1,2) = -cphi*cpsi - sphi*spsi*stheta;
  dR_dPhi(2,2) = -ctheta*sphi;

  dR_dTheta(0,0) = -cpsi*stheta;
  dR_dTheta(1,0) = -spsi*stheta;
  dR_dTheta(2,0) = -ctheta;

  dR_dTheta(0,1) = cpsi*ctheta*sphi;
  dR_dTheta(1,1) = ctheta*sphi*spsi;
  dR_dTheta(2,1) = -sphi*stheta;

  dR_dTheta(0,2) = cphi*cpsi*ctheta;
  dR_dTheta(1,2) = cphi*ctheta*spsi;
  dR_dTheta(2,2) = -cphi*stheta;

  dR_dPsi(0,0) = -ctheta*spsi;
  dR_dPsi(1,0) = cpsi*ctheta;
  dR_dPsi(2,0) = 0.;

  dR_dPsi(0,1) = -cphi*cpsi - sphi*spsi*stheta;
  dR_dPsi(1,1) = -cphi*spsi + cpsi*sphi*stheta;
  dR_dPsi(2,1) = 0.;

  dR_dPsi(0,2) = cpsi*sphi - cphi*spsi*stheta;
  dR_dPsi(1,2) = sphi*spsi + cphi*cpsi*stheta;
  dR_dPsi(2,2) = 0.;
      
  g[3] = matricesInnerProd(dR_dPhi, R);
  g[4] = matricesInnerProd(dR_dTheta, R);
  g[5] = matricesInnerProd(dR_dPsi, R);
}

////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::estimateRigidTransformationBFGS (const PointCloudSource &cloud_src, 
                                                                                                  const std::vector<int> &indices_src, 
                                                                                                  const PointCloudTarget &cloud_tgt, 
                                                                                                  const std::vector<int> &indices_tgt, 
                                                                                                  Eigen::Matrix4f &transformation_matrix)
{
  if (indices_src.size () < 4)     // need at least 4 samples
  {
    PCL_THROW_EXCEPTION (NotEnoughPointsException, 
                         "[pcl::CalibGeneralizedIterativeClosestPoint::estimateRigidTransformationBFGS] Need at least 4 points to estimate a transform! Source and target have " << indices_src.size () << " points!");
    return;
  }
  // Set the initial solution
  Vector6d x = Vector6d::Zero ();
  x[0] = transformation_matrix (0,3);
  x[1] = transformation_matrix (1,3);
  x[2] = transformation_matrix (2,3);
  x[3] = atan2 (transformation_matrix (2,1), transformation_matrix (2,2));
  x[4] = asin (-transformation_matrix (2,0));
  x[5] = atan2 (transformation_matrix (1,0), transformation_matrix (0,0));

  // Set temporary pointers
  tmp_src_ = &cloud_src;
  tmp_tgt_ = &cloud_tgt;
  tmp_idx_src_ = &indices_src;
  tmp_idx_tgt_ = &indices_tgt;

  // Optimize using forward-difference approximation LM
  const double gradient_tol = 1e-2;
  OptimizationFunctorWithIndices functor(this);
  BFGS<OptimizationFunctorWithIndices> bfgs (functor);
  bfgs.parameters.sigma = 0.01;
  bfgs.parameters.rho = 0.01;
  bfgs.parameters.tau1 = 9;
  bfgs.parameters.tau2 = 0.05;
  bfgs.parameters.tau3 = 0.5;
  bfgs.parameters.order = 3;

  int inner_iterations_ = 0;
  int result = bfgs.minimizeInit (x);
  result = BFGSSpace::Running;
  do
  {
    inner_iterations_++;
    result = bfgs.minimizeOneStep (x);
    if(result)
    {
      break;
    }
    result = bfgs.testGradient(gradient_tol);
  } while(result == BFGSSpace::Running && inner_iterations_ < max_inner_iterations_);
  if(result == BFGSSpace::NoProgress || result == BFGSSpace::Success || inner_iterations_ == max_inner_iterations_)
  {
    PCL_DEBUG ("[pcl::registration::TransformationEstimationBFGS::estimateRigidTransformation]");
    PCL_DEBUG ("BFGS solver finished with exit code %i \n", result);
    transformation_matrix.setIdentity();
    applyState(transformation_matrix, x);
  }
  else
    PCL_THROW_EXCEPTION(SolverDidntConvergeException, 
                        "[pcl::" << getClassName () << "::TransformationEstimationBFGS::estimateRigidTransformation] BFGS solver didn't converge!");
}

////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline double
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::OptimizationFunctorWithIndices::operator() (const Vector6d& x)
{
  Eigen::Matrix4f transformation_matrix = gicp_->base_transformation_;
  gicp_->applyState(transformation_matrix, x);
  double f = 0;
  int m = static_cast<int> (gicp_->tmp_idx_src_->size ());
  Eigen::Matrix4f trans_src, trans_tgt ;
  for (int i = 0; i < m; ++i)
  {
    // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
    const auto& point_src = gicp_->tmp_src_->points[(*gicp_->tmp_idx_src_)[i]] ;
    Vector4fMapConst p_src = point_src.getVector4fMap ();
    // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
    const auto& point_tgt = gicp_->tmp_tgt_->points[(*gicp_->tmp_idx_tgt_)[i]] ;
    Vector4fMapConst p_tgt = point_tgt.getVector4fMap ();
    // get transformation from body to global frame
    trans_src = gicp_->trans_src_4f_ ;
    trans_tgt = gicp_->trans_tgt_4f_ ;
    Eigen::Vector4f p_global_src (trans_src * transformation_matrix * p_src);
	Eigen::Vector4f p_global_tgt (trans_tgt * transformation_matrix * p_tgt);

    // Estimate the distance (cost function)
    // The last coordiante is still guaranteed to be set to 1.0
    Eigen::Vector3d res(p_global_src[0] - p_global_tgt[0], p_global_src[1] - p_global_tgt[1], p_global_src[2] - p_global_tgt[2]);
    Eigen::Vector3d temp (gicp_->mahalanobis((*gicp_->tmp_idx_src_)[i]) * res);
    //increment= res'*temp/num_matches = temp'*M*temp/num_matches (we postpone 1/num_matches after the loop closes)
    f+= double(res.transpose() * temp);
  }
  return f/m;
}

////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::OptimizationFunctorWithIndices::df (const Vector6d& x, Vector6d& g)
{
  Eigen::Matrix4f transformation_matrix = gicp_->base_transformation_;
  gicp_->applyState(transformation_matrix, x);
  //Zero out g
  g.setZero ();
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero ();
  Eigen::Matrix3d R_src = Eigen::Matrix3d::Zero ();
  Eigen::Matrix3d R_tgt = Eigen::Matrix3d::Zero ();
  const Eigen::Matrix3d& R1 = gicp_->rot_src_ ;
  const Eigen::Matrix3d& R0 = gicp_->rot_tgt_ ;
  int m = static_cast<int> (gicp_->tmp_idx_src_->size ());
  for (int i = 0; i < m; ++i)
  {
    // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_src = gicp_->tmp_src_->points[(*gicp_->tmp_idx_src_)[i]].getVector4fMap ();
    // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_tgt = gicp_->tmp_tgt_->points[(*gicp_->tmp_idx_tgt_)[i]].getVector4fMap ();

    Eigen::Vector4f p_global_src (gicp_->trans_src_4f_ * transformation_matrix * p_src);
	Eigen::Vector4f p_global_tgt (gicp_->trans_tgt_4f_ * transformation_matrix * p_tgt) ;
    // The last coordiante is still guaranteed to be set to 1.0
    Eigen::Vector3d res (p_global_src[0] - p_global_tgt[0], p_global_src[1] - p_global_tgt[1], p_global_src[2] - p_global_tgt[2]);
    // temp = M*res
    Eigen::Vector3d temp (gicp_->mahalanobis ((*gicp_->tmp_idx_src_)[i]) * res);
    // Increment translation gradient
    // g.head<3> ()+= 2*(R1-R0)'*M*res/num_matches (we postpone 2/num_matches after the loop closes)
    g.head<3> ()+= gicp_->rot_diff_.transpose() * temp;
    // Increment rotation gradient
	// R_src += 2*p_src3*(M*res)'*R1/num_matches
	Eigen::Vector4f pp = gicp_->base_transformation_ * p_src;
    Eigen::Vector3d p_src3 (pp[0], pp[1], pp[2]);
    R_src += p_src3 * temp.transpose();
	// R_tgt += 2*p_tgt3*(M*res)'*R0/num_matches
	pp = gicp_->base_transformation_ * p_tgt ;
	Eigen::Vector3d p_tgt3 (pp[0], pp[1], pp[2]) ;
	R_tgt += p_tgt3 * temp.transpose();
  }
  // R = R_src - R_tgt
  R_src *= 2.0/m ;
  R_src *= R1 ;
  R_tgt *= 2.0/m ;
  R_tgt *= R0 ;
  R = R_src - R_tgt ;
  g.head<3> ()*= 2.0/m;
  /// esimate xyz (z is not estimated as it needs changes in pitch and roll) or rpy once a time
  if(gicp_->estimate_xyz_)
      g.tail<4>().setZero() ;
  else
    g.head<3>().setZero() ;
  gicp_->computeRDerivative(x, R, g);
}

////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::OptimizationFunctorWithIndices::fdf (const Vector6d& x, double& f, Vector6d& g)
{
  Eigen::Matrix4f transformation_matrix = gicp_->base_transformation_;
  gicp_->applyState(transformation_matrix, x);
  f = 0;
  g.setZero ();
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero ();
  Eigen::Matrix3d R_src = Eigen::Matrix3d::Zero ();
  Eigen::Matrix3d R_tgt = Eigen::Matrix3d::Zero ();
  const Eigen::Matrix3d& R1 = gicp_->rot_src_ ;
  const Eigen::Matrix3d& R0 = gicp_->rot_tgt_ ;
  const int m = static_cast<const int> (gicp_->tmp_idx_src_->size ());
  for (int i = 0; i < m; ++i)
  {
    // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_src = gicp_->tmp_src_->points[(*gicp_->tmp_idx_src_)[i]].getVector4fMap ();
    // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
    Vector4fMapConst p_tgt = gicp_->tmp_tgt_->points[(*gicp_->tmp_idx_tgt_)[i]].getVector4fMap ();
    
	Eigen::Vector4f p_global_src (gicp_->trans_src_4f_ * transformation_matrix * p_src);
	Eigen::Vector4f p_global_tgt (gicp_->trans_tgt_4f_ * transformation_matrix * p_tgt) ;
    // The last coordiante is still guaranteed to be set to 1.0
    Eigen::Vector3d res (p_global_src[0] - p_global_tgt[0], p_global_src[1] - p_global_tgt[1], p_global_src[2] - p_global_tgt[2]);
    // temp = M*res
    Eigen::Vector3d temp (gicp_->mahalanobis((*gicp_->tmp_idx_src_)[i]) * res);
    // Increment total error
    f+= double(res.transpose() * temp);
    // Increment translation gradient
    // g.head<3> ()+= 2*(R1-R0)'*M*res/num_matches (we postpone 2/num_matches after the loop closes)
    g.head<3> ()+= gicp_->rot_diff_.transpose() * temp;
    // Increment rotation gradient
	// R_src += 2*p_src3*(M*res)'*R1/num_matches
	Eigen::Vector4f pp = gicp_->base_transformation_ * p_src;
    Eigen::Vector3d p_src3 (pp[0], pp[1], pp[2]);
    R_src += p_src3 * temp.transpose();
	// R_tgt += 2*p_tgt3*(M*res)'*R0/num_matches
	pp = gicp_->base_transformation_ * p_tgt ;
	Eigen::Vector3d p_tgt3 (pp[0], pp[1], pp[2]) ;
	R_tgt += p_tgt3 * temp.transpose();
  }
  f/= double(m);
  // R = R_src - R_tgt
  R_src *= 2.0/m ;
  R_src *= R1 ;
  R_tgt *= 2.0/m ;
  R_tgt *= R0 ;
  R = R_src - R_tgt ;
  g.head<3> ()*= double(2.0/m);
  /// esimate xyz (z is not estimated as it needs changes in pitch and roll) or rpy once a time
  if(gicp_->estimate_xyz_)
      g.tail<4>().setZero() ;
  else
    g.head<3>().setZero() ;
  gicp_->computeRDerivative(x, R, g);
}

////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeTransformation (PointCloudSource &output, const Eigen::Matrix4f& guess)
{
  // initialize calibration matrices 
  initCalibMatrices() ;
  
  pcl::IterativeClosestPoint<PointSource, PointTarget>::initComputeReciprocal ();
  using namespace std;
  // Difference between consecutive transforms
  double delta = 0;
  // Get the size of the target
  const size_t N = indices_->size ();
  // Set the mahalanobis matrices to identity
  mahalanobis_.resize (N, Eigen::Matrix3d::Identity ());
  // Compute target cloud covariance matrices
  if ((!target_covariances_) || (target_covariances_->empty ()))
  {
    target_covariances_.reset (new MatricesVector);  
    computeCovariances<PointTarget> (target_, tree_, *target_covariances_);
  }
  // Compute input cloud covariance matrices
  if ((!input_covariances_) || (input_covariances_->empty ()))
  {
    input_covariances_.reset (new MatricesVector);
    computeCovariances<PointSource> (input_, tree_reciprocal_, *input_covariances_);
  }

  base_transformation_ = guess;
  nr_iterations_ = 0;
  converged_ = false;
  double dist_threshold = corr_dist_threshold_ * corr_dist_threshold_;
  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);
  const	Eigen::Matrix3d& R1 = rot_src_ ;
  const Eigen::Matrix3d& R0 = rot_tgt_ ;

  while(!converged_)
  {
    size_t cnt = 0;
    std::vector<int> source_indices (indices_->size ());
    std::vector<int> target_indices (indices_->size ());

    // guess corresponds to base_t and transformation_ to t
    Eigen::Matrix4d transform_R = Eigen::Matrix4d::Zero ();
    for(size_t i = 0; i < 4; i++)
      for(size_t j = 0; j < 4; j++)
        for(size_t k = 0; k < 4; k++)
          transform_R(i,j)+= double(transformation_(i,k)) * double(guess(k,j));

    Eigen::Matrix3d R = transform_R.topLeftCorner<3,3> ();

    for (size_t i = 0; i < N; i++)
    {
      PointSource query = output[i];
      query.getVector4fMap () = guess * query.getVector4fMap ();
      query.getVector4fMap () = transformation_ * query.getVector4fMap ();
	  query.getVector4fMap () = trans_src_4f_ * query.getVector4fMap () ;
	  query.getVector4fMap () = trans_tgt_4f_.inverse() * query.getVector4fMap () ;
	  query.getVector4fMap () = transformation_.inverse() * query.getVector4fMap () ;
	  query.getVector4fMap () = guess.inverse() * query.getVector4fMap () ;

      if (!searchForNeighbors (query, nn_indices, nn_dists))
      {
        PCL_ERROR ("[pcl::%s::computeTransformation] Unable to find a nearest neighbor in the target dataset for point %d in the source!\n", getClassName ().c_str (), (*indices_)[i]);
        return;
      }
      
      // Check if the distance to the nearest neighbor is smaller than the user imposed threshold
      if (nn_dists[0] < dist_threshold)
      {
        Eigen::Matrix3d &C1 = (*input_covariances_)[i];
        Eigen::Matrix3d &C0 = (*target_covariances_)[nn_indices[0]];
        Eigen::Matrix3d &M = mahalanobis_[i];
        // M = R1*R*C1
        M = R1 * R * C1;
        // temp = (R1*R)*C1*(R1*R)' + (R0*R)*C0*(R0*R)'
        Eigen::Matrix3d temp = M * R.transpose() * R1.transpose() ;
        temp+= R0 * R * C0 * R.transpose() * R0.transpose() ;

        // M = temp^-1
        M = temp.inverse ();
        source_indices[cnt] = static_cast<int> (i);
        target_indices[cnt] = nn_indices[0];
        cnt++;
      }
    }
    // Resize to the actual number of valid correspondences
    source_indices.resize(cnt); target_indices.resize(cnt);
    /* optimize transformation using the current assignment and Mahalanobis metrics*/
    previous_transformation_ = transformation_;
    //optimization right here
    try
    {
      rigid_transformation_estimation_(output, source_indices, *target_, target_indices, transformation_);
      /* compute the delta from this iteration */
      delta = 0.;
      for(int k = 0; k < 4; k++) {
        for(int l = 0; l < 4; l++) {
          double ratio = 1;
          if(k < 3 && l < 3) // rotation part of the transform
            ratio = 1./rotation_epsilon_;
          else
            ratio = 1./transformation_epsilon_;
          double c_delta = ratio*fabs(previous_transformation_(k,l) - transformation_(k,l));
          if(c_delta > delta)
            delta = c_delta;
        }
      }
    } 
    catch (PCLException &e)
    {
      PCL_DEBUG ("[pcl::%s::computeTransformation] Optimization issue %s\n", getClassName ().c_str (), e.what ());
      break;
    }
    nr_iterations_++;
    // Check for convergence
    if (nr_iterations_ >= max_iterations_ || delta < 1)
    {
      converged_ = true;
      previous_transformation_ = transformation_;
      PCL_DEBUG ("[pcl::%s::computeTransformation] Convergence reached. Number of iterations: %d out of %d. Transformation difference: %f\n",
                 getClassName ().c_str (), nr_iterations_, max_iterations_, (transformation_ - previous_transformation_).array ().abs ().sum ());
    } 
    else
      PCL_DEBUG ("[pcl::%s::computeTransformation] Convergence failed\n", getClassName ().c_str ());
  }
  //for some reason the static equivalent methode raises an error
  // final_transformation_.block<3,3> (0,0) = (transformation_.block<3,3> (0,0)) * (guess.block<3,3> (0,0));
  // final_transformation_.block <3, 1> (0, 3) = transformation_.block <3, 1> (0, 3) + guess.rightCols<1>.block <3, 1> (0, 3);
  final_transformation_.topLeftCorner (3,3) = previous_transformation_.topLeftCorner (3,3) * guess.topLeftCorner (3,3);
  auto transformed_guess = previous_transformation_.topLeftCorner(3,3) * guess.topRightCorner(3,1) ;
  final_transformation_(0,3) = previous_transformation_(0,3) + transformed_guess(0) ; //+ guess(0,3);
  final_transformation_(1,3) = previous_transformation_(1,3) + transformed_guess(1) ; //+ guess(1,3);
  final_transformation_(2,3) = previous_transformation_(2,3) + transformed_guess(2) ; //+ guess(2,3);

  // Transform the point cloud
  pcl::transformPointCloud (*input_, output, final_transformation_);
}

template <typename PointSource, typename PointTarget> void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::applyState(Eigen::Matrix4f &t, const Vector6d& x) const
{
  // !!! CAUTION Stanford GICP uses the Z Y X euler angles convention
  Eigen::Matrix3f R;
  R = Eigen::AngleAxisf (static_cast<float> (x[5]), Eigen::Vector3f::UnitZ ())
    * Eigen::AngleAxisf (static_cast<float> (x[4]), Eigen::Vector3f::UnitY ())
    * Eigen::AngleAxisf (static_cast<float> (x[3]), Eigen::Vector3f::UnitX ());
  t.topLeftCorner<3,3> ().matrix () = R * t.topLeftCorner<3,3> ().matrix ();
  t.topRightCorner(3,1) = R * t.topRightCorner(3,1) ;
  Eigen::Vector4f T (static_cast<float> (x[0]), static_cast<float> (x[1]), static_cast<float> (x[2]), 0.0f);
  t.col (3) += T;
}

template <typename PointSource, typename PointTarget> void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::exec(const Eigen::Matrix4f& guess)
{
	initCalibPairs() ;
    /// assign guess to initial transformation
    transformation_ =  guess ;
    
    //base_transformation_ = guess ;
    
    double delta = 0;
    int nr_iterations_ = 0;
	bool converged = false;
    estimate_xyz_ = true ; 
	double obj_error = 0.0 ;
	while(!converged)	
    {
        estimate_xyz_ = !estimate_xyz_ ;
        if(estimate_rpy_only_)
            estimate_xyz_ = false ;
        // guess corresponds to base_t and transformation_ to t
        Eigen::Matrix4d transform_R = Eigen::Matrix4d::Zero ();
        for(size_t i = 0; i < 4; i++)
          for(size_t j = 0; j < 4; j++)
            for(size_t k = 0; k < 4; k++)
              transform_R(i,j)+= double(transformation_(i,k)) * double(base_transformation_(k,j));

        Eigen::Matrix3d R = transform_R.topLeftCorner<3,3> ();

        // compute correspondences
        if(!computeCorrespondences(transformation_, R))
            return ;
        
        if(!calib_estimation_(pairs_, transformation_))
            return ;

		/* compute error */
		Vector6f x_i = getXYZRPY(transformation_) ;
		obj_error = functor_(x_i.cast<double>()) ; 	
		printf("current error %f \n", obj_error) ;
        /* compute the delta from this iteration */
        delta = 0.;
        for(int k = 0; k < 4; k++) {
          for(int l = 0; l < 4; l++) {
            double ratio = 1;
            if(k < 3 && l < 3) // rotation part of the transform
              ratio = 1./rotation_epsilon_;
            else
              ratio = 1./transformation_epsilon_;
            double c_delta = ratio*fabs(previous_transformation_(k,l) - transformation_(k,l));
            if(c_delta > delta)
              delta = c_delta;
          }
        }
        nr_iterations_++;
        // check for convergence
        printf("max_iteration %d, curr_iteration %d \n", max_iterations_, nr_iterations_) ;
        if (delta < 1 || nr_iterations_ > max_iterations_)
        {
		  if(obj_error < max_convergence_error_ || nr_iterations_ > max_iterations_)
		  {
            converged = true;
            printf("[pcl::computeTransformation] Convergence reached. Number of iterations: %d out of %d. Transformation difference: %f\n",
                       nr_iterations_, max_iterations_, (transformation_ - previous_transformation_).array ().abs ().sum ());
		  }
		  else
		  {
			max_nearest_search_ += 5 ;
			printf("error is still large: use %d \n", max_nearest_search_) ; 
		  }
        } 
        previous_transformation_ = transformation_;
    }

    final_transformation_.topLeftCorner (3,3) = previous_transformation_.topLeftCorner (3,3) * base_transformation_.topLeftCorner (3,3);
    auto transformed_guess = previous_transformation_.topLeftCorner(3,3) * base_transformation_.topRightCorner(3,1) ;
    final_transformation_(0,3) = previous_transformation_(0,3) + transformed_guess(0) ; //+ guess(0,3);
    final_transformation_(1,3) = previous_transformation_(1,3) + transformed_guess(1) ; //+ guess(1,3);
    final_transformation_(2,3) = previous_transformation_(2,3) + transformed_guess(2) ; //+ guess(2,3);
    
}

template <typename PointSource, typename PointTarget> void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::add_pair(PointCloudSourceConstPtr pc_src, 
																			    const Eigen::Matrix4d& trans_src,
                                                                                PointCloudTargetConstPtr pc_tgt, 
																			    const Eigen::Matrix4d& trans_tgt) 
{
	CalibPair pair ;
	pair.pc_src_ = pc_src ;
	pair.trans_src_ = trans_src ;
	pair.pc_tgt_ = pc_tgt ;
	pair.trans_tgt_ = trans_tgt ;
	pairs_.push_back(pair) ;
}

template <typename PointSource, typename PointTarget> void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::add_pair(PointCloudSourceConstPtr pc_src, 
																			    const Eigen::Matrix4d& trans_src,
                                                                                const std::vector<Eigen::Matrix3d>& cov_src,
                                                                                const std::vector<Eigen::Vector3d>& normal_src,
                                                                                PointCloudTargetConstPtr pc_tgt, 
																			    const Eigen::Matrix4d& trans_tgt,
                                                                                const std::vector<Eigen::Matrix3d>& cov_tgt,
                                                                                const std::vector<Eigen::Vector3d>& normal_tgt) 
{
	CalibPair pair ;
	pair.pc_src_ = pc_src ;
	pair.trans_src_ = trans_src ;
	pair.pc_tgt_ = pc_tgt ;
	pair.trans_tgt_ = trans_tgt ;
    pair.cov_src_plane_ =  cov_src ;
    pair.normal_src_plane_ = normal_src ;
    pair.cov_tgt_plane_ = cov_tgt ;
    pair.normal_tgt_plane_ = normal_tgt ;

	pairs_.push_back(pair) ;
    

}

template <typename PointSource, typename PointTarget> 
bool had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::initCalibPairs()
{
	for(auto& p : pairs_)
		initCalibPair(p) ;

    base_transformation_ = final_transformation_ = transformation_ = previous_transformation_ = Eigen::Matrix4f::Identity ();

	return true ;
}

template <typename PointSource, typename PointTarget> 
bool had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::initCalibPair(CalibPair& p)
{
	// build kdtree
	p.tree_src_ = KdTreePtrSrc(new KdTreeSrc) ;
	p.tree_tgt_ = KdTreePtrTgt(new KdTreeTgt) ;

	p.tree_src_->setInputCloud(p.pc_src_) ;
	p.tree_tgt_->setInputCloud(p.pc_tgt_) ;

	// build covariances 
    if(p.cov_src_plane_.size() == 0)
    {
	    computeCovariancesAndNormals<PointSource> (p.pc_src_, p.tree_src_, p.cov_src_, p.normal_src_) ;
	    computeCovariancesAndNormals<PointTarget> (p.pc_tgt_, p.tree_tgt_, p.cov_tgt_, p.normal_tgt_) ;
    }
    else
    {
	    computeCovariancesAndNormals<PointSource> (p.pc_src_, p.tree_src_, p.cov_src_, p.normal_src_, p.cov_src_plane_, p.normal_src_plane_) ;
	    computeCovariancesAndNormals<PointTarget> (p.pc_tgt_, p.tree_tgt_, p.cov_tgt_, p.normal_tgt_, p.cov_tgt_plane_, p.normal_tgt_plane_) ;
    }

	// prepare internal matrices
	p.rot_src_ = p.trans_src_.template topLeftCorner<3,3>() ;
	p.rot_tgt_ = p.trans_tgt_.template topLeftCorner<3,3>() ;
	p.trans_src_4f_ = p.trans_src_.template cast<float>() ;
	p.trans_tgt_4f_ = p.trans_tgt_.template cast<float>() ;
	Eigen::Matrix3d T1 = p.trans_src_.template topLeftCorner<3,3> () ;
	Eigen::Matrix3d T0 = p.trans_tgt_.template topLeftCorner<3,3> () ;
	p.rot_diff_ = T1-T0 ;

    /// set transformation lookup table 
    if(use_point_time_)
    {
        for(const PointSource& point : p.pc_src_->points)
        {
            set_trans(point, p.trans_lookup_src_) ;
        }
        for(const PointTarget& point : p.pc_tgt_->points)
        {
            set_trans(point, p.trans_lookup_tgt_) ;
        }
    }

    /// covariance viewer 
    boost::shared_ptr<had::PCLViewer> viewer ;
    std::array<int,3> color_src{{255, 0, 0}} ;
    std::array<int,3> color_tgt{{0, 255, 0}} ;
    
    if(draw_cloud_lidar_ || draw_normal_lidar_) 
    {
        viewer = boost::shared_ptr<had::PCLViewer>(new had::PCLViewer()) ;
        viewer->initialize() ;
    }

    if(draw_cloud_lidar_)
    {
        viewer->add_cloud_to_viewer<PointSource>(p.pc_src_, color_src,  std::string("cov_src")) ;
        viewer->add_cloud_to_viewer<PointTarget>(p.pc_tgt_, color_tgt,  std::string("cov_tgt")) ;
    }

    if(draw_normal_lidar_)
    {
        drawNormal(viewer, *p.pc_src_, p.cov_src_, color_src, "src") ;
        drawNormal(viewer, *p.pc_tgt_, p.cov_tgt_, color_tgt, "tgt") ;
    }
    
    if(draw_cloud_lidar_ || draw_normal_lidar_) 
        viewer->show() ;

	return true ;
}

template <typename PointSource, typename PointTarget> 
template<typename PointT>
void had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::
drawNormal(boost::shared_ptr<had::PCLViewer> viewer, 
                const pcl::PointCloud<PointT>& cloud,
                const MatricesVector& covariances,
                const std::array<int,3> color,
                const char * base_name) 
{
    char arrow_name[50] ; 
    for(size_t i=0; i<covariances.size() ; ++i)
    {
        const auto& cov = covariances[i] ;
        const auto& point_start = cloud[i] ;
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU) ;
        Eigen::Matrix3d U = svd.matrixU() ;
        Eigen::Vector3d col = U.col(2) ;
        Eigen::Vector3d v_start(point_start.x, point_start.y, point_start.z) ;
        Eigen::Vector3d v_end = v_start + col ;
        PointSource point_end ;
        point_end.x = v_end[0] ;
        point_end.y = v_end[1] ;
        point_end.z = v_end[2] ;
        sprintf(arrow_name, "%s_%zu", base_name, i) ;
        viewer->add_arrow(point_end, point_start, color, arrow_name) ; 
    }
    
}

template <typename PointSource, typename PointTarget> 
void had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::
drawCloudWorld(const Eigen::Matrix4f& curr_trans,
               const CalibPair& pair, 
               std::string winName)
{
    /// transform cloud from lidar to world frame
    Eigen::Matrix4f trans_l2b = curr_trans * base_transformation_ ;  
    pcl::PointCloud<PointSource> cloud_world_src ;
    transformTimedPointCloud(*pair.pc_src_, 
                             pair.trans_lookup_src_, 
                             pair.trans_src_4f_,
                             trans_l2b,
                             cloud_world_src) ;

    pcl::PointCloud<PointSource> cloud_world_tgt ;
    transformTimedPointCloud(*pair.pc_tgt_, 
                             pair.trans_lookup_tgt_, 
                             pair.trans_tgt_4f_,
                             trans_l2b,
                             cloud_world_tgt) ;

    //pcl::transformPointCloud(*pair.pc_src_, cloud_world_src, trans_l2w_src) ;
    //pcl::transformPointCloud(*pair.pc_tgt_, cloud_world_tgt, trans_l2w_tgt) ;
    
    /// create viewer object 
    boost::shared_ptr<had::PCLViewer> viewer ;
    std::array<int,3> color_src{{255, 0, 0}} ;
    std::array<int,3> color_tgt{{0, 255, 0}} ;

    viewer = boost::shared_ptr<had::PCLViewer>(new had::PCLViewer(winName)) ;
    viewer->initialize() ;
 
    /// add pointcloud
    viewer->add_cloud_to_viewer<PointSource>(cloud_world_src.makeShared(), color_src, std::string("cloud_world_src")) ; 
    viewer->add_cloud_to_viewer<PointTarget>(cloud_world_tgt.makeShared(), color_tgt, std::string("cloud_world_tgt")) ; 
    
    /// add correspondence
    char line_name[50] ;
    std::array<int,3> color_line{{100, 100, 100}} ;
    for(size_t i=0; i<pair.indices_src_.size(); ++i)
    {
        const auto& pt_src = cloud_world_src[pair.indices_src_[i]] ;
        const auto& pt_tgt = cloud_world_tgt[pair.indices_tgt_[i]] ;
        sprintf(line_name, "line_%zu", i) ;
        viewer->add_line(pt_src, pt_tgt, color_line, line_name) ;
    }

    viewer->show() ; 
}

template <typename PointSource, typename PointTarget> 
void had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::updateMahalanobis(const Eigen::Matrix3d& curr_rot) 
{
    Eigen::Matrix3d R1, R0 ;
    for(auto& p : pairs_)
    {
	    //const auto& R1 = p.rot_src_ ;
	    //const auto& R0 = p.rot_tgt_ ;
	    const auto& R  = curr_rot ;
        for(size_t i=0; i<p.indices_src_.size(); ++i)
        {
            const Eigen::Matrix3d& C1 = p.cov_src_[p.indices_src_[i]] ;
            const Eigen::Matrix3d& C0 = p.cov_tgt_[p.indices_tgt_[i]] ;
            
            R1 = get_trans((*p.pc_src_)[p.indices_src_[i]], p.trans_lookup_src_, p.trans_src_4f_).template topLeftCorner<3,3>().template cast<double>() ;    
            R0 = get_trans((*p.pc_tgt_)[p.indices_tgt_[i]], p.trans_lookup_tgt_, p.trans_tgt_4f_).template topLeftCorner<3,3>().template cast<double>() ;    

            p.mahalanobis_[i] = computeMahalanobis(C1, R1, C0, R0, R) ; 
        }
    }
}

template <typename PointSource, typename PointTarget> 
Eigen::Matrix3d had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeMahalanobis(const Eigen::Matrix3d& C1,
                                                                                                           const Eigen::Matrix3d& R1, 
                                                                                                           const Eigen::Matrix3d& C0, 
                                                                                                           const Eigen::Matrix3d& R0, 
                                                                                                           const Eigen::Matrix3d& R) 
{
    // M = R1*R*C1
    Eigen::Matrix3d M = R1 * R * C1;
    // temp = (R1*R)*C1*(R1*R)' + (R0*R)*C0*(R0*R)'
    Eigen::Matrix3d temp = M * R.transpose() * R1.transpose() ;
    temp+= R0 * R * C0 * R.transpose() * R0.transpose() ;
    // temp = temp.inverse() 
    return temp.inverse() ;
}
                                                                            
        
template <typename PointSource, typename PointTarget> 
bool had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeCorrespondences(const Vector6d& x) 
{
    initCalibPairs() ;

    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity() ;
    applyState(trans, x) ;

    Eigen::Matrix3d rot = trans.topLeftCorner<3,3>().cast<double>() ;

    return computeCorrespondences(trans, rot) ;

}

template <typename PointSource, typename PointTarget> 
bool had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeCorrespondences(const Eigen::Matrix4f& curr_trans, const Eigen::Matrix3d& curr_rot)
{
    static int cnt = 0 ;
    printf("----------------iteration %d ---------------- \n", cnt++) ;
    int pair_index = 0 ;
	for(auto& p : pairs_)
	{
        if(correspondence_type_ == 0)
        {
		    if(!computeCorrespondence(curr_trans, curr_rot, p))
		    	return false ;
        }
        else if(correspondence_type_ == 1)
        {
		    if(!computeCorrespondence2(curr_trans, curr_rot, p, pair_index))
		    	return false ;
        }
        else
        {
		    if(!computeCorrespondence3(curr_trans, curr_rot, p))
		    	return false ;
        }
       
        ++pair_index ;

        char winName[50] ;
        sprintf(winName, "pair %d", cnt) ;
        if(draw_cloud_world_)
            drawCloudWorld(curr_trans, p, winName) ;
	}
	return true ;
}

template <typename PointSource, typename PointTarget> 
bool had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeCorrespondence(const Eigen::Matrix4f& curr_trans,  const Eigen::Matrix3d& curr_rot, CalibPair& p)
{
	const size_t N = p.pc_src_->size() ;
	p.indices_src_.resize(N, -1) ;
	p.indices_tgt_.resize(N, -1) ;
	p.mahalanobis_.resize(N, Eigen::Matrix3d::Identity()) ;
	p.timed_rot_diff_.resize(N, Eigen::Matrix3d::Identity()) ;
	std::vector<int> nn_indices(1) ;
	std::vector<float> nn_dists(1) ;
    double dist_threshold = corr_dist_threshold_ * corr_dist_threshold_;

    Eigen::Matrix3d R1, R0 ;

	//const auto& R1 = p.rot_src_ ;
	//const auto& R0 = p.rot_tgt_ ;
	const auto& R  = curr_rot ;

	size_t cnt = 0 ;
    for (size_t i = 0; i < N; i++)
    {
      PointSource query = (*p.pc_src_)[i];
      query.getVector4fMap () = base_transformation_* query.getVector4fMap ();
      query.getVector4fMap () = curr_trans * query.getVector4fMap ();
	  query.getVector4fMap () = p.trans_src_4f_ * query.getVector4fMap () ;
	  query.getVector4fMap () = p.trans_tgt_4f_.inverse() * query.getVector4fMap () ;
	  query.getVector4fMap () = curr_trans.inverse() * query.getVector4fMap () ;
	  query.getVector4fMap () = base_transformation_.inverse() * query.getVector4fMap () ;

      if (!searchNearestNeighbors (*p.tree_tgt_, query, 1, nn_indices, nn_dists))
      {
        PCL_ERROR ("[pcl::%s::computeTransformation] Unable to find a nearest neighbor in the target dataset for point %d in the source!\n", getClassName ().c_str (), (*indices_)[i]);
        continue ;
      }
      
      // Check if the distance to the nearest neighbor is smaller than the user imposed threshold
      if (nn_dists[0] < dist_threshold)
      {
        const Eigen::Matrix3d &C1 = p.cov_src_[i];
        const Eigen::Matrix3d &C0 = p.cov_tgt_[nn_indices[0]];
    
        R1 = get_trans((*p.pc_src_)[i], p.trans_lookup_src_, p.trans_src_4f_).template topLeftCorner<3,3>().template cast<double>() ;    
        R0 = get_trans((*p.pc_tgt_)[nn_indices[0]], p.trans_lookup_tgt_, p.trans_tgt_4f_).template topLeftCorner<3,3>().template cast<double>() ;    
        
        p.mahalanobis_[cnt] = computeMahalanobis(C1, R1, C0, R0, R) ; 
        p.timed_rot_diff_[cnt] = R1-R0 ;
        p.indices_src_[cnt] = static_cast<int> (i);
        p.indices_tgt_[cnt] = nn_indices[0];
        cnt++;
      }
    }

	p.indices_src_.resize(cnt) ;
	p.indices_tgt_.resize(cnt) ;
    // note mahalanobis is different from original implementation!!!
	p.mahalanobis_.resize(cnt) ;
	p.timed_rot_diff_.resize(cnt) ;

	return true ;
}

template <typename PointSource, typename PointTarget> 
bool had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeCorrespondence2(const Eigen::Matrix4f& curr_trans,  
                                                                                                 const Eigen::Matrix3d& curr_rot, 
                                                                                                 CalibPair& p,
                                                                                                 int pair_index)
{
	const size_t N = p.pc_src_->size() ;
	p.indices_src_.resize(N, -1) ;
	p.indices_tgt_.resize(N, -1) ;
	p.mahalanobis_.resize(N, Eigen::Matrix3d::Identity()) ;
	p.timed_rot_diff_.resize(N, Eigen::Matrix3d::Identity()) ;
    int num_nearest = std::min(max_nearest_search_, (int)p.pc_tgt_->size()) ;
	std::vector<int> nn_indices(num_nearest) ;
	std::vector<float> nn_dists(num_nearest) ;
    std::unordered_map<int, int> hash ;
    hash.reserve(N) ;
    double dist_threshold = corr_dist_threshold_ * corr_dist_threshold_;

    Eigen::Matrix3d R1, R0 ;
	//const auto& R1 = p.rot_src_ ;
	//const auto& R0 = p.rot_tgt_ ;
	const auto& R  = curr_rot ;

	size_t cnt = 0 ;
    for (size_t i = 0; i < N; i++)
    {
      PointSource query = (*p.pc_src_)[i];
      query.getVector4fMap () = base_transformation_* query.getVector4fMap ();
      query.getVector4fMap () = curr_trans * query.getVector4fMap ();
	  query.getVector4fMap () = p.trans_src_4f_ * query.getVector4fMap () ;
	  query.getVector4fMap () = p.trans_tgt_4f_.inverse() * query.getVector4fMap () ;
	  query.getVector4fMap () = curr_trans.inverse() * query.getVector4fMap () ;
	  query.getVector4fMap () = base_transformation_.inverse() * query.getVector4fMap () ;

      if (!searchNearestNeighbors (*p.tree_tgt_, query, num_nearest, nn_indices, nn_dists))
      {
        PCL_ERROR ("[pcl::%s::computeTransformation] Unable to find a nearest neighbor in the target dataset for point %d in the source!\n", getClassName ().c_str (), (*indices_)[i]);
        continue ;
      }
     
      /// go though the nearest neighbours to find the least-used nearest point
      int index_tgt = -1, min_cnt = INT_MAX ;
      for(int j=0; j<num_nearest; ++j)
      {
        // Check if the distance to the nearest neighbor is smaller than the user imposed threshold
        if (nn_dists[j] < dist_threshold)
        {
          if(hash.find(nn_indices[j]) == hash.end())
          {
            hash[nn_indices[j]] = 0 ;
            index_tgt = nn_indices[j] ;
            break ;
          }
          else
          {
            if(hash[nn_indices[j]] < min_cnt)
            {
               min_cnt = hash[nn_indices[j]] ;
               index_tgt = nn_indices[j] ; 
            }
          }
        }
      }
        
      if(index_tgt != -1)
      { 
        /// check if their differences are smaller than a threshold 
        Eigen::Vector3f normal_src = base_transformation_.topLeftCorner(3,3) * p.normal_src_[i] ;
        normal_src = curr_trans.topLeftCorner(3,3) * normal_src ;
        normal_src = p.trans_src_4f_.topLeftCorner(3,3) * normal_src ;
        Eigen::Vector3f normal_tgt = base_transformation_.topLeftCorner(3,3) * p.normal_tgt_[index_tgt] ;
        normal_tgt = curr_trans.topLeftCorner(3,3) * normal_tgt ;
        normal_tgt = p.trans_tgt_4f_.topLeftCorner(3,3) * normal_tgt ;
        double normal_diff = angleBtwVectors3d(normal_src.data(), normal_tgt.data()) ;
        //printf("normal_diff %g, max_normal_diff %g \n", normal_diff, max_normal_diff_) ; 
        if(normal_diff < max_normal_diff_ || fabs(normal_diff - M_PI) < max_normal_diff_)
        {
            /// compute informaiton matrix
            hash[index_tgt] += 1 ;
            const Eigen::Matrix3d &C1 = p.cov_src_[i];
            const Eigen::Matrix3d &C0 = p.cov_tgt_[index_tgt];
            
            R1 = get_trans((*p.pc_src_)[i], p.trans_lookup_src_, p.trans_src_4f_).template topLeftCorner<3,3>().template cast<double>() ;    
            R0 = get_trans((*p.pc_tgt_)[index_tgt], p.trans_lookup_tgt_, p.trans_tgt_4f_).template topLeftCorner<3,3>().template cast<double>() ;    

            p.mahalanobis_[cnt] = computeMahalanobis(C1, R1, C0, R0, R) ; 
            p.timed_rot_diff_[cnt] = R1-R0 ;
            p.indices_src_[cnt] = static_cast<int> (i);
            p.indices_tgt_[cnt] = index_tgt;
            cnt++;
        }
      }
    }
   
    printf("pair %d : %d correspondences found \n", pair_index, cnt) ;

	p.indices_src_.resize(cnt) ;
	p.indices_tgt_.resize(cnt) ;
    // note mahalanobis is different from original implementation!!!
	p.mahalanobis_.resize(cnt) ;
	p.timed_rot_diff_.resize(cnt) ;

	return true ;
}

template <typename PointSource, typename PointTarget> 
bool had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeCorrespondence3(const Eigen::Matrix4f& curr_trans,  const Eigen::Matrix3d& curr_rot, CalibPair& p)
{
	const size_t N = p.pc_src_->size() ;
	p.indices_src_.resize(N, -1) ;
	p.indices_tgt_.resize(N, -1) ;
	p.mahalanobis_.resize(N, Eigen::Matrix3d::Identity()) ;
	p.timed_rot_diff_.resize(N, Eigen::Matrix3d::Identity()) ;
	std::vector<int> nn_indices(1) , nn_rindices(1);
	std::vector<float> nn_dists(1), nn_rdists(1) ;
    double dist_threshold = corr_dist_threshold_ * corr_dist_threshold_;

    Eigen::Matrix3d R1, R0 ;
	//const auto& R1 = p.rot_src_ ;
	//const auto& R0 = p.rot_tgt_ ;
	const auto& R  = curr_rot ;

	size_t cnt = 0 ;
    for (size_t i = 0; i < N; i++)
    {
      PointSource query = (*p.pc_src_)[i];
      query.getVector4fMap () = base_transformation_* query.getVector4fMap ();
      query.getVector4fMap () = curr_trans * query.getVector4fMap ();
	  query.getVector4fMap () = p.trans_src_4f_ * query.getVector4fMap () ;
	  query.getVector4fMap () = p.trans_tgt_4f_.inverse() * query.getVector4fMap () ;
	  query.getVector4fMap () = curr_trans.inverse() * query.getVector4fMap () ;
	  query.getVector4fMap () = base_transformation_.inverse() * query.getVector4fMap () ;

      if (!searchNearestNeighbors (*p.tree_tgt_, query, 1, nn_indices, nn_dists))
      {
        PCL_ERROR ("[pcl::%s::computeTransformation] Unable to find a nearest neighbor in the target dataset for point %d in the source!\n", getClassName ().c_str (), (*indices_)[i]);
        continue ;
      }
      
      // Check if the distance to the nearest neighbor is smaller than the user imposed threshold
      if (nn_dists[0] < dist_threshold)
      {
        /// check if the reversal mapping holds 
        PointSource rquery = (*p.pc_tgt_)[nn_indices[0]];
        rquery.getVector4fMap () = base_transformation_* rquery.getVector4fMap ();
        rquery.getVector4fMap () = curr_trans * rquery.getVector4fMap ();
	    rquery.getVector4fMap () = p.trans_tgt_4f_ * rquery.getVector4fMap () ;
	    rquery.getVector4fMap () = p.trans_src_4f_.inverse() * rquery.getVector4fMap () ;
	    rquery.getVector4fMap () = curr_trans.inverse() * rquery.getVector4fMap () ;
	    rquery.getVector4fMap () = base_transformation_.inverse() * rquery.getVector4fMap () ;
        
        if (!searchNearestNeighbors (*p.tree_src_, rquery, 1, nn_rindices, nn_rdists))
        {
          PCL_ERROR ("[pcl::%s::computeTransformation] Unable to find a nearest neighbor in the target dataset for point %d in the target!\n", getClassName ().c_str (), nn_indices[0]);
          continue ;
        }
     
        if(nn_rindices[0] == i)
        {
            const Eigen::Matrix3d &C1 = p.cov_src_[i];
            const Eigen::Matrix3d &C0 = p.cov_tgt_[nn_indices[0]];
            
            R1 = get_trans((*p.pc_src_)[i], p.trans_lookup_src_, p.trans_src_4f_).template topLeftCorner<3,3>().template cast<double>() ;    
            R0 = get_trans((*p.pc_tgt_)[nn_indices[0]], p.trans_lookup_tgt_, p.trans_tgt_4f_).template topLeftCorner<3,3>().template cast<double>() ;    

            p.mahalanobis_[cnt] = computeMahalanobis(C1, R1, C0, R0, R) ; 
            p.timed_rot_diff_[cnt] = R1-R0 ;
            p.indices_src_[cnt] = static_cast<int> (i);
            p.indices_tgt_[cnt] = nn_indices[0];
            cnt++;
        }
      }
    }

	p.indices_src_.resize(cnt) ;
	p.indices_tgt_.resize(cnt) ;
    // note mahalanobis is different from original implementation!!!
	p.mahalanobis_.resize(cnt) ;
	p.timed_rot_diff_.resize(cnt) ;

	return true ;

}

template <typename PointSource, typename PointTarget> 
template<typename PointT>
bool had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::searchNearestNeighbors(const pcl::search::KdTree<PointT>& tree,  
																							    	const PointT &query, 
																							    	int num, 
																							    	std::vector<int>& index, 
																							    	std::vector<float>& distance) 
{
        int k = tree.nearestKSearch (query, num, index, distance);
        if (k == 0)
          return (false);
        return (true);
}

template <typename PointSource, typename PointTarget> 
bool had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::estimateCalibBFGS(const std::vector<CalibPair>& pairs,
                                                                                             Eigen::Matrix4f& transformation_matrix)
{
    int cnt_src = 0, cnt_tgt = 0 ;
    for(const auto& p : pairs)
    {
        cnt_src += p.indices_src_.size() ;
        cnt_tgt += p.indices_tgt_.size() ;
    }
    if(cnt_src < 4)
    {
        printf("[pcl::CalibGeneralizedIterativeClosestPoint::estimateRigidTransformationBFGS] Need at least 4 points to estimate a transform! Source and target have %d points \n", cnt_src);
        return false ;
    }
    if(cnt_tgt < 4)
    {
        printf("[pcl::CalibGeneralizedIterativeClosestPoint::estimateRigidTransformationBFGS] Need at least 4 points to estimate a transform! Source and target have %d points \n", cnt_tgt);
        return false ;
    }

    // Set the initial solution
    Vector6d x = Vector6d::Zero ();
    x[0] = transformation_matrix (0,3);
    x[1] = transformation_matrix (1,3);
    x[2] = transformation_matrix (2,3);
    x[3] = atan2 (transformation_matrix (2,1), transformation_matrix (2,2));
    x[4] = asin (-transformation_matrix (2,0));
    x[5] = atan2 (transformation_matrix (1,0), transformation_matrix (0,0));
    
    // Set temporary pointers
    pairs_ptr_ = std::make_shared< const std::vector<CalibPair> >(pairs) ;
    
    // Optimize using forward-difference approximation LM
    const double gradient_tol = 1e-2;
    CalibOptimizationFunctorWithIndices functor(this);
    BFGS<CalibOptimizationFunctorWithIndices> bfgs (functor);
    bfgs.parameters.sigma = 0.01;
    bfgs.parameters.rho = 0.01;
    bfgs.parameters.tau1 = 9;
    bfgs.parameters.tau2 = 0.05;
    bfgs.parameters.tau3 = 0.5;
    bfgs.parameters.order = 3;
    
    int inner_iterations_ = 0;
    int result = bfgs.minimizeInit (x);
    result = BFGSSpace::Running;
    do
    {
      inner_iterations_++;
      result = bfgs.minimizeOneStep (x);
      if(result)
      {
        break;
      }
      result = bfgs.testGradient(gradient_tol);
    } while(result == BFGSSpace::Running && inner_iterations_ < max_inner_iterations_);
    if(result == BFGSSpace::NoProgress || result == BFGSSpace::Success || inner_iterations_ == max_inner_iterations_)
    {
      PCL_DEBUG ("[pcl::registration::TransformationEstimationBFGS::estimateRigidTransformation]");
      PCL_DEBUG ("BFGS solver finished with exit code %i \n", result);
      transformation_matrix.setIdentity();
      applyState(transformation_matrix, x);
    }
    else
    {
        printf("[pcl::TransformationEstimationBFGS::estimateRigidTransformation] BFGS solver didn't converge!");
        return false ;
    }
    return true ;
}

template <typename PointSource, typename PointTarget> 
Vector6f had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::getFinalXYZRPY()
{
    Vector6f xyzrpy ;
    Eigen::Matrix4f result = this->getFinalTransformation() ;
    Eigen::Vector3f ypr = result.topLeftCorner<3,3>().eulerAngles(2, 1, 0) ;
    xyzrpy(3) = ypr(2) ;
    xyzrpy(4) = ypr(1) ;
    xyzrpy(5) = ypr(0) ;
    xyzrpy.head<3>() = result.topRightCorner<3,1>() ;
    return xyzrpy ;
}

template <typename PointSource, typename PointTarget> 
Vector6f had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::getXYZRPY(const Eigen::Matrix4f& trans)
{
    Vector6f xyzrpy ;
    Eigen::Vector3f ypr = trans.topLeftCorner<3,3>().eulerAngles(2, 1, 0) ;
    xyzrpy(3) = ypr(2) ;
    xyzrpy(4) = ypr(1) ;
    xyzrpy(5) = ypr(0) ;
    xyzrpy.head<3>() = trans.topRightCorner<3,1>() ;
    return xyzrpy ;
}

template <typename PointSource, typename PointTarget> void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::setGicpEpsilon(double e)
{
    gicp_epsilon_ = e ;
}

template <typename PointSource, typename PointTarget> double
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeError(const Vector6d& x, std::string solName)
{
    /// draw cloud and correspondence for x in world frame 
    base_transformation_.setIdentity() ;
    Eigen::Matrix4f transformation_matrix = base_transformation_ ;
    applyState(transformation_matrix, x) ;
    
    /// have to update mahalanobis matrices according to current solution
    updateMahalanobis(transformation_matrix.topLeftCorner<3,3>().cast<double>()) ;
    
    if(draw_cloud_world_final_)
    {
        for(const auto& p : pairs_)
        {
            drawCloudWorld(transformation_matrix, p, solName) ;
        }
    }

    return functor_(x) ;
}

template <typename PointSource, typename PointTarget> void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::snoop2(const Vector6d& x, std::string solName)
{
    Eigen::Matrix4f transformation_matrix = base_transformation_;
    applyState(transformation_matrix, x);
    double f = 0;
    int cnt = 0 ;
    Eigen::Matrix4f trans_src, trans_tgt ; 
    for(const auto& p : pairs_)
    {
      int m = static_cast<int> (p.indices_src_.size ());
      for (int i = 0; i < m; ++i)
      {
        // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
        Vector4fMapConst p_src = p.pc_src_->points[p.indices_src_[i]].getVector4fMap ();
        //printf("source point: %f %f %f %f \n", p_src[0], p_src[1], p_src[2], p_src[3]) ;
        
        // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
        Vector4fMapConst p_tgt = p.pc_tgt_->points[p.indices_tgt_[i]].getVector4fMap ();
        // get transformation from body to world
        trans_src = get_trans((*p.pc_src_)[p.indices_src_[i]], p.trans_lookup_src_, p.trans_src_4f_) ;    
        trans_tgt = get_trans((*p.pc_tgt_)[p.indices_tgt_[i]], p.trans_lookup_tgt_, p.trans_tgt_4f_) ;   
        
        Eigen::Vector4f p_global_src (trans_src * transformation_matrix * p_src);
        Eigen::Vector4f p_global_tgt (trans_tgt * transformation_matrix * p_tgt);
    
        // Estimate the distance (cost function)
        // The last coordiante is still guaranteed to be set to 1.0
        Eigen::Vector3d res(p_global_src[0] - p_global_tgt[0], p_global_src[1] - p_global_tgt[1], p_global_src[2] - p_global_tgt[2]);
        Eigen::Vector3d temp (p.mahalanobis_[i]* res);
        //increment= res'*temp/num_matches = temp'*M*temp/num_matches (we postpone 1/num_matches after the loop closes)
        f+= double(res.transpose() * temp);
      }
      cnt += m ;
    }
    double ave_error = f/cnt;
    printf("ave error %f \n", ave_error) ;
}

    template <typename PointSource, typename PointTarget> void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::snoop(const Vector6d& x, std::string solName)
{
    base_transformation_.setIdentity() ;
    Eigen::Matrix4f transformation_matrix = base_transformation_;
    applyState(transformation_matrix, x);
    
    /// have to update mahalanobis matrices according to current solution
    updateMahalanobis(transformation_matrix.topLeftCorner<3,3>().cast<double>()) ;

    /// get current average error
    double ave_error = functor_(x) ;

    std::array<int, 3> color_src{{255, 0, 0}} ;
    std::array<int, 3> color_tgt{{0, 255, 0}} ;
    std::array<int, 3> color_mid{{0, 0, 255}} ;
    double fi = 0;
    int cnt = 0 ;
    int cnt_outliers = 0 ; 
    for(const auto& p : pairs_)
    {

        int m = static_cast<int> (p.indices_src_.size ());
        //Eigen::Matrix4f trans_l2w_src = p.trans_src_4f_ * transformation_matrix ;  
        //Eigen::Matrix4f trans_l2w_tgt = p.trans_tgt_4f_ * transformation_matrix ;  
        
        
        pcl::PointCloud<PointSource> cloud_world_src, cloud_world_tgt ;
        transformTimedPointCloud(*p.pc_src_, 
                                 p.trans_lookup_src_,
                                 p.trans_src_4f_,
                                 transformation_matrix, 
                                 cloud_world_src) ;
        transformTimedPointCloud(*p.pc_tgt_, 
                                 p.trans_lookup_tgt_,
                                 p.trans_tgt_4f_,
                                 transformation_matrix, 
                                 cloud_world_tgt) ;

        //pcl::transformPointCloud(*p.pc_src_, cloud_world_src, trans_l2w_src) ;
        //pcl::transformPointCloud(*p.pc_tgt_, cloud_world_tgt, trans_l2w_tgt) ;
        Eigen::Matrix4f trans_src, trans_tgt ; 
        for (int i = 0; i < m; ++i)
        {
            printf(">>> pair id %d: point %d \n", cnt, i) ;
            printf("error %g ave error %g \n", fi, ave_error) ;
            // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
            const auto& pcl_start_src = cloud_world_src[p.indices_src_[i]] ;
            const auto& pcl_start_tgt = cloud_world_tgt[p.indices_tgt_[i]] ;
            Eigen::Vector4f p_global_src = pcl_start_src.getVector4fMap() ; 
            Eigen::Vector4f p_global_tgt = pcl_start_tgt.getVector4fMap() ;
            
            // The last coordiante is still guaranteed to be set to 1.0
            Eigen::Vector3d res(p_global_src[0] - p_global_tgt[0], p_global_src[1] - p_global_tgt[1], p_global_src[2] - p_global_tgt[2]);
            Eigen::Vector3d temp (p.mahalanobis_[i]* res);
            //increment= res'*temp/num_matches = temp'*M*temp/num_matches (we postpone 1/num_matches after the loop closes)
            fi =  double(res.transpose() * temp);
            if(fi > ave_error) 
            {
                printf(">>> pair id %d \n", cnt) ;
                printf("error %g ave error %g \n", fi, ave_error) ;
                std::cout << "point difference" << std::endl ;
                std::cout << res << std::endl ;
            
                boost::shared_ptr<had::PCLViewer> viewer(new had::PCLViewer(solName)) ;
                viewer->initialize() ;
                /// add cloud to viewer
                viewer->add_cloud_to_viewer<PointSource>(cloud_world_src.makeShared(), color_src, std::string("cloud_world_src")) ;
                viewer->add_cloud_to_viewer<PointTarget>(cloud_world_tgt.makeShared(), color_tgt, std::string("cloud_world_tgt")) ;
                /// add normals of current pair to viewer
                trans_src = get_trans((*p.pc_src_)[p.indices_src_[i]], p.trans_lookup_src_, p.trans_src_4f_) ;    
                trans_tgt = get_trans((*p.pc_tgt_)[p.indices_tgt_[i]], p.trans_lookup_tgt_, p.trans_tgt_4f_) ;   
                Eigen::Vector3f end_src = p_global_src.head<3>() + trans_src.template topLeftCorner<3,3>() * transformation_matrix.topLeftCorner<3,3>() * p.normal_src_[p.indices_src_[i]] ;
                Eigen::Vector3f end_tgt = p_global_tgt.head<3>() + trans_tgt.template topLeftCorner<3,3>() * transformation_matrix.topLeftCorner<3,3>() * p.normal_tgt_[p.indices_tgt_[i]] ;
                PointSource pcl_end_src ;
                pcl_end_src.x = end_src[0] ;
                pcl_end_src.y = end_src[1] ;
                pcl_end_src.z = end_src[2] ;
                PointTarget pcl_end_tgt ;
                pcl_end_tgt.x = end_tgt[0] ;
                pcl_end_tgt.y = end_tgt[1] ;
                pcl_end_tgt.z = end_tgt[2] ;
                char arrow_name[50] ;
                sprintf(arrow_name, "normal_src") ;
                viewer->add_arrow(pcl_end_src, pcl_start_src, color_src, arrow_name) ;
                sprintf(arrow_name, "normal_tgt") ;
                viewer->add_arrow(pcl_end_tgt, pcl_start_tgt, color_tgt, arrow_name) ;
                /// add mahalanobis matrix eigenvector to viewer 
                Eigen::JacobiSVD<Eigen::Matrix3d> svd(p.mahalanobis_[i], Eigen::ComputeFullU) ;
                Eigen::Matrix3d U = svd.matrixU() ;
                Eigen::Vector3f start_mid = 0.5*(p_global_src.head<3>()+p_global_tgt.head<3>()) ;
                Eigen::Vector3f end_mid = start_mid + U.col(0).cast<float>() ;
                PointSource pcl_start_mid ;
                pcl_start_mid.x = start_mid[0] ; 
                pcl_start_mid.y = start_mid[1] ; 
                pcl_start_mid.z = start_mid[2] ; 
                PointTarget pcl_end_mid ;
                pcl_end_mid.x = end_mid[0] ;
                pcl_end_mid.y = end_mid[1] ;
                pcl_end_mid.z = end_mid[2] ;
                sprintf(arrow_name, "normal_mid") ;
                viewer->add_arrow(pcl_end_mid, pcl_start_mid, color_mid, arrow_name) ;
                
                viewer->show() ;
                
                /// create viewer_lidar to see cloud in lidar frame
                boost::shared_ptr<had::PCLViewer> viewer_lidar(new had::PCLViewer(solName+" lidar")) ;
                viewer_lidar->initialize() ;
                /// add original cloud viewer
                viewer_lidar->add_cloud_to_viewer<PointSource>(p.pc_src_, color_src, std::string("cloud_lidar_src")) ;
                viewer_lidar->add_cloud_to_viewer<PointTarget>(p.pc_tgt_, color_tgt, std::string("cloud_lidar_tgt")) ;
                /// add normals of current pair to viewer
                const auto& pcl_start_src_lidar = p.pc_src_->points[p.indices_src_[i]] ;
                const auto& pcl_start_tgt_lidar = p.pc_tgt_->points[p.indices_tgt_[i]] ;
                const auto& cov_src_lidar = p.cov_src_[p.indices_src_[i]] ;
                const auto& cov_tgt_lidar = p.cov_tgt_[p.indices_tgt_[i]] ;
                Eigen::JacobiSVD<Eigen::Matrix3d> svd_src(cov_src_lidar, Eigen::ComputeFullU) ;
                Eigen::JacobiSVD<Eigen::Matrix3d> svd_tgt(cov_tgt_lidar, Eigen::ComputeFullU) ;
                Eigen::Matrix3d U_src = svd_src.matrixU() ;
                Eigen::Matrix3d U_tgt = svd_tgt.matrixU() ;
                Eigen::Vector3f end_src_lidar = pcl_start_src_lidar.getVector4fMap().template head<3>() + U_src.col(2).cast<float>() ;
                Eigen::Vector3f end_tgt_lidar = pcl_start_tgt_lidar.getVector4fMap().template head<3>() + U_tgt.col(2).cast<float>() ;
                std::cout << "source cloud normal " << std::endl ;
                std::cout << U_src.col(2) << std::endl ;
                std::cout << "target cloud normal " << std::endl ;
                std::cout << U_tgt.col(2) << std::endl ;
                PointSource pcl_end_src_lidar ;
                pcl_end_src_lidar.x = end_src_lidar[0] ;
                pcl_end_src_lidar.y = end_src_lidar[1] ;
                pcl_end_src_lidar.z = end_src_lidar[2] ;
                PointSource pcl_end_tgt_lidar ;
                pcl_end_tgt_lidar.x = end_tgt_lidar[0] ;
                pcl_end_tgt_lidar.y = end_tgt_lidar[1] ;
                pcl_end_tgt_lidar.z = end_tgt_lidar[2] ;
                sprintf(arrow_name, "normal_src_liar") ;
                viewer_lidar->add_arrow(pcl_end_src_lidar, pcl_start_src_lidar, color_src, arrow_name) ;
                sprintf(arrow_name, "normal_tgt_liar") ;
                viewer_lidar->add_arrow(pcl_end_tgt_lidar, pcl_start_tgt_lidar, color_tgt, arrow_name) ;
                
                viewer_lidar->show() ;

                ++cnt_outliers ;
            }
        }
        ++cnt ;
    }

    printf("total outliers %d \n", cnt_outliers) ;
}

////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline double
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::CalibOptimizationFunctorWithIndices::operator() (const Vector6d& x)
{
  Eigen::Matrix4f transformation_matrix = gicp_->base_transformation_;
  gicp_->applyState(transformation_matrix, x);
  double f = 0;
  int cnt = 0 ;
  for(const auto& p : gicp_->pairs_)
  {
    int m = static_cast<int> (p.indices_src_.size ());
    for (int i = 0; i < m; ++i)
    {
      // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
      Vector4fMapConst p_src = p.pc_src_->points[p.indices_src_[i]].getVector4fMap ();
      // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
      Vector4fMapConst p_tgt = p.pc_tgt_->points[p.indices_tgt_[i]].getVector4fMap ();
      Eigen::Matrix4f trans_src = gicp_->get_trans((*p.pc_src_)[p.indices_src_[i]], p.trans_lookup_src_, p.trans_src_4f_) ;    
      Eigen::Matrix4f trans_tgt = gicp_->get_trans((*p.pc_tgt_)[p.indices_tgt_[i]], p.trans_lookup_tgt_, p.trans_tgt_4f_) ;   
      Eigen::Vector4f p_global_src (trans_src * transformation_matrix * p_src);
      Eigen::Vector4f p_global_tgt (trans_tgt * transformation_matrix * p_tgt);

      // Estimate the distance (cost function)
      // The last coordiante is still guaranteed to be set to 1.0
      Eigen::Vector3d res(p_global_src[0] - p_global_tgt[0], p_global_src[1] - p_global_tgt[1], p_global_src[2] - p_global_tgt[2]);
      Eigen::Vector3d temp (p.mahalanobis_[i]* res);
      //increment= res'*temp/num_matches = temp'*M*temp/num_matches (we postpone 1/num_matches after the loop closes)
      f+= double(res.transpose() * temp);
    }
    cnt += m ;
  }
  return f/cnt;
}

////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::CalibOptimizationFunctorWithIndices::df (const Vector6d& x, Vector6d& g)
{
  Eigen::Matrix4f transformation_matrix = gicp_->base_transformation_;
  gicp_->applyState(transformation_matrix, x);
  //Zero out g
  g.setZero ();
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero ();
  Eigen::Matrix3d R_src = Eigen::Matrix3d::Zero ();
  Eigen::Matrix3d R_tgt = Eigen::Matrix3d::Zero ();
  int cnt = 0 ;
  for(const auto& p : gicp_->pairs_)
  {
    int m = static_cast<int> (p.indices_src_.size ());
    Eigen::Matrix3d R1, R0 ;
    Eigen::Matrix3d R_src_p = Eigen::Matrix3d::Zero ();
    Eigen::Matrix3d R_tgt_p = Eigen::Matrix3d::Zero ();
    for (int i = 0; i < m; ++i)
    {
      // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
      Vector4fMapConst p_src = p.pc_src_->points[p.indices_src_[i]].getVector4fMap ();
      // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
      Vector4fMapConst p_tgt = p.pc_tgt_->points[p.indices_tgt_[i]].getVector4fMap ();

      Eigen::Matrix4f trans_src = gicp_->get_trans((*p.pc_src_)[p.indices_src_[i]], p.trans_lookup_src_, p.trans_src_4f_) ;    
      Eigen::Matrix4f trans_tgt = gicp_->get_trans((*p.pc_tgt_)[p.indices_tgt_[i]], p.trans_lookup_tgt_, p.trans_tgt_4f_) ;   
      Eigen::Vector4f p_global_src (trans_src * transformation_matrix * p_src);
      Eigen::Vector4f p_global_tgt (trans_tgt * transformation_matrix * p_tgt) ;
      // The last coordiante is still guaranteed to be set to 1.0
      Eigen::Vector3d res (p_global_src[0] - p_global_tgt[0], p_global_src[1] - p_global_tgt[1], p_global_src[2] - p_global_tgt[2]);
      // temp = M*res
      Eigen::Vector3d temp (p.mahalanobis_[i] * res);
      // Increment translation gradient
      // g.head<3> ()+= 2*(R1-R0)'*M*res/num_matches (we postpone 2/num_matches after the loop closes)
      g.head<3> ()+= p.timed_rot_diff_[i].transpose() * temp;
      // Increment rotation gradient
      // R_src += 2*p_src3*(M*res)'*R1/num_matches
      Eigen::Vector4f pp = gicp_->base_transformation_ * p_src;
      Eigen::Vector3d p_src3 (pp[0], pp[1], pp[2]);
      R1 = gicp_->get_trans((*p.pc_src_)[p.indices_src_[i]], p.trans_lookup_src_, p.trans_src_4f_).template topLeftCorner<3,3>().template cast<double>() ;    
      R_src_p += p_src3 * temp.transpose() * R1;
      // R_tgt += 2*p_tgt3*(M*res)'*R0/num_matches
      pp = gicp_->base_transformation_ * p_tgt ;
      Eigen::Vector3d p_tgt3 (pp[0], pp[1], pp[2]) ;
      R0 = gicp_->get_trans((*p.pc_tgt_)[p.indices_tgt_[i]], p.trans_lookup_tgt_, p.trans_tgt_4f_).template topLeftCorner<3,3>().template cast<double>() ;    
      R_tgt_p += p_tgt3 * temp.transpose() * R0;
    }
    //R_src_p *= R1 ;
    //R_tgt_p *= R0 ;
    R_src += R_src_p ;
    R_tgt += R_tgt_p ;
    cnt += m ;
  }
  // R = R_src - R_tgt
  R_src *= 2.0/cnt ;
  R_tgt *= 2.0/cnt ;
  R = R_src - R_tgt ;
  g.head<3> ()*= 2.0/cnt;
  /// esimate xyz (z is not estimated as it needs changes in pitch and roll) or rpy once a time
  if(gicp_->estimate_xyz_)
      g.tail<4>().setZero() ;
  else
    g.head<3>().setZero() ;
  gicp_->computeRDerivative(x, R, g);
}

////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
had::CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::CalibOptimizationFunctorWithIndices::fdf (const Vector6d& x, double& f, Vector6d& g)
{
  Eigen::Matrix4f transformation_matrix = gicp_->base_transformation_;
  gicp_->applyState(transformation_matrix, x);
  //Zero out g
  f = 0;
  g.setZero ();
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero ();
  Eigen::Matrix3d R_src = Eigen::Matrix3d::Zero ();
  Eigen::Matrix3d R_tgt = Eigen::Matrix3d::Zero ();
  int cnt = 0 ;
  for(const auto& p : gicp_->pairs_)
  {
    int m = static_cast<int> (p.indices_src_.size ());
    Eigen::Matrix3d R1, R0 ;
    Eigen::Matrix3d R_src_p = Eigen::Matrix3d::Zero ();
    Eigen::Matrix3d R_tgt_p = Eigen::Matrix3d::Zero ();
    for (int i = 0; i < m; ++i)
    {
      // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
      Vector4fMapConst p_src = p.pc_src_->points[p.indices_src_[i]].getVector4fMap ();
      // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
      Vector4fMapConst p_tgt = p.pc_tgt_->points[p.indices_tgt_[i]].getVector4fMap ();

      Eigen::Matrix4f trans_src = gicp_->get_trans((*p.pc_src_)[p.indices_src_[i]], p.trans_lookup_src_, p.trans_src_4f_) ;    
      Eigen::Matrix4f trans_tgt = gicp_->get_trans((*p.pc_tgt_)[p.indices_tgt_[i]], p.trans_lookup_tgt_, p.trans_tgt_4f_) ;   
      Eigen::Vector4f p_global_src (trans_src * transformation_matrix * p_src);
      Eigen::Vector4f p_global_tgt (trans_tgt * transformation_matrix * p_tgt) ;
      // The last coordiante is still guaranteed to be set to 1.0
      Eigen::Vector3d res (p_global_src[0] - p_global_tgt[0], p_global_src[1] - p_global_tgt[1], p_global_src[2] - p_global_tgt[2]);
      // temp = M*res
      Eigen::Vector3d temp (p.mahalanobis_[i] * res);
      // Increment total error
      f+= double(res.transpose() * temp);
      // Increment translation gradient
      // g.head<3> ()+= 2*(R1-R0)'*M*res/num_matches (we postpone 2/num_matches after the loop closes)
      g.head<3> ()+= p.timed_rot_diff_[i].transpose() * temp;
      // Increment rotation gradient
      // R_src += 2*p_src3*(M*res)'*R1/num_matches
      Eigen::Vector4f pp = gicp_->base_transformation_ * p_src;
      Eigen::Vector3d p_src3 (pp[0], pp[1], pp[2]);
      R1 = gicp_->get_trans((*p.pc_src_)[p.indices_src_[i]], p.trans_lookup_src_, p.trans_src_4f_).template topLeftCorner<3,3>().template cast<double>() ;    
      R_src_p += p_src3 * temp.transpose() * R1;
      // R_tgt += 2*p_tgt3*(M*res)'*R0/num_matches
      pp = gicp_->base_transformation_ * p_tgt ;
      Eigen::Vector3d p_tgt3 (pp[0], pp[1], pp[2]) ;
      R0 = gicp_->get_trans((*p.pc_tgt_)[p.indices_tgt_[i]], p.trans_lookup_tgt_, p.trans_tgt_4f_).template topLeftCorner<3,3>().template cast<double>() ;    
      R_tgt_p += p_tgt3 * temp.transpose() * R0 ;
    }
    //R_src_p *= R1 ;
    //R_tgt_p *= R0 ;
    R_src += R_src_p ;
    R_tgt += R_tgt_p ;
    cnt += m ;
  }
  // R = R_src - R_tgt
  R_src *= 2.0/cnt ;
  R_tgt *= 2.0/cnt ;
  R = R_src - R_tgt ;
  g.head<3> ()*= 2.0/cnt;
  /// esimate xyz (z is not estimated as it needs changes in pitch and roll) or rpy once a time
  if(gicp_->estimate_xyz_)
      g.tail<4>().setZero() ;
  else
    g.head<3>().setZero() ;
  // f
  f/= double(cnt);
  gicp_->computeRDerivative(x, R, g);
}

#endif //_IMPL_HAD_CALIB_HPP_
