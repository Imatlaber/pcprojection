/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef _HAD_RANSAC_HPP_
#define _HAD_RANSAC_HPP_

#include <boost/pointer_cast.hpp>
#include "had_ransac.h"

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
had::HadRandomSampleConsensus<PointT>::computeModel (int)
{

  HadSampleConsensusModelPlanePtr had_plane = boost::dynamic_pointer_cast<HadSampleConsensusModelPlane<PointT>>(sac_model_) ;

  // Warn and exit if no threshold was set
  if (threshold_ == std::numeric_limits<double>::max())
  {
    PCL_ERROR ("[pcl::HadRandomSampleConsensus::computeModel] No threshold set!\n");
    return (false);
  }

  iterations_ = 0;
  int n_best_inliers_count = -INT_MAX;
  double k = 1.0;

  std::vector<int> selection;
  Eigen::VectorXf model_coefficients;

  double log_probability  = log (1.0 - probability_);
  double one_over_indices = 1.0 / static_cast<double> (had_plane->getIndices ()->size ());

  int n_inliers_count = 0;
  unsigned skipped_count = 0;
  // supress infinite loops by just allowing 10 x maximum allowed iterations for invalid model parameters!
  const unsigned max_skip = max_iterations_ * 10;
  
  // Iterate
  while (iterations_ < k && skipped_count < max_skip)
  {
    // Get X samples which satisfy the model criteria
    had_plane->getSamples (iterations_, selection);

    if (selection.empty ()) 
    {
      PCL_ERROR ("[pcl::HadRandomSampleConsensus::computeModel] No samples could be selected!\n");
      break;
    }

    
    // Search for inliers in the point cloud for the current plane model M
    if (!had_plane->computeModelCoefficients (selection, model_coefficients))
    {
      //++iterations_;
      ++skipped_count;
      continue;
    }

    // Select the inliers that are within threshold_ from the model
    //sac_model_->selectWithinDistance (model_coefficients, threshold_, inliers);
    //if (inliers.empty () && k > 1.0)
    //  continue;

    n_inliers_count = had_plane->countWithinDistanceAndRadius (model_coefficients, threshold_, selection);

    // Better match ?
    if (n_inliers_count > n_best_inliers_count)
    {
      n_best_inliers_count = n_inliers_count;

      // Save the current model/inlier/coefficients selection as being the best so far
      model_              = selection;
      model_coefficients_ = model_coefficients;

      // Compute the k parameter (k=log(z)/log(1-w^n))
      double w = static_cast<double> (n_best_inliers_count) * one_over_indices;
      double p_no_outliers = 1.0 - pow (w, static_cast<double> (selection.size ()));
      p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon (), p_no_outliers);       // Avoid division by -Inf
      p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
      k = log_probability / log (p_no_outliers);
    }

    ++iterations_;
    PCL_DEBUG ("[pcl::HadRandomSampleConsensus::computeModel] Trial %d out of %f: %d inliers (best is: %d so far).\n", iterations_, k, n_inliers_count, n_best_inliers_count);
    if (iterations_ > max_iterations_)
    {
      PCL_DEBUG ("[pcl::HadRandomSampleConsensus::computeModel] RANSAC reached the maximum number of trials.\n");
      break;
    }
  }

  PCL_DEBUG ("[pcl::HadRandomSampleConsensus::computeModel] Model: %lu size, %d inliers.\n", model_.size (), n_best_inliers_count);

  if (model_.empty ())
  {
    inliers_.clear ();
    return (false);
  }

  // Get the set of inliers that correspond to the best model found so far
  had_plane->selectWithinDistanceAndRadius (model_coefficients_, threshold_, model_, inliers_);
  return (true);
}

#define PCL_INSTANTIATE_HadRandomSampleConsensus(T) template class PCL_EXPORTS pcl::HadRandomSampleConsensus<T>;

#endif    // _HAD_RANSAC_HPP_

