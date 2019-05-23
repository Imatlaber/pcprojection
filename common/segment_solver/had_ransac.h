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

#ifndef _HAD_RANSAC_H_
#define _HAD_RANSAC_H_

#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/sac_model.h>
#include "model_plane.h"

namespace had
{
  /** \brief @b HadRandomSampleConsensus represents an implementation of the RANSAC (RAndom SAmple Consensus) algorithm, as 
    * described in: "Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and 
    * Automated Cartography", Martin A. Fischler and Robert C. Bolles, Comm. Of the ACM 24: 381–395, June 1981.
    * \author Radu B. Rusu
    * \ingroup sample_consensus
    */
  template <typename PointT>
  class HadRandomSampleConsensus : public pcl::SampleConsensus<PointT>
  {
    typedef typename pcl::SampleConsensusModel<PointT>::Ptr SampleConsensusModelPtr;
    typedef typename HadSampleConsensusModelPlane<PointT>::Ptr HadSampleConsensusModelPlanePtr ;

    public:
      typedef boost::shared_ptr<HadRandomSampleConsensus> Ptr;
      typedef boost::shared_ptr<const HadRandomSampleConsensus> ConstPtr;
      
      using pcl::SampleConsensus<PointT>::max_iterations_;
      using pcl::SampleConsensus<PointT>::threshold_;
      using pcl::SampleConsensus<PointT>::iterations_;
      using pcl::SampleConsensus<PointT>::sac_model_;
      using pcl::SampleConsensus<PointT>::model_;
      using pcl::SampleConsensus<PointT>::model_coefficients_;
      using pcl::SampleConsensus<PointT>::inliers_;
      using pcl::SampleConsensus<PointT>::probability_;

      /** \brief RANSAC (RAndom SAmple Consensus) main constructor
        * \param[in] model a Sample Consensus model
        */
      HadRandomSampleConsensus (const SampleConsensusModelPtr &model) 
        : pcl::SampleConsensus<PointT> (model)
      {
        // Maximum number of trials before we give up.
        max_iterations_ = 10000;
      }

      /** \brief RANSAC (RAndom SAmple Consensus) main constructor
        * \param[in] model a Sample Consensus model
        * \param[in] threshold distance to model threshold
        */
      HadRandomSampleConsensus (const SampleConsensusModelPtr &model, double threshold) 
        : pcl::SampleConsensus<PointT> (model, threshold)
      {
        // Maximum number of trials before we give up.
        max_iterations_ = 10000;
      }

      /** \brief Compute the actual model and find the inliers
        * \param[in] debug_verbosity_level enable/disable on-screen debug information and set the verbosity level
        */
      bool 
      computeModel (int debug_verbosity_level = 0) override ;
  };
}

#include "had_ransac.hpp"

#endif  //#ifndef _HAD_RANSAC_H_

