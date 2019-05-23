/* Copyright (c) AutoNavi - All Rights Reserved 
 * Author: shuo <shuo@alibaba-inc.com>
 * Description: customized segmentation class implementation based on pcl
 */

#ifndef _HAD_SEG_HPP_
#define _HAD_SEG_HPP_

#include "had_seg.h"
#include "model_plane.h"
#include "had_ransac.h"

template <typename PointT> void
had::HadSegment<PointT>::segment (pcl::PointIndices &inliers, pcl::ModelCoefficients &model_coefficients)
{
  // Copy the header information
  inliers.header = model_coefficients.header = input_->header;

  if (!initCompute ()) 
  {
    inliers.indices.clear (); model_coefficients.values.clear ();
    return;
  }

  // Initialize the Sample Consensus model and set its parameters
  if (!initSACModel (model_type_))
  {
    PCL_ERROR ("[pcl::%s::segment] Error initializing the SAC model!\n", getClassName ().c_str ());
    deinitCompute ();
    inliers.indices.clear (); model_coefficients.values.clear ();
    return;
  }
  // Initialize the Sample Consensus method and set its parameters
  initSAC (method_type_);

  if (!sac_->computeModel (0))
  {
    PCL_ERROR ("[pcl::%s::segment] Error segmenting the model! No solution found.\n", getClassName ().c_str ());
    deinitCompute ();
    inliers.indices.clear (); model_coefficients.values.clear ();
    return;
  }

  // Get the model inliers
  sac_->getInliers (inliers.indices);

  // Get the model coefficients
  Eigen::VectorXf coeff;
  sac_->getModelCoefficients (coeff);

  // If the user needs optimized coefficients
  if (optimize_coefficients_)
  {
    Eigen::VectorXf coeff_refined;
    model_->optimizeModelCoefficients (inliers.indices, coeff, coeff_refined);
    model_coefficients.values.resize (coeff_refined.size ());
    memcpy (&model_coefficients.values[0], &coeff_refined[0], coeff_refined.size () * sizeof (float));
    // Refine inliers
    std::vector<int> selection ;
    sac_->getModel(selection) ;
    HadSampleConsensusModelPlanePtr had_plane = boost::dynamic_pointer_cast<HadSampleConsensusModelPlane<PointT>>(model_) ;
    had_plane->selectWithinDistanceAndRadius (coeff_refined, threshold_, selection, inliers.indices);
  }
  else
  {
    model_coefficients.values.resize (coeff.size ());
    memcpy (&model_coefficients.values[0], &coeff[0], coeff.size () * sizeof (float));
  }

  deinitCompute ();
}

template<typename PointT>
bool had::HadSegment<PointT>::initSACModel(const int model_type)
{
    switch (model_type)
    {
        case had::HAD_SACMODEL_PLANE:
        {
            printf("using HAD_SACMODEL_PLANE model \n") ;
            model_ = SampleConsensusModelPtr(new had::HadSampleConsensusModelPlane<PointT>(input_, *indices_, random_)) ;
            break ;
        }
    }

    return true ;
}

template <typename PointT> 
void had::HadSegment<PointT>::initSAC (const int method_type)
{
    switch (method_type)
    {
        case had::HAD_SACMETHOD_RANSAC:
        default:
        {
            printf("using HAD_SACMETHOD_RANSAC method \n") ;
            sac_ = SampleConsensusPtr(new had::HadRandomSampleConsensus<PointT>(model_, threshold_)) ;
            break ;
        }

    }

    // Set the Sample Consensus parameters if they are given/changed
    if (sac_->getProbability () != probability_)
    {
      PCL_DEBUG ("[pcl::%s::initSAC] Setting the desired probability to %f\n", getClassName ().c_str (), probability_);
      sac_->setProbability (probability_);
    }
    if (max_iterations_ != -1 && sac_->getMaxIterations () != max_iterations_)
    {
      PCL_DEBUG ("[pcl::%s::initSAC] Setting the maximum number of iterations to %d\n", getClassName ().c_str (), max_iterations_);
      sac_->setMaxIterations (max_iterations_);
    }
    if (samples_radius_ > 0.)
    {
      PCL_DEBUG ("[pcl::%s::initSAC] Setting the maximum sample radius to %f\n", getClassName ().c_str (), samples_radius_);
      // Set maximum distance for radius search during random sampling
      model_->setSamplesMaxDist (samples_radius_, samples_radius_search_);
    }

}

template <typename PointT> bool
had::HadSegment<PointT>::initCompute ()
{
  // Check if input was set
  if (!input_)
    return (false);

  // If no point indices have been given, construct a set of indices for the entire input point cloud
  if (!indices_)
  {
    fake_indices_ = true;
    indices_.reset (new std::vector<int>);
    try
    {
      indices_->resize (input_->points.size ());
    }
    catch (const std::bad_alloc&)
    {
      PCL_ERROR ("[initCompute] Failed to allocate %lu indices.\n", input_->points.size ());
    }
    for (size_t i = 0; i < indices_->size (); ++i) { (*indices_)[i] = static_cast<int>(i); }
  }

  // If we have a set of fake indices, but they do not match the number of points in the cloud, update them
  if (fake_indices_ && indices_->size () != input_->points.size ())
  {
    size_t indices_size = indices_->size ();
    indices_->resize (input_->points.size ());
    for (size_t i = indices_size; i < indices_->size (); ++i) { (*indices_)[i] = static_cast<int>(i); }
  }

  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
had::HadSegment<PointT>::deinitCompute ()
{
  return (true);
}


#endif // _HAD_SEG_HPP_
