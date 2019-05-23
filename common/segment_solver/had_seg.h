/* Copyright (c) AutoNavi - All Rights Reserved 
 * Author: shuo <shuo@alibaba-inc.com>
 * Description: customized segmentation class based on pcl
 */

#ifndef _HAD_SEG_H_
#define _HAD_SEG_H_

#include <string>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/sac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "model_plane.h"

namespace had
{

    enum HadSacModel
    {
        HAD_SACMODEL_PLANE,
        HAD_SACMODEL_NUM
    } ;

    enum HasSacMethod
    {
        HAD_SACMETHOD_RANSAC,
        HAD_SACMETHOD_NUM
    } ;

    template <typename PointT>
    class HadSegment : public pcl::SACSegmentation<PointT>
    {
        public:
        using pcl::PCLBase<PointT>::input_ ;
        using pcl::PCLBase<PointT>::indices_ ;
        using pcl::PCLBase<PointT>::fake_indices_ ;
        using pcl::SACSegmentation<PointT>::random_ ;
        using pcl::SACSegmentation<PointT>::model_ ;
        using pcl::SACSegmentation<PointT>::sac_ ;
        using pcl::SACSegmentation<PointT>::probability_ ;
        using pcl::SACSegmentation<PointT>::max_iterations_ ;
        using pcl::SACSegmentation<PointT>::samples_radius_ ;
        using pcl::SACSegmentation<PointT>::samples_radius_search_ ;
        using pcl::SACSegmentation<PointT>::threshold_ ;
        using pcl::SACSegmentation<PointT>::model_type_ ;
        using pcl::SACSegmentation<PointT>::method_type_ ;
        using pcl::SACSegmentation<PointT>::optimize_coefficients_ ;
        
        typedef typename pcl::SampleConsensusModel<PointT>::Ptr SampleConsensusModelPtr ;
        typedef typename pcl::SampleConsensus<PointT>::Ptr SampleConsensusPtr ;
        typedef typename HadSampleConsensusModelPlane<PointT>::Ptr HadSampleConsensusModelPlanePtr ;
       
        HadSegment(bool random = false):
            pcl::SACSegmentation<PointT>(random)
        {

        }

        virtual ~HadSegment() {}

        public:

        /// Base method for segmentation of a model in a PointCloud given by <setInputCloud (), setIndices ()>
        /// \param[out] inliers the resultant point indices that support the model found (inliers)
        /// \param[out] model_coefficients the resultant model coefficients
        virtual void segment (pcl::PointIndices &inliers, pcl::ModelCoefficients &model_coefficients) override ;

        /// Initialize the sample consensus model and set is parameters
        /// \param[in] model_type type of SAC model to be used
        virtual bool initSACModel(const int model_type) override ; 

        /// Initialize the sample consensus method and set its parameters 
        /// \param[in] method_type the type of SAC method to be used 
        virtual void initSAC (const int method_type) override ;
        
        /// get class name
        virtual std::string getClassName() const override
        {
            return std::string("HadSegment") ;
        }

        bool initCompute () ;

        bool deinitCompute () ;
    } ;















}


#include "had_seg.hpp"

#endif // _HAD_SEG_H_
