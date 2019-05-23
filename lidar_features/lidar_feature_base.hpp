/* Copyright (c) AutoNavi - All Rights Reserved 
 * Author: shuo <shuo@alibaba-inc.com>
 * Description: base lidar feature class
 */

#ifndef _LIDAR_FEATURE_BASE_HPP_
#define _LIDAR_FEATURE_BASE_HPP_

#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <lcm/lcm-cpp.hpp>
#include <Eigen/Core>
#include "../../utilities/hadif/param.hpp"
#include "../../utilities/hadif/trans.hpp"

template<typename T>
class LidarFeatureBase
{
	public:
    typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerPtr ;
   
    LidarFeatureBase():
        standalone_(true)
    {
        lcm_ = new lcm::LCM ;
        param_ = new param_t(lcm_) ;
        trans_ = new trans_t(lcm_->getUnderlyingLCM(), param_) ;
    }

    LidarFeatureBase(lcm::LCM * lcm, 
                     trans_t * trans,
                     std::string ns ):
        lcm_(lcm),
        trans_(trans),
        standalone_(false),
        namespace_(ns)
    {
        param_ = new param_t(lcm_) ;
    }

    virtual ~LidarFeatureBase()
    {
        if(!standalone_)
            return ;
        if(trans_)
            delete trans_ ;
        if(param_)
            delete param_ ;
        if(lcm_)
            delete lcm_ ;
    }
   
	virtual void extract(const typename pcl::PointCloud<T>& pc, std::vector<int>& inliers, int arg=0) {} ;	
	
	virtual void extract(pcl::PointCloud<T>& pc, 
                         std::vector<Eigen::Matrix3d> * covs = nullptr, 
                         std::vector<Eigen::Vector3d> * normals = nullptr, 
                         int arg=0)  {}

    public:

    lcm::LCM * lcm_ ;
    param_t * param_ ;
    trans_t * trans_ ;
    bool standalone_ ;
    std::string namespace_ ;  
} ;













#endif 
