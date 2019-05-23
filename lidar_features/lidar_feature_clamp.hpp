/* Copyright (c) AutoNavi - All Rights Reserved 
 * Author: shuo <shuo@alibaba-inc.com>
 * Description: class to pass through a range of points
 */

#ifndef _LIDAR_FEATURE_CLAMP_HPP_
#define _LIDAR_FEATURE_CLAMP_HPP_

#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/filters/passthrough.h>
#include "lidar_feature_base.hpp"

template<typename T>
class LidarOrgClamp : public LidarFeatureBase<T>
{
	public:

	void extract(pcl::PointCloud<T>& pc, 
                         std::vector<Eigen::Matrix3d> * covs = nullptr, 
                         std::vector<Eigen::Vector3d> * normals = nullptr, 
                         int arg=0) override
	{
		pcl::copyPointCloud(*filter_by_field(pc.makeShared(), "c", 0, 2, true), pc) ;
		pcl::copyPointCloud(*filter_by_field(pc.makeShared(), "c", 0, 60, false), pc) ;
	}

	typename pcl::PointCloud<T>::Ptr filter_by_field(typename pcl::PointCloud<T>::Ptr cloud, std::string field_name, float min, float max, bool FilterLimitsNegative)
	{
		typename pcl::PointCloud<T>::Ptr cloud_filtered(new pcl::PointCloud<T>) ;
		pcl::copyPointCloud(*cloud, *cloud_filtered);	
		
		if(field_name == "c")
		{
			for(int i = 0; i < cloud->size(); ++ i)
			{
				pcl::PointXYZ tmppoint;
				tmppoint.x = cloud->at(i).x;
				tmppoint.y = cloud->at(i).y;
				tmppoint.z = cloud->at(i).z;
				if(!FilterLimitsNegative)
					if(tmppoint.getVector3fMap().norm() < min || tmppoint.getVector3fMap().norm() > max)
					{
						cloud_filtered->at(i).x = 0;
						cloud_filtered->at(i).y = 0;
						cloud_filtered->at(i).z = 0;
					}
				if(FilterLimitsNegative)
					if(tmppoint.getVector3fMap().norm() > min && tmppoint.getVector3fMap().norm() < max)
					{
						cloud_filtered->at(i).x = 0;
						cloud_filtered->at(i).y = 0;
						cloud_filtered->at(i).z = 0;
					}	
			}
		}
		
		return cloud_filtered ;
	}
} ;



#endif
