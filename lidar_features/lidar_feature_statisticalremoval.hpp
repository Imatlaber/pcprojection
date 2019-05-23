/* Copyright (c) AutoNavi - All Rights Reserved 
 * Author: shuo <shuo@alibaba-inc.com>
 * Description: class to pass through a range of points
 */

#ifndef _LIDAR_FEATURE_STATISTICALREMOVAL_HPP_
#define _LIDAR_FEATURE_STATISTICALREMOVAL_HPP_

#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "lidar_feature_base.hpp"

template<typename T>
class LidarUnorgStatisticalRemoval : public LidarFeatureBase<T>
{
	public:

	void extract(pcl::PointCloud<T>& pc, 
                         std::vector<Eigen::Matrix3d> * covs = nullptr, 
                         std::vector<Eigen::Vector3d> * normals = nullptr, 
                         int arg=0) override
	{
		pcl::copyPointCloud(*statistical_removal(pc.makeShared()), pc) ;
	}

	typename pcl::PointCloud<T>::Ptr statistical_removal(typename pcl::PointCloud<T>::Ptr cloud)
	{		
		typename pcl::PointCloud<T>::Ptr cloud_filtered(new pcl::PointCloud<T>) ;
		cloud_filtered->header.stamp = cloud->header.stamp;
		for(int i = 0; i < cloud->size(); ++i)
		{
			if(cloud->at(i).x != 0 && cloud->at(i).y != 0 && cloud->at(i).z != 0)
			{
				cloud_filtered->push_back(cloud->at(i));
			}
		}
		pcl::RadiusOutlierRemoval<T> outrem;
		// build the filter
		outrem.setInputCloud(cloud_filtered);
		outrem.setRadiusSearch(0.5);
		outrem.setMinNeighborsInRadius (5);
		// apply filter
		outrem.filter (*cloud_filtered);
    
// 		pcl::StatisticalOutlierRemoval<TYPE> sor;
// 		sor.setInputCloud (cloud_filtered);
// 		sor.setMeanK (50);
// 		sor.setStddevMulThresh (3.0);
// 		sor.filter (*cloud_filtered);
		
		return cloud_filtered;
	}
} ;



#endif
