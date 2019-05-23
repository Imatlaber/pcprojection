/* Copyright (c) AutoNavi - All Rights Reserved 
 * Author: shuo <shuo@alibaba-inc.com>
 * Description: class to pass through a range of points
 */

#ifndef _LIDAR_FEATURE_ILLUTREE_HPP_
#define _LIDAR_FEATURE_ILLUTREE_HPP_

#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/pca.h>
#include "lidar_feature_base.hpp"
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/extract_indices.h>

template<typename T>
class LidarUnorgEliminateTree : public LidarFeatureBase<T>
{
	public:

	void extract(pcl::PointCloud<T>& pc,
                         std::vector<Eigen::Matrix3d> * covs = nullptr, 
                         std::vector<Eigen::Vector3d> * normals = nullptr, 
                         int arg=0) override
	{
		pcl::copyPointCloud(*elimimation(pc.makeShared()), pc) ;
	}

	typename pcl::PointCloud<T>::Ptr elimimation(typename pcl::PointCloud<T>::Ptr cloud)
	{
		float radius = 0.5f;
		typename pcl::KdTreeFLANN<T> kdtree;	
		kdtree.setInputCloud (cloud);
		pcl::IndicesPtr alloutliers (new std::vector<int>);
		typename pcl::PointCloud<T>::Ptr cloud_filtered(new pcl::PointCloud<T>) ;
		for(int indpt = 0; indpt < cloud->size(); ++ indpt)
		{
			T searchPoint = cloud->at(indpt);
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;		

			if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 3 )
			{
				typename pcl::PointCloud<T>::Ptr cloud_pca(new pcl::PointCloud<T>) ;
				cloud_pca->resize(pointIdxRadiusSearch.size ());
				for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
				{
					cloud_pca->points[i].x = cloud->points[ pointIdxRadiusSearch[i] ].x; 
					cloud_pca->points[i].y = cloud->points[ pointIdxRadiusSearch[i] ].y; 
					cloud_pca->points[i].z = cloud->points[ pointIdxRadiusSearch[i] ].z; 				
				}
						
				pcl::PCA<T> pca;
				pca.setInputCloud(cloud_pca);
				Eigen::Vector3f eigenvalues = pca.getEigenValues();
				Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
				//cout << eigenvectors << endl;
				//cout << eigenvalues << endl;
				Eigen::Vector3f V1 = eigenvectors.col(0);
				Eigen::Vector3f V2 = eigenvectors.col(1);
				Eigen::Vector3f V3 = eigenvectors.col(2);
			
				float threshold = 0.3f;
				if((eigenvalues(2) / eigenvalues(1) > threshold && eigenvalues(1) / eigenvalues(0) > threshold) || eigenvalues(2) > 0.3)
				{
					alloutliers->push_back(indpt);
				}
			}
		}
		
		typename pcl::ExtractIndices<T> filter;
		filter.setInputCloud (cloud);
		filter.setIndices (alloutliers);
		// Extract the points in cloud_in referenced by indices_in as a separate point cloud:
		filter.setNegative (true);
		filter.filter (*cloud_filtered);
		
		return cloud_filtered ;
	}
} ;



#endif
