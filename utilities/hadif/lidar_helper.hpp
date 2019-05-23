/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: lidar helper
 */

#ifndef LIDAR_HELPER_H
#define LIDAR_HELPER_H

#include <Eigen/Sparse>
#include <map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include "../../utilities/drivers/velodyne/pcl_point_types.h"

template <class T>
class LidarHelper 
{
public:
    //Split pointclouds in a specified 2D square grid and perform voxel downsampling
    LidarHelper(double res, double grid_size);
    
	static pcl::PointCloud<T> getDownsamplePointCloud(typename pcl::PointCloud<T>::Ptr pc_ptr, double res);

    void insertPoints(pcl::PointCloud<T> &pc);

    void getAllPoints(pcl::PointCloud<T> &all_pc);

	Eigen::SparseMatrix<uint16_t> getSparseMat(const pcl::PointCloud<T> &pcl_pt, double res, double canvas_size=100.);
	
	pcl::PointCloud<pcl::PointXYZ> getPointInGrid(const pcl::PointCloud<T> &pcl_pt, double res, double canvas_size=100.);
	
	//using -100m:100m by default as our drawing canvas for both dimensions
	int countPointsInGrid(const pcl::PointCloud<T> &pcl_pt, double res, double canvas_size=100.);

	template <typename T1, typename T2>
	cv::Mat getImg(const pcl::PointCloud<T1> &pc, T2 value, double res, double canvas, int CV_TYPE) ;

public:
	double downsample_res_, grid_size_; // in meter
    std::map<int, typename pcl::PointCloud<T>::Ptr> smaller_pc_segments_;

};



#endif // LIDAR_HELPER_H

