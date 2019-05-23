/* Copyright (c) EcarX - All Rights Reserved
 * Author: Ke Ma <ke.ma@ecarx.com.cn>
 * description: base class for KITTI Velodyne cloudpoints projection on images
 */
#ifndef _DATAVELCAMFUSIONBASE_HPP_
#define _DATAVELCAMFUSIONBASE_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

class DataVelCamFusionBase
{
public:
	DataVelCamFusionBase() {}

	virtual bool isPointInBox(pcl::PointXYZI &point_3d,
							  double boxForward,
							  double boxHorizontal,
							  double boxVertical,
							  double lasIntensity) = 0;

	virtual bool readPointCloud(std::vector<pcl::PointXYZI> &vPts3d,
								const std::string &veloBinPath) = 0;

	virtual bool transVelo2Cam(std::vector<pcl::PointXYZI> &vPts3d,
							   std::vector<cv::Point3f> &vPts2d) = 0;

	virtual bool calPixelIntensity(const std::vector<cv::Point3f> &vPtsCamXYI,
								   const std::string &imgPath,
								   cv::Mat &imProjection) = 0;

	virtual bool plotImage(cv::Mat &imageProjection) = 0;

	virtual bool saveImage(bool isSaveImgOut,
						   cv::Mat &imageProjection,
						   const std::string &imgFilePath) = 0;

	virtual bool exec() = 0;
};

#endif // _DATAVELCAMFUSIONBASE_HPP_