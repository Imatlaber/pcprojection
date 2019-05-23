/* Copyright (c) EcarX - All Rights Reserved
 * Author: Ke Ma <ke.ma@ecarx.com.cn>
 * description: KITTI Velodyne cloudpoints projection on images
 */
#ifndef _KITTIVELCAMFUSION_HPP_
#define _KITTIVELCAMFUSION_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <ctime>
#include <time.h>
#include <math.h>
#include "utilities/hadif/reader_helper.hpp"
#include "utilities/hadif/file_helper.hpp"
#include "utilities/hadif/color_helper.h"
#include "utilities/hadif/math_helper.hpp"
#include "dataVelCamFusionBase.hpp"

class KittiVelCamFusion : public DataVelCamFusionBase
{
public:
	KittiVelCamFusion(std::string recordDir,
					  std::string imgOutDir,
					  int max_scans)
		: recordDir_(recordDir),
		  max_scans_(max_scans),
		  imgOutDir_(imgOutDir) {}

	KittiVelCamFusion()
	{
		std::string recordDir = "/media/mk11/Dataset/KITTI_all/Raw/2011_09_26_drive_0009";
		std::string imgOutDir = "/home/mk11/compileWS/pclprojection/ImageOut/";
		int max_scans = 100;

		KittiVelCamFusion(recordDir, imgOutDir, max_scans);
	}

	// Filter out point cloud that is outside the view box, e.g., if a Velodyne
	// point is too far (low reflection intensity) from the Velodyne mounting base,
	// it will be filtered out.
	// pcl::PointXYZI pt3d: 	an input 3D point containing [x, y, z, I];
	// double boxForward: 		viewable distance along velodyne x-axis;
	// double boxHorizontal: 	viewable distance along velodyne y-axis;
	// double boxVertical: 		viewable distance along velodyne z-axis;
	// double lasIntensity: 	viewable reflection laser intensity;
	bool isPointInBox(pcl::PointXYZI &pt3d,
					  double boxForward,
					  double boxHorizontal,
					  double boxVertical,
					  double lasIntensity) override
	{
		if (pt3d.x < 0 || pt3d.x > boxForward)
		{
			return false;
		}
		else if (abs(pt3d.y) > boxHorizontal / 2)
		{
			return false;
		}
		else if (abs(pt3d.z) > boxVertical / 2)
		{
			return false;
		}
		else if (pt3d.intensity < lasIntensity)
		{
			return false;
		}

		return true;
	}

	// Read Velodyne point cloud data from binary file.
	bool readPointCloud(std::vector<pcl::PointXYZI> &vPts3d,
						const std::string &veloBinPath) override
	{
		// allocate 4 MB buffer (only ~130*4*4 KB are needed)
		int32_t num = 1000000;
		float *data = (float *)malloc(num * sizeof(float));

		// pointers
		float *px = data + 0;
		float *py = data + 1;
		float *pz = data + 2;
		float *pr = data + 3;

		// load point cloud
		FILE *stream;
		stream = fopen(veloBinPath.c_str(), "rb");
		num = fread(data, sizeof(float), num, stream) / 4;

		double boxForward = 50;
		double boxHorizontal = 200;
		double boxVertical = 20;
		double LasIntensity = 0.015;

		for (int32_t i = 0; i < num; i++)
		{
			pcl::PointXYZI point_3d;
			point_3d.x = *px;
			point_3d.y = *py;
			point_3d.z = *pz;
			point_3d.intensity = *pr;

			if (isPointInBox(point_3d, boxForward, boxHorizontal, boxVertical, LasIntensity))
			{
				vPts3d.push_back(point_3d);
			}

			px += 4;
			py += 4;
			pz += 4;
			pr += 4;
		}

		fclose(stream);
		free(data);

		return true;
	}

	// Transform points from velodyne coordinate to camera corrected frame.
	// Transformation matrices are available in KITTI raw data files, located in:
	// "~/KITTI_all/Raw/2011_09_26_drive_0009/2011_09_26/calib_cam_to_cam.txt"
	// "~/KITTI_all/Raw/2011_09_26_drive_0009/2011_09_26/calib_cam_to_cam.txt"
	// "~/KITTI_all/Raw/2011_09_26_drive_0009/2011_09_26/calib_velo_to_cam.txt"
	bool transVelo2Cam(std::vector<pcl::PointXYZI> &vPts3d,
					   std::vector<cv::Point3f> &vPts2d) override
	{
		// "/media/mk11/Dataset/KITTI_all/Raw/2011_09_26_drive_0009/2011_09_26/calib_cam_to_cam.txt"
		cv::Mat P_rect = (cv::Mat_<double>(3, 4) << 7.215377e+02, 0.000000e+00, 6.095593e+02, 0.000000e+00,
						  0.000000e+00, 7.215377e+02, 1.728540e+02, 0.000000e+00,
						  0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00);

		// "/media/mk11/Dataset/KITTI_all/Raw/2011_09_26_drive_0009/2011_09_26/calib_cam_to_cam.txt"
		cv::Mat R_rect = (cv::Mat_<double>(4, 4) << 9.999239e-01, 9.837760e-03, -7.445048e-03, 0.000000e+00,
						  -9.869795e-03, 9.999421e-01, -4.278459e-03, 0.000000e+00,
						  7.402527e-03, 4.351614e-03, 9.999631e-01, 0.000000e+00,
						  0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00);

		// "/media/mk11/Dataset/KITTI_all/Raw/2011_09_26_drive_0009/2011_09_26/calib_velo_to_cam.txt"
		cv::Mat Tr_vel2cam = (cv::Mat_<double>(4, 4) << 7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
							  1.480249e-02, 7.280733e-04, -9.998902e-01, -7.631618e-02,
							  9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01,
							  0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00);

		vPts2d.clear();

		for (size_t i = 0; i < vPts3d.size(); ++i)
		{
			cv::Mat px = (cv::Mat_<double>(3, 1) << 0, 0, 1);

			cv::Mat X = (cv::Mat_<double>(4, 1) << vPts3d.at(i).x, vPts3d.at(i).y, vPts3d.at(i).z, 1);

			px = P_rect * R_rect * Tr_vel2cam * X;
			px = px / px.at<double>(2, 0);

			cv::Point3f pt(px.at<double>(0, 0), px.at<double>(1, 0), vPts3d.at(i).intensity);

			vPts2d.push_back(pt);
		}

		return true;
	}

	// Project point cloud in camera coordinate to image frame,
	// i.e., camera coordinate (x, y) -> pixel (u, v).
	bool calPixelIntensity(const std::vector<cv::Point3f> &vPtsCamXYI,
						   const std::string &imgPath,
						   cv::Mat &imProjection) override
	{
		imProjection = cv::imread(imgPath.c_str());

		for (size_t i = 0; i < vPtsCamXYI.size(); ++i)
		{
			cv::Point3f pt_2d = vPtsCamXYI[i];
			if (pt_2d.x < 0 || pt_2d.x > imProjection.cols)
			{
				continue;
			}
			else if (pt_2d.y < 0 || pt_2d.y > imProjection.rows)
			{
				continue;
			}
			else
			{
				cv::Mat colourBar = (cv::Mat_<double>(10, 3) << 0, 0, 0.6667,
									 0, 0, 1.0000,
									 0, 0.3333, 1.0000,
									 0, 0.6667, 1.0000,
									 0, 1.0000, 1.0000,
									 0.3333, 1.0000, 0.6667,
									 0.6667, 1.0000, 0.3333,
									 1.0000, 1.0000, 0,
									 1.0000, 0.6667, 0,
									 1.0000, 0.3333, 0);

				int idxColour = round(pt_2d.z * 10);

				cv::Point2f pCentre(pt_2d.x, pt_2d.y);
				int radius = 2;
				int colourR = colourBar.at<double>(idxColour, 2) * 255;
				int colourG = colourBar.at<double>(idxColour, 1) * 255;
				int colourB = colourBar.at<double>(idxColour, 0) * 255;
				const cv::Scalar &color = cv::Scalar(colourR, colourG, colourB);
				int thickness = 5;

				cv::circle(imProjection, pCentre, radius, color);
			}
		}

		return true;
	}

	// Plot image
	bool plotImage(cv::Mat &imageProjection) override
	{
		cv::imshow("project image", imageProjection);

		cv::moveWindow("project image", 400, 250);
		cv::waitKey(1);

		return true;
	}

	// Save image to file
	bool saveImage(bool isSaveImgOut,
				   cv::Mat &imageProjection,
				   const std::string &imgOutDir) override
	{
		if (!isSaveImgOut)
			return true;

		std::string filePath(imgOutDir);
		std::string fileName("IM_OUT_");

		time_t now = time(0);
		struct tm tstruct;
		char buf[80];
		tstruct = *localtime(&now);

		strftime(buf, sizeof(buf), "%Y%m%d%H%M%S", &tstruct);
		std::string mytime(buf);
		std::string fileExt(".jpg");

		std::string imOutPath = "" + filePath + fileName + mytime + fileExt;

		if (!cv::imwrite(imOutPath, imageProjection))
		{
			std::cerr << "\t mk11-error: save image output not sucessful" << std::endl;
			return false;
		}
		else
		{
			// std::cerr << "\t Image Results Saved: " + imOutPath << std::endl;
		}

		return true;
	}

	// Execute the program
	bool exec() override
	{
		std::string dataDir = recordDir_ + "/2011_09_26/2011_09_26_drive_0009_sync";
		std::string fileDir_velo = dataDir + "/velodyne_points/data/";
		std::string fileDir_img = dataDir + "/image_00/data/";

		std::map<int, std::string> pc_files = had::File::getFileNames(fileDir_velo, ".bin", 10, 6, false);
		std::map<int, std::string> im_files = had::File::getFileNames(fileDir_img, ".png", 10, 6, false);

		int scan_index = 0;
		auto pim = im_files.begin();
		for (auto ppc = pc_files.begin(); ppc != pc_files.end(); ppc++, pim++)
		{
			std::string filePath_velo = ppc->second;
			std::string filePath_img = pim->second;

			std::vector<pcl::PointXYZI> pts_velo_XYZI;

			if (!readPointCloud(pts_velo_XYZI, filePath_velo))
			{
				std::cerr << "\t mk11-error: loading Velodyne binary not sucessful." << std::endl;
				return false;
			}

			std::vector<cv::Point3f> vPts2d;

			if (!transVelo2Cam(pts_velo_XYZI, vPts2d))
			{
				std::cerr << "\t mk11-error: error in display." << std::endl;
				return false;
			}

			cv::Mat imProj;

			if (!calPixelIntensity(vPts2d, filePath_img, imProj))
			{
				std::cerr << "\t mk11-error: error in pixel calculation." << std::endl;
				return false;
			}

			if (!plotImage(imProj))
			{
				std::cerr << "\t mk11-error: error in image ploting." << std::endl;
				return false;
			}

			if (!saveImage(false, imProj, imgOutDir_))
			{
				std::cerr << "\t mk11-error: error in saving image." << std::endl;
				return false;
			}

			++scan_index;
			if (max_scans_ > 0 && scan_index > max_scans_)
				break;
		}

		return true;
	}

private:
	std::string recordDir_;
	std::string imgOutDir_;
	int max_scans_;
};

#endif // _KITTIVELCAMFUSION_HPP_




// #include "kittiVelCamFusion.hpp"
// #include "kittiOXTS.hpp"
// #include <iostream>

// int main(int argc, char **argv)
// {

// 	std::string recDir = "/media/mk11/Dataset/KITTI_all/Raw/2011_09_26_drive_0009";
// 	std::string imgOutDir = "/home/mk11/compileWS/pclprojection/ImageOut/";

// 	DataVelCamFusionBase *velCamFusion_ = new KittiVelCamFusion(recDir, imgOutDir, 2000);

// 	if (!velCamFusion_->exec())
// 	{
// 		std::cerr << "\t mk11-error: unexpected finish." << std::endl;
// 	}

// 	delete velCamFusion_;


// 	return 0;
// }
