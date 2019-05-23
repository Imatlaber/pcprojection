/* Copyright (c) EcarX - All Rights Reserved
 * Author: Ke Ma <ke.ma@ecarx.com.cn>
 * description: KITTI oxts imu operations
 */
#ifndef _KITTIOXTS_HPP_
#define _KITTIOXTS_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

#include "utilities/hadif/reader_helper.hpp"
#include "utilities/hadif/file_helper.hpp"
#include "utilities/hadif/color_helper.h"
#include "utilities/hadif/math_helper.hpp"
#include "dataVelCamFusionBase.hpp"

#include <fstream> // std::ifstream
#include <ctime>
#include <time.h>
#include <math.h>

#include "oxtsPose.hpp"

#define SHOW(a) std::cout << #a << ": " << (a) << "\n" \
						  << std::endl
#define PI 3.14159265359

class KittiOXTS
{
public:
	KittiOXTS()
	{
		base_dir_ = "/media/mk11/Dataset/KITTI_all/Raw/2011_09_26_drive_0009/2011_09_26/2011_09_26_drive_0009_sync";
		dir_timePose_ = "/home/mk11/compileWS/pcprojection/output";
	}

	bool loadOxtsData()
	{
		int N = 30;
		std::string oxtsFileDir = base_dir_ + "/oxts/data";
		std::map<int, std::string> oxts_files = had::File::getFileNames(oxtsFileDir, ".txt", 10, 6, false);
		std::vector<cv::Mat_<double>> matOxts;

		int count = 0;
		for (auto p = oxts_files.begin(); p != oxts_files.end(); p++, count++)
		{
			// std::cout << "\t *************\t " << count << "\t *************" << std::endl;
			std::string filePath_oxts = p->second;
			std::ifstream ifs(filePath_oxts);
			std::string str;

			// std::cout << "\t filePath_oxts: " << filePath_oxts << std::endl;

			cv::Mat_<double> mat30(30, 1);
			for (int i = 0; i < N; ++i)
			{
				double adouble;
				ifs >> adouble;
				mat30(i, 0) = adouble;
				// std::cout << "\t i: " << i << "\t adouble: " << adouble << std::endl;
			}
			matOxts.push_back(mat30);

			ifs.close();

			// for (int i = 0; i < N; ++i)
			// {
			// 	std::cout << i + 1 << "\t" << matOxts.at(count).at<double>(i, 0) << std::endl;
			// }

			// std::cout << " ***************** " << std::endl;
		}

		oxtsData_ = matOxts;

		return true;
	}

	long long stringToDatetime(const std::string &str)
	{
		char *cha = (char *)str.data();
		tm timeinfo;
		int year, month, day, hour, minute, second, ns;
		sscanf(cha, "%d-%d-%d %d:%d:%d.%d", &year, &month, &day, &hour, &minute, &second, &ns);
		timeinfo.tm_year = year - 1900; // year, start from 1900
		timeinfo.tm_mon = month - 1;	// month, range: 0-11
		timeinfo.tm_mday = day;
		timeinfo.tm_hour = hour;
		timeinfo.tm_min = minute;
		timeinfo.tm_sec = second;

		timeinfo.tm_isdst = 0;
		time_t t = mktime(&timeinfo); // convert form tm ot time_t

		long long ts = t * 1e6 + ns / 1000;

		return ts;
	}

	bool loadTimestamps()
	{
		std::string fileDir_oxts = base_dir_ + "/oxts";
		std::string filePath_oxts = fileDir_oxts + "/timestamps.txt";

		std::ifstream ifs(filePath_oxts);
		std::string str;
		int count = 0;
		while (std::getline(ifs, str))
		{
			long long t = stringToDatetime(str);
			vTimestamps_.push_back(t);
			// printf("%lld\n", vTimestamps_.at(count));
			count++;
		}
		N_SAMPLES_ = count;
		ifs.close();

		return true;
	}

	double latToScale(double lat)
	{
		return cos(lat * PI / 180.0);
	}

	cv::Point2d latlonToMercator(double lat, double lon, double scale)
	{
		cv::Point2d xy;
		double er = 6378137;
		double mx = scale * lon * PI * er / 180;
		double my = scale * er * log(tan((90 + lat) * PI / 360));
		xy.x = mx;
		xy.y = my;
		return xy;
	}

	cv::Mat_<double> combineMatRt(cv::Mat_<double> R, cv::Mat_<double> t)
	{
		cv::Mat_<double> Rt_combined = (cv::Mat_<double>(4, 4) << 0, 0, 0, 0,
										0, 0, 0, 0,
										0, 0, 0, 0,
										0, 0, 0, 0);

		cv::Mat tmp = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);

		// Note: Rt_combined(cv::Rect(3, 0, 1, 3)), where x-axis is horizontal
		// and y-axis is vertical, e.g., cv::Rect(3, 0, 1, 3) means a 3x1
		// matrix starting from the upper-right corner of matrix Rt_combined.
		cv::Mat_<double> tmpR = Rt_combined(cv::Rect(0, 0, 3, 3));
		cv::Mat_<double> tmpt = Rt_combined(cv::Rect(3, 0, 1, 3));
		cv::Mat_<double> tmp1 = Rt_combined(cv::Rect(0, 3, 4, 1));

		R.copyTo(tmpR);
		t.copyTo(tmpt);
		tmp.copyTo(tmp1);

		return Rt_combined;
	}

	bool convertOxtsToPose()
	{
		std::vector<cv::Mat_<double>> oxts = oxtsData_;

		double scale = latToScale(oxts.at(0).at<double>(0, 0));
		// std::cerr << "scale: " << scale << std::endl;

		cv::Mat tr0_inv = (cv::Mat_<double>(4, 4) << 0, 0, 0, 0,
						   0, 0, 0, 0,
						   0, 0, 0, 0,
						   0, 0, 0, 0);
		for (int i = 0; i < N_SAMPLES_; i++)
		{
			// std::cerr << "\t ************** \t i: " << i << std::endl;
			// translation matrix
			double lat = oxts.at(i).at<double>(0, 0);
			double lon = oxts.at(i).at<double>(1, 0);
			cv::Point2d xy;

			xy = latlonToMercator(lat, lon, scale);

			double x = xy.x;
			double y = xy.y;
			double z = oxts.at(i).at<double>(2, 0);
			cv::Mat t = (cv::Mat_<double>(3, 1) << x, y, z);

			// rotation matrix
			double rx = oxts.at(i).at<double>(3, 0); // roll
			double ry = oxts.at(i).at<double>(4, 0); // pitch
			double rz = oxts.at(i).at<double>(5, 0); // yaw aka headign

			// base = > nav(level oxts = > rotated oxts)
			cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0,
						  0, cos(rx), -sin(rx),
						  0, sin(rx), cos(rx));
			// SHOW(Rx);
			cv::Mat Ry = (cv::Mat_<double>(3, 3) << cos(ry), 0, sin(ry),
						  0, 1, 0,
						  -sin(ry), 0, cos(ry));
			// SHOW(Ry);
			cv::Mat Rz = (cv::Mat_<double>(3, 3) << cos(rz), -sin(rz), 0,
						  sin(rz), cos(rz), 0,
						  0, 0, 1);
			// SHOW(Rz);
			cv::Mat_<double> R = Rz * Ry * Rx;

			// normalize translation and rotation (start at [0, 0, 0])
			if (i == 0)
			{
				tr0_inv = combineMatRt(R, t).inv();
			}
			cv::Mat_<double> matPose;
			matPose = tr0_inv * combineMatRt(R, t);

			// std::cout << matPose << std::endl;

			// std::cout << std::setw(20) << matPose.at<double>(0, 3);
			// std::cout << std::setw(20) << matPose.at<double>(1, 3);
			// std::cout << std::setw(20) << matPose.at<double>(2, 3) << std::endl;

			oxtsPoses_.push_back(matPose);
		}
		return true;
	}

	bool combineTimePose()
	{
		vOxtsTimePoses_.clear();
		for (int i = 0; i < N_SAMPLES_; ++i)
		{
			long long time = vTimestamps_.at(i);
			double x = oxtsPoses_.at(i).at<double>(0, 3);
			double y = oxtsPoses_.at(i).at<double>(1, 3);
			double z = oxtsPoses_.at(i).at<double>(2, 3);
			OxtsPose *op = new OxtsPose(time, x, y, z);

			vOxtsTimePoses_.push_back(*op);
		}

		return true;
	}

	bool getTimePoseCopy(std::vector<OxtsPose> &vOxtsTimePoses_copy)
	{
		vOxtsTimePoses_copy.clear();
		vOxtsTimePoses_copy = vOxtsTimePoses_;
		return true;
	}

	std::string getTimeExt()
	{
		time_t now = time(0);
		struct tm tstruct;
		char buf[80];
		tstruct = *localtime(&now);

		strftime(buf, sizeof(buf), "%Y%m%d%H%M%S", &tstruct);
		std::string mytime(buf);

		return mytime;
	}

	bool printVeloTimePoses()
	{
		for (int i = 0; i < N_SAMPLES_; ++i)
		{
			long long t = vOxtsTimePoses_.at(i).getTime();
			double x = vOxtsTimePoses_.at(i).getx();
			double y = vOxtsTimePoses_.at(i).gety();
			double z = vOxtsTimePoses_.at(i).getz();

			printf("%lld \t %.6f \t %.6f \t %.6f \t\n", t, x, y, z);
		}

		return true;
	}

	bool write2Textfile()
	{
		std::ofstream ofs;
		std::string outFileName = "sequence_txyz_";
		std::string outFilePath = dir_timePose_ + "/" + outFileName + getTimeExt() + ".txt";

		ofs.open(outFilePath);

		for (int i = 0; i < N_SAMPLES_; ++i)
		{
			long long t = vOxtsTimePoses_.at(i).getTime();
			double x = vOxtsTimePoses_.at(i).getx();
			double y = vOxtsTimePoses_.at(i).gety();
			double z = vOxtsTimePoses_.at(i).getz();

			char oline[100];
			sprintf(oline, "%lld \t %.6f \t %.6f \t %.6f \t\n", t, x, y, z);

			ofs << oline;
		}

		ofs.close();
		return true;
	}

	bool exec()
	{
		std::cerr << "\t \t \t loading loadTimestamps ... \t ";
		if (!loadTimestamps())
		{
			std::cerr << "ERROR" << std::endl;
		}
		else
		{
			std::cerr << "complete"
					  << "\n"
					  << std::endl;
		}

		std::cerr << "\t \t \t loading loadOxtsData ... \t ";
		if (!loadOxtsData())
		{
			std::cerr << "ERROR" << std::endl;
		}
		else
		{
			std::cerr << "complete"
					  << "\n"
					  << std::endl;
		}

		std::cerr << "\t \t \t loading convertOxtsToPose ... \t ";
		if (!convertOxtsToPose())
		{
			std::cerr << "ERROR" << std::endl;
		}
		else
		{
			std::cerr << "complete"
					  << "\n"
					  << std::endl;
		}

		std::cerr << "\t \t \t loading combineTimePose ... \t ";
		if (!combineTimePose())
		{
			std::cerr << "ERROR" << std::endl;
		}
		else
		{
			std::cerr << "complete"
					  << "\n"
					  << std::endl;
		}

		std::cerr << "\t \t \t loading write2Textfile ... \t ";
		if (!write2Textfile())
		{
			std::cerr << "ERROR" << std::endl;
		}
		else
		{
			std::cerr << "complete"
					  << "\n"
					  << std::endl;
		}

		return true;
	}

private:
	std::string base_dir_;
	std::string dir_timePose_;
	int N_SAMPLES_ = 0;						  // number of samples = 447
	std::vector<long long> vTimestamps_;	  // a vector of timestamps, length 447
	std::vector<cv::Mat_<double>> oxtsData_;  // a vector of 30x1 matrices, length 447
	std::vector<cv::Mat_<double>> oxtsPoses_; // a vector of 4x4 poses, length 447
	std::vector<OxtsPose> vOxtsTimePoses_;	// a vectoro of objects OxtsPose, length 447
};

#endif // _KITTIOXTS_HPP_

// #include "kittiVelCamFusion.hpp"
// #include "kittiOXTS.hpp"
// #include "oxtsPose.hpp"
// #include <iostream>

// int main(int argc, char **argv)
// {

// 	KittiOXTS *ko = new KittiOXTS();

// 	if (!ko->exec())
// 	{
// 		std::cerr << "\t mk11-error: something wrong" << std::endl;
// 	}

// 	return 0;
// }
