/* Copyright (c) AutoNavi - All Rights Reserved 
 * Author: shuo <shuo@alibaba-inc.com>
 * Description: class to extract vertical features
 */

#ifndef _LIDAR_FEATURE_VERT_HPP_
#define _LIDAR_FEATURE_VERT_HPP_

#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <opencv2/core/core.hpp>
#include "utilities/hadif/math_helper.hpp"
#include "utilities/drivers/velodyne/pcl_point_types.h"
#include "utilities/drivers/velodyne/velodyne_driver.hpp"
#include "utilities/drivers/velodyne/hdl32_driver.hpp"
#include "utilities/drivers/velodyne/hdl64_driver.hpp"
#include "lidar_feature_base.hpp"

// TODO comment origin of the range values
static constexpr double VALID_RANGE_MIN = 1.0;
static constexpr double VALID_RANGE_MAX = 70.0;
static constexpr double SELF_DETECTION_RANGE = 3;

enum class TreeType
{
    UNCLASSIFIED,
    TYPEA,
    TYPEB,
    TYPEC,
    TYPED,
    TYPEE,
    COUNT //Count is use to keep track of total elements, don't manual assign TreeType
};

template<typename T>
class LidarUnorgVertical : public LidarFeatureBase<T>
{
	public:

	LidarUnorgVertical(lcm::LCM * lcm,
					   trans_t * trans,
					   std::string ns,
					   std::string type):
		LidarFeatureBase<T>(lcm, trans, ns),
		type_in_(type)
	{
		load_params() ;
		print_params() ;
	}

	void load_params()
	{
        if(!this->param_)
            return ;

        this->param_->setNamespace(this->namespace_) ;
        this->param_->getParam("tol_horizontal", tol_horizontal_, 0.05) ;

		if(type_in_ == "hdl32" || type_in_ == "HDL32")
		{
			type_ = VelodyneType::HDL32 ;
			calib_ = dynamic_cast<VelodyneCalibration*>(new HDL32Calibration()) ;
		}
		else
		{
			printf("Error: unsupported type %s. Exit \n", type_in_.c_str()) ;
			exit(-1) ;
		}
	}

	void print_params()
	{
        printf("[%s : vert] \n", this->namespace_.c_str()) ;
        printf("\ttol_horizontal %20.3f\n", tol_horizontal_) ; 

	}

	void extract(pcl::PointCloud<T>& pc, 
                         std::vector<Eigen::Matrix3d> * covs = nullptr, 
                         std::vector<Eigen::Vector3d> * normals = nullptr, 
                         int arg=0)  override
	{
		pcl::PointCloud<T> pc_filtered ;
		if(arg == 0)
			getVerticalPoints(pc, pc_filtered) ;
		else
			getGroundPoints(pc, pc_filtered) ;
		pcl::copyPointCloud(pc_filtered, pc) ;
	}

    cv::Mat padEachLayer(cv::Mat &original, int padding_size)
    {
        cv::Mat padded_mat = cv::Mat(original.rows*padding_size, original.cols, original.type());
        for(int i=0; i<original.rows; i++)
        {
            for(int j=0; j<padding_size; j++)
            {
                original.row(i).copyTo(padded_mat.row(i*padding_size+j));
            }
        }
        return padded_mat;
    }

    //only one registerType survived
    inline void registerType(cv::Mat &cost_map, cv::Mat &label_map, int i, int j, float cost, TreeType type, bool vertical)
    {
        int label = label_map.at<int>(j, i);
		float curr_cost = cost_map.at<float>(j, i) ;

        int offset = 0;
        //Classification indices of horizontal points always comes
        //after vertical points, and hence the offset
        if(!vertical)
            offset = (int)TreeType::COUNT;

        // Update the label where the point is not classified yet or has lower type
        if(label>(int)type || label==(int)TreeType::UNCLASSIFIED)
        //if(label>(int)type || (label == (int)type && curr_cost > cost + 5e-2) || label==(int)TreeType::UNCLASSIFIED)
		{
            label_map.at<int>(j, i) = (int)type + offset;
			cost_map.at<float>(j, i) = cost ;
		}
    }

    void treePass2(bool vertical, 
                   cv::Mat& cost_map, 
                   cv::Mat &dist_map, 
                   const pcl::PointCloud<PCLVelodyne> &pcl,
                   std::vector<int8_t>& point_types,
                   cv::Mat &label_map)
	{
        float normal_xy[3] = {0, 0, 0} ;
		if(vertical)
            normal_xy[2] = 1 ;
        else
            normal_xy[0] = 1 ;

		for(int i=0; i<dist_map.cols-3; ++i)
			for(int j=0;j<dist_map.rows-3; ++j)
			{
				const auto& p1 = pcl.points[i*pcl.height+j] ;
				const auto& p2 = pcl.points[(i+1)*pcl.height+j] ;
				const auto& p3 = pcl.points[i*pcl.height+j+1] ;
				float pt21[3] = {p2.x-p1.x, p2.y-p1.y, p2.z-p1.z} ;
				float pt31[3] = {p3.x-p1.x, p3.y-p1.y, p3.z-p1.z} ;
				float d1=dist_map.at<float>(j,i) ;
				float d2=dist_map.at<float>(j,i+1) ;
				float d3=dist_map.at<float>(j+1,i) ;
               
			    // conditon 1: roughly on the same vertical plane
				float dist_thres = 0.2;
				if(fabs(d2-d1)>dist_thres || fabs(d3-d1)>dist_thres)
					continue ;

				// conditon 2: look for 3-point vertical patch
				float normal_p[3]={0} ;
				cross(pt21, pt31, normal_p) ; 
				float proj = dotprod3d(normal_p, normal_xy) ;
				float proj_thres = 1e-4 ;
				if(proj < proj_thres)
                    registerType(cost_map, label_map, i ,j, 0, TreeType::TYPEA, vertical);
				
			}

        //apply labels to the components
        for(int i=0; i<label_map.cols; i++)
        {
            for(int j=0; j<label_map.rows; j++)
                point_types[i*pcl.height+j] = label_map.at<int>(j,i);
        }

	}

    //This function walk through all the points in each vertical strips of the Velodyne
    //to classify if the points are horizontal/vertical. It is classified into 5 different
    //types and can be used to quickly obtain horizontal/vertical surface.
    //This assumes that the incoming point cloud has organized data structure similar
    //to how a 2D image is arranged, where X=rotation angle, Y=Ring index of a velodyne
    void treePass (bool vertical, 
                   cv::Mat& cost_map, 
                   cv::Mat &dist_map, 
                   const pcl::PointCloud<PCLVelodyne> &pcl, 
                   std::vector<int8_t>& point_types,
                   cv::Mat &label_map)
    {
        for(int i=0; i<dist_map.cols-3; i++)
        {
            for(int j=0; j<dist_map.rows-3; j++)
            {
                //here we look for possible vertical pts at 5 different combinations
                // Y   A        B        C        D        E
                // ^ | X X    X | X    | X X    X | X    X X |
                // | | X X    | X X    X | X    X | X    X | X
                // | | X X    | X X    | X X    | X X    | X X
                // ----> X
                float distance = pcl.points[i*pcl.height+j].distance;

                // <1.7 is where self detection occurred since the Velodyne is mounted
                // very near to the roof
                // TODO: Setup generic filtering class for pointcloud
                if(distance<SELF_DETECTION_RANGE || distance>VALID_RANGE_MAX)
                    continue;
                float X1Y1 = dist_map.at<float>(j,i);
                float X1Y2 = dist_map.at<float>(j+1,i);
                float X1Y3 = dist_map.at<float>(j+2,i);
                float X2Y3 = dist_map.at<float>(j+2, i+1);
                float X2Y2 = dist_map.at<float>(j+1, i+1);
                float X3Y3 = dist_map.at<float>(j+2, i+2);

                float dist_thres = tol_horizontal_ ;

                //Using class enumeration for point type classification
                //case A
				float cost_A1 = fabs(X1Y1-X1Y2) ;
				float cost_A2 = fabs(X1Y2-X1Y3) ;
                bool A1 = cost_A1<dist_thres;
                bool A2 = cost_A2<dist_thres;
                //case B
				float cost_B1 = cost_A1 ;
				float cost_B2 = fabs(X1Y2-X2Y3) ;
                bool B1 = A1;
                bool B2 = cost_B2<dist_thres;
                //case C
				float cost_C1 = fabs(X2Y2-X1Y1) ;
				float cost_C2 = fabs(X1Y3-X2Y2) ;
                bool C1 = cost_C1<dist_thres;
                bool C2 = cost_C2<dist_thres;
                //case D
				float cost_D1 = cost_C1 ;
				float cost_D2 = fabs(X2Y3-X2Y2) ;
                bool D1 = C1;
                bool D2 = cost_D2<dist_thres;
                //case E
				float cost_E1 = cost_D1 ;
				float cost_E2 = fabs(X3Y3-X2Y2) ;
                bool E1 = D1;
                bool E2 = cost_E2<dist_thres;

                if(A1)
                {
                    //This is the most relax criteria, where only two points is used for classification
                    //Since we only have 16 layers, we use the following in order to allow more features extracted
                    registerType(cost_map, label_map, i ,j, cost_A1, TreeType::TYPEA, vertical);
                    registerType(cost_map, label_map, i, j+1, cost_A1, TreeType::TYPEA, vertical);
                }
                if(A1&&A2)
                {
                    //Layers arrangment matches type A
                    registerType(cost_map, label_map, i ,j, (cost_A1+cost_A2)/2.0, TreeType::TYPEA, vertical);
                    registerType(cost_map, label_map, i, j+1, (cost_A1+cost_A2)/2.0, TreeType::TYPEA, vertical);
                    registerType(cost_map, label_map, i, j+2, (cost_A1+cost_A2)/2.0, TreeType::TYPEA, vertical);
                }

                if(B1&&B2)
                {
                    //Layers arrangment matches type B
                    registerType(cost_map, label_map, i, j, (cost_B1+cost_B2)/2.0, TreeType::TYPEB, vertical);
                    registerType(cost_map, label_map, i, j+1, (cost_B1+cost_B2)/2.0, TreeType::TYPEB, vertical);
                    registerType(cost_map, label_map, (i+1), j+2, (cost_B1+cost_B2)/2.0, TreeType::TYPEB, vertical);
                }

                if(C1&&C2)
                {
                    //Layers arrangment matches type C
                    registerType(cost_map, label_map, i, j, (cost_C1+cost_C2)/2.0, TreeType::TYPEC, vertical);
                    registerType(cost_map, label_map, (i+1), j+1, (cost_C1+cost_C2)/2.0, TreeType::TYPEC, vertical);
                    registerType(cost_map, label_map, i, j+2, (cost_C1+cost_C2)/2.0, TreeType::TYPEC, vertical);
                }

                if(D1&&D2)
                {
                    //Layers arrangment matches type D
                    registerType(cost_map, label_map, i, j, (cost_D1+cost_D2)/2.0, TreeType::TYPED, vertical);
                    registerType(cost_map, label_map, (i+1), j+1, (cost_D1+cost_D2)/2.0, TreeType::TYPED, vertical);
                    registerType(cost_map, label_map, (i+1), j+2, (cost_D1+cost_D2)/2.0, TreeType::TYPED, vertical);
                }

                if(E1&&E2)
                {
                    //Layers arrangment matches type E
                    registerType(cost_map, label_map, i, j, (cost_E1+cost_E2)/2.0, TreeType::TYPEE, vertical);
                    registerType(cost_map, label_map, (i+1), j+1, (cost_E1+cost_E2)/2.0, TreeType::TYPEE, vertical);
                    registerType(cost_map, label_map, (i+2), j+2, (cost_E1+cost_E2)/2.0, TreeType::TYPEE, vertical);
                }
            }
        }

        //apply labels to the components
        for(int i=0; i<label_map.cols; i++)
        {
            for(int j=0; j<label_map.rows; j++)
                point_types[i*pcl.height+j] = label_map.at<int>(j,i);
        }
    }

    std::vector<int8_t> getLCutVelodyne(const pcl::PointCloud<T> &pcl, bool is_vertical)
    {
		assert(pcl.height == calib_->total_ring) ;

        int horizontal_pts = pcl.width;
        cv::Mat distx_map = cv::Mat(pcl.height, horizontal_pts, CV_32FC1);
        cv::Mat disty_map = cv::Mat(pcl.height, horizontal_pts, CV_32FC1);
        cv::Mat cost_map = cv::Mat(pcl.height, horizontal_pts, CV_32FC1);
        cv::Mat label_map = cv::Mat(pcl.height, horizontal_pts, CV_32SC1);

        std::vector<double> angle_indices ;
        for(const auto& c : calib_->cal_)
			angle_indices.push_back(c.angle) ;


        for(int hor_idx=0; hor_idx<horizontal_pts; hor_idx++)
        {
            for(int i=hor_idx*pcl.height, angle_idx = 0; i<(hor_idx*pcl.height)+pcl.height; ++i, ++angle_idx)
            {
				const auto& p = pcl.points[i] ;
				double x = sqrt(p.x*p.x+p.y*p.y) ;
				double y = p.z ;
                //double distance = pcl.points[i].distance;
                //double x = distance  * cos(angle_indices[angle_idx]);
                //double y = distance * sin(angle_indices[angle_idx]);
                distx_map.at<float>(i%pcl.height, i/pcl.height) = x;
                disty_map.at<float>(i%pcl.height, i/pcl.height) = y;
				cost_map.at<float>(i%pcl.height, i/pcl.height) = FLT_MAX;
                label_map.at<int>(i%pcl.height, i/pcl.height) = 0;
            }
        }

        std::vector<int8_t> point_types ;
        point_types.resize(pcl.size()) ;
        treePass(is_vertical, cost_map, distx_map, pcl, point_types, label_map);
        
        return point_types;
    }

    std::vector<int8_t> getLCutVelodyne2(const pcl::PointCloud<T> &pcl, bool is_vertical)
    {
		assert(pcl.height == calib_->total_ring) ;

        int horizontal_pts = pcl.width;
        cv::Mat distx_map = cv::Mat(pcl.height, horizontal_pts, CV_32FC1);
        cv::Mat disty_map = cv::Mat(pcl.height, horizontal_pts, CV_32FC1);
        cv::Mat cost_map = cv::Mat(pcl.height, horizontal_pts, CV_32FC1);
        cv::Mat label_map = cv::Mat(pcl.height, horizontal_pts, CV_32SC1);

        std::vector<double> angle_indices ;
        for(const auto& c : calib_->cal_)
			angle_indices.push_back(c.angle) ;


        for(int hor_idx=0; hor_idx<horizontal_pts; hor_idx++)
        {
            for(int i=hor_idx*pcl.height, angle_idx = 0; i<(hor_idx*pcl.height)+pcl.height; ++i, ++angle_idx)
            {
				const auto& p = pcl.points[i] ;
				double x = sqrt(p.x*p.x+p.y*p.y) ;
				double y = sqrt(p.distance*p.distance-x*x) ;
                //double distance = pcl.points[i].distance;
                //double x = distance  * cos(angle_indices[angle_idx]);
                //double y = distance * sin(angle_indices[angle_idx]);
                distx_map.at<float>(i%pcl.height, i/pcl.height) = x;
                disty_map.at<float>(i%pcl.height, i/pcl.height) = y;
				cost_map.at<float>(i%pcl.height, i/pcl.height) = FLT_MAX;
                label_map.at<int>(i%pcl.height, i/pcl.height) = 0;
            }
        }
    
        std::vector<int8_t> point_types ;
        point_types.resize(pcl.size()) ;
        treePass2(is_vertical, cost_map, distx_map, pcl, point_types, label_map);
        
        return point_types;
    }

    pcl::PointCloud<PCLVelodyne> connectedComponentAnalysis(pcl::PointCloud<PCLVelodyne> &pcl,
                                    cv::Mat &label_map, cv::Mat &distx_map)
    {
        int horizontal_pts = pcl.width;
        cv::Mat compt_map = cv::Mat::zeros(pcl.height, horizontal_pts, CV_32SC1);
        //perform connected component analysis
        int compt_id = 1;
        std::map<int, int> label_root_map;
        for(int i=0; i<label_map.cols; i++)
        {
            for(int j=0; j<label_map.rows; j++)
            {
                int label = label_map.at<int>(j,i);
                float dist_x = distx_map.at<float>(j, i);
                if(label > 0)
                {
                    //get smallest neighboring label
                    std::vector<int> n_labels;
                    for(int k=-1; k<2; k++)
                    {
                        for(int l=-1; l<2; l++)
                        {
                            int x=i+k;
                            int y=j+l;
                            if(x>=0 && x<compt_map.cols && y>=0 && y<compt_map.rows)
                            {
                                int current_label = compt_map.at<int>(y, x);
                                int neighbor_type = label_map.at<int>(y, x);
                                float dist_x_neighbor = distx_map.at<float>(y, x);
                                bool close_neighbor = fabs(dist_x-dist_x_neighbor)<0.2;
                                bool same_neighbor_type = false;
                                if(neighbor_type>5 && label > 5) same_neighbor_type = true;
                                //special attention to vertical features
                                else if(neighbor_type<=5 && label <=5 && close_neighbor) same_neighbor_type = true;
                                if(current_label!=0 && same_neighbor_type)
                                    n_labels.push_back(current_label);
                            }
                        }
                    }
                    int smallest_label = 0;
                    if(n_labels.size()>0)
                    {
                        smallest_label = n_labels[0];
                        for(size_t m = 1; m<n_labels.size(); m++)
                            if(smallest_label>n_labels[m]) smallest_label = n_labels[m];
                    }
                    //neighbor label exist
                    if(smallest_label>0)
                    {
                        compt_map.at<int>(j,i) = smallest_label;
                        for(size_t m=0; m<n_labels.size(); m++)
                        {
                            if(label_root_map[n_labels[m]]>smallest_label)
                                label_root_map[n_labels[m]] = smallest_label;
                        }
                    }
                    else
                    {
                        compt_map.at<int>(j,i) = compt_id;
                        label_root_map[compt_id] = compt_id;
                        compt_id++;
                    }
                }
            }
        }

        //reindex so that the label always start from 1 and continuously
        //indexing to the total unique objects
        std::map<int, std::vector<int>> reindex_label_root_map;
        for(std::map<int, int>::iterator it = label_root_map.begin();
            it != label_root_map.end(); it++
            )
        {
            reindex_label_root_map[it->second].push_back(it->first);
        }

        std::map<int, int> final_reindex;
        //let's have 1024 shades of jet color to choose from

        int color_interval = 1024/label_root_map.size();
        for(std::map<int, std::vector<int>>::iterator it = reindex_label_root_map.begin();
            it != reindex_label_root_map.end(); it++
            )
        {
            int final_index = (rand() % label_root_map.size() + 1)*color_interval;
            for(size_t i=0; i<it->second.size(); i++)
            {
                final_reindex[it->second[i]] = final_index;
            }
//            final_index++;
        }

        pcl::PointCloud<PCLVelodyne> vertical_pcl;
        vertical_pcl.header = pcl.header;
        //apply labels to the components
        for(int i=0; i<label_map.cols; i++)
        {
            for(int j=0; j<label_map.rows; j++)
            {
                PCLVelodyne pt = pcl.points[i*pcl.height+j];
                pt.type = label_map.at<int>(j,i);
                if(pt.type>0 /*&& pt.type<6*/)
                {
                    pt.label = final_reindex[compt_map.at<int>(j,i)];
                    vertical_pcl.points.push_back(pt);
                }
            }
        }
        return vertical_pcl;
    }

    void getVerticalPoints(const pcl::PointCloud<T> &pc, pcl::PointCloud<PCLVelodyne>& pc_ext_filtered)
    {
        pcl::copyPointCloud(pc, pc_ext_filtered) ;
		pc_ext_filtered.clear() ;
        std::vector<int8_t> point_types = getLCutVelodyne(pc, true);
        for(size_t i=0; i<pc.size(); ++i)
        {
            if(point_types[i] > (int)TreeType::UNCLASSIFIED && point_types[i] < (int)TreeType::COUNT)
            {
                pc_ext_filtered.push_back(pc[i]) ;
            }
        }
    }

    void getVerticalPoints2(const pcl::PointCloud<T> &pc, pcl::PointCloud<PCLVelodyne>& pc_ext_filtered)
    {
        pcl::copyPointCloud(pc, pc_ext_filtered) ;
		pc_ext_filtered.clear() ;
        std::vector<int8_t> point_types = getLCutVelodyne2(pc, true);
        for(size_t i=0; i<pc.size(); ++i)
        {
            if(point_types[i] > (int)TreeType::UNCLASSIFIED && point_types[i] < (int)TreeType::COUNT)
            {
                pc_ext_filtered.push_back(pc[i]) ;
            }
        }
    }

    void getGroundPoints(const pcl::PointCloud<T> &pc, pcl::PointCloud<PCLVelodyne>& pc_ext_filtered)
    {
        pcl::copyPointCloud(pc, pc_ext_filtered) ;
		pc_ext_filtered.clear() ;
        std::vector<int8_t> point_types = getLCutVelodyne(pc, false);
        for(size_t i=0; i<pc.size(); ++i)
        {
            if(point_types[i] > (int)TreeType::TYPEE && pc[i].z < 0.0) //assuming velodyne is mounted higher than ground
                pc_ext_filtered.push_back(pc[i]);
        }
    }

    void getGroundPoints2(const pcl::PointCloud<T> &pc, pcl::PointCloud<PCLVelodyne>& pc_ext_filtered)
    {
        pcl::copyPointCloud(pc, pc_ext_filtered) ;
		pc_ext_filtered.clear() ;
        std::vector<int8_t> point_types = getLCutVelodyne2(pc, false);
        for(size_t i=0; i<pc.size(); ++i)
        {
            if(point_types[i] > (int)TreeType::TYPEE && pc[i].z < 0.0) //assuming velodyne is mounted higher than ground
                pc_ext_filtered.push_back(pc[i]);
        }
    }


public:
	VelodyneCalibration * calib_ ;	
	VelodyneType type_ ;
	double tol_horizontal_ ;	
	std::string type_in_ ;
} ;














#endif 
