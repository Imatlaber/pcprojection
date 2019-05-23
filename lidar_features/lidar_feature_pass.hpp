/* Copyright (c) AutoNavi - All Rights Reserved 
 * Author: shuo <shuo@alibaba-inc.com>
 * Description: class to pass through a range of points
 */

#ifndef _LIDAR_FEATURE_PASS_HPP_
#define _LIDAR_FEATURE_PASS_HPP_

#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/filters/passthrough.h>
#include "lidar_feature_base.hpp"

template<typename T>
class LidarUnorgPass : public LidarFeatureBase<T>
{
	public:
    LidarUnorgPass():
        LidarFeatureBase<T>()
    {
        setup() ;
    }
    
    LidarUnorgPass(lcm::LCM * lcm,
                    trans_t * trans,
                    std::string ns):
        LidarFeatureBase<T>(lcm, trans, ns)
    {
        setup() ;
    }

    void setup()
    {
        load_params() ;
        print_params() ;
    }

    void load_params() 
    {
        if(!this->param_)
            return ;

        this->param_->setNamespace(this->namespace_) ;
        this->param_->getParam("x_min", x_min_, -150.0) ;
        this->param_->getParam("x_max", x_max_,  150.0) ;
        this->param_->getParam("y_min", y_min_, -150.0) ;
        this->param_->getParam("y_max", y_max_,  150.0) ;
        this->param_->getParam("z_min", z_min_, -150.0) ;
        this->param_->getParam("z_max", z_max_,  150.0) ;
        this->param_->getParam("distance_min", d_min_,  0.0) ;
        this->param_->getParam("distance_max", d_max_,  150.0) ;
        this->param_->getParam("azimuth_min", a_min_,  -200.0) ;
        this->param_->getParam("azimuth_max", a_max_,  200.0) ;
		
		double factor = M_PI/180.0 ;
		a_min_ *= factor ;
		a_max_ *= factor ;
    }

    void print_params() 
    {
        printf("[%s : pass] \n", this->namespace_.c_str()) ;
        printf("\tx_min %20.3f\n",x_min_) ; 
        printf("\tx_max %20.3f\n",x_max_) ; 
        printf("\ty_min %20.3f\n",y_min_) ; 
        printf("\ty_max %20.3f\n",y_max_) ; 
        printf("\tz_min %20.3f\n",z_min_) ; 
        printf("\tz_max %20.3f\n",z_max_) ; 
        printf("\tdistance_min %20.3f\n",d_min_) ; 
        printf("\tdistance_max %20.3f\n",d_max_) ; 
    }

	void extract(pcl::PointCloud<T>& pc,
                         std::vector<Eigen::Matrix3d> * covs = nullptr, 
                         std::vector<Eigen::Vector3d> * normals = nullptr, 
                         int arg=0) override
	{
		pcl::PassThrough<T> pass ;
	    pcl::IndicesPtr inliers_indices(new std::vector<int>()) ;
      
        /// pass filter in x 
		pass.setInputCloud(pc.makeShared()) ;
        filter_by_field(pass, inliers_indices, "x", x_min_, x_max_, false) ;
       
        /// pass filter in y
        pass.setIndices(inliers_indices) ;
        filter_by_field(pass, inliers_indices, "y", y_min_, y_max_, false) ;
        
        /// pass filter in z
        pass.setIndices(inliers_indices) ;
        filter_by_field(pass, inliers_indices, "z", z_min_, z_max_, false) ;
        
        /// pass filter in distance
        pass.setIndices(inliers_indices) ;
        filter_by_field(pass, inliers_indices, "distance", d_min_, d_max_, false) ;
        
        /// get cloud from indices and copy to output
		pcl::PointCloud<T> filtered_pc ;
		pcl::copyPointCloud(pc, *inliers_indices, filtered_pc) ;
		//pcl::copyPointCloud(filtered_pc, pc) ;
		//pass.filter(filtered_pc) ;

		// filter in azimuth 
		pcl::PointCloud<T> filtered_azimuth ; 
		pcl::copyPointCloud(filtered_pc, filtered_azimuth) ;
		filter_by_azimuth(filtered_pc, a_min_, a_max_, filtered_azimuth) ;
		pcl::copyPointCloud(filtered_azimuth, pc) ;
	}

	void filter_by_field(pcl::PassThrough<T>& filter, pcl::IndicesPtr indices, std::string field_name, float min, float max, bool FilterLimitsNegative)
    {
		filter.setFilterFieldName(field_name) ;
		filter.setFilterLimits(min, max) ;
		filter.setFilterLimitsNegative(FilterLimitsNegative);
		filter.filter(*indices) ; 
		//printf("filtered indices size %zu \n", indices->size()) ;
    }

	void filter_by_field(pcl::PointCloud<T>& cloud, std::string field_name, float min, float max, bool FilterLimitsNegative)
	{
		pcl::PointCloud<T> filtered_pc ;
		pcl::PassThrough<T> pass ;
		pass.setInputCloud(cloud.makeShared()) ;
		pass.setFilterFieldName(field_name) ;
		pass.setFilterLimits(min, max) ;
		pass.setFilterLimitsNegative(FilterLimitsNegative);
		pass.filter(filtered_pc) ;
		pcl::copyPointCloud(filtered_pc, cloud) ;
	}

	void filter_by_azimuth(const pcl::PointCloud<T>& cloud_in, float min, float max, pcl::PointCloud<T>& cloud_out)
	{
		cloud_out.clear() ;
		for(const auto& p : cloud_in)
		{
			double azimuth = atan2(p.y, p.x); 
			if(azimuth > min && azimuth < max)
				cloud_out.push_back(p) ;
		}
		cloud_out.width = cloud_out.size() ;
		cloud_out.height = 1 ;
		cloud_out.is_dense = true ;
	}

    public:
    
    double x_min_ ;
    double x_max_ ;
    double y_min_ ;
    double y_max_ ;
    double z_min_ ;
    double z_max_ ;
    double d_min_ ;
    double d_max_ ;
	double a_min_ ;
	double a_max_ ;
    

} ;



#endif
