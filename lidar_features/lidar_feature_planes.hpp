/* Copyright (c) AutoNavi - All Rights Reserved 
 * Author: Baowei <baowei.lbw@alibaba-inc.com>
 * Description: to find planes from a pointcloud
 */

#ifndef _LIDAR_FEATURE_PLANE_HPP_
#define _LIDAR_FEATURE_PLANE_HPP_

#include "../../utilities/drivers/velodyne/pcl_point_types.h"
#include <vector>
#include <string>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include "../../common/segment_solver/had_seg.h"
#include "lidar_feature_base.hpp"
#include "../../utilities/hadif/math_helper.hpp"
#include "../../common/pcl_viewer/pcl_viewer.hpp"
#include "../../lidar_features/lidar_feature_voxelgridnotmean.hpp"

template<typename T>
class LidarPlanes : public LidarFeatureBase<T>
{
public:
	std::vector< std::pair<pcl::PointCloud<T>, Eigen::Vector3f> > planeAndNormal_;
	
	LidarPlanes():
	LidarFeatureBase<T>()
	{
		setup() ;
	}
	
	LidarPlanes(lcm::LCM * lcm,	trans_t * trans, std::string ns): LidarFeatureBase<T>(lcm, trans, ns)
	{
		setup() ;
	}
	
	
	
	
	void setup()
	{
		load_params() ;
		print_params() ;
	}
	
	void returnPlaneAndNormal(std::vector< std::pair<pcl::PointCloud<T>, Eigen::Vector3f> > & planeAndNormal)
	{
		planeAndNormal = planeAndNormal_;
	}
	
	void load_params() 
	{
		if(!this->param_)
			return ;
		
		this->param_->setNamespace(this->namespace_) ;
		this->param_->getParam("distance_threshold", distance_threshold_, 0.01) ;
		this->param_->getParam("min_residual_ratio", min_residual_ratio_, 0.2) ;
		this->param_->getParam("min_inliers_ratio", min_inliers_ratio_, 0.2) ;
		this->param_->getParam("max_search_radius", max_search_radius_, 20.0) ;
		this->param_->getParam("max_plane_points", max_plane_points_, 100) ;
		this->param_->getParam("max_distance_ratio", max_distance_ratio_, 1.0) ;
		this->param_->getParam("show_cloud_orig", show_cloud_orig_, false) ;
		this->param_->getParam("show", show_, false) ;
		this->param_->getParam("incorrect_normal_ratio",incorrect_normal_ratio_, 0.5);			 
	}
	
	void print_params() 
	{
		printf("[%s : plane] \n", this->namespace_.c_str()) ;
		printf("\tdistance_threshold %20.3f\n",distance_threshold_) ; 
		printf("\tmin_residual_ratio %20.3f\n",min_residual_ratio_) ; 
		printf("\tmin_inliers_ratio %20.3f\n",min_inliers_ratio_) ; 
		printf("\tmax_search_radius %20.3f\n",max_search_radius_) ; 
		printf("\tmax_plane_points %20d\n",max_plane_points_) ; 
		printf("\tshow_cloud_orig %20.3d\n",show_cloud_orig_) ; 
		printf("\tshow %20.3d\n",show_) ; 
		printf("\tincorrect_normal_ratio %20.3f\n",incorrect_normal_ratio_);
	}
	
	bool reject_trees(pcl::PointCloud<T>& ori_pc, pcl::PointCloud<T>& cloud, const pcl::ModelCoefficients& plane) 
	{
		
		
		float m_voxelGridleafseiz = 0.3;
		pcl::PointCloud<T> ori_pc_new, cloud_new;
		VoxelGrid<T> sor;
		sor.setLeafSize (m_voxelGridleafseiz, m_voxelGridleafseiz, m_voxelGridleafseiz);
		sor.setInputCloud (ori_pc.makeShared());
		sor.filter (ori_pc_new);
		
		sor.setInputCloud (cloud.makeShared());
		sor.filter (cloud_new);
		
		// 			pcl::visualization::PCLVisualizer viewer;
		// 			viewer.removeAllPointClouds();
		// 			viewer.removeAllShapes();
		// 			pcl::visualization::PointCloudColorHandlerCustom<T> prev_cloud_color ( ori_pc_new.makeShared(), 0, 255, 255 );
		// 			viewer.addPointCloud (ori_pc_new.makeShared(), prev_cloud_color, "curr_points_new");   
		// 			pcl::visualization::PointCloudColorHandlerCustom<T> curr_cloud_color ( cloud_new.makeShared(), 255, 255, 0 );
		// 			viewer.addPointCloud (cloud_new.makeShared(), curr_cloud_color, "prev_points_new"); 
		
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		typename pcl::search::KdTree<T>::Ptr searchTree (new pcl::search::KdTree<T>);
		searchTree->setInputCloud(ori_pc_new.makeShared());
		pcl::NormalEstimationOMP<T, pcl::Normal> normalEstimator;
		normalEstimator.setInputCloud(cloud_new.makeShared());
		normalEstimator.setSearchMethod(searchTree);
		int corr_k = 20;
		if(cloud_new.size() > 20)
			corr_k = 20;
		else corr_k = cloud_new.size();
		normalEstimator.setKSearch(corr_k);
		normalEstimator.setSearchSurface(ori_pc_new.makeShared());
		normalEstimator.compute(*normals);
		
		// 			viewer.addPointCloudNormals<T, pcl::Normal> (cloud_new.makeShared(), normals, 10, 0.7, "curr_normals");
		pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
		normal->resize(1);
		normal->at(0).normal_x = plane.values[0];
		normal->at(0).normal_y = plane.values[1];
		normal->at(0).normal_z = plane.values[2];
		typename pcl::PointCloud<T> onept;
		onept.resize(1);
		onept.at(0) = cloud_new.at(0);
		// 			viewer.addPointCloudNormals<T, pcl::Normal> (onept.makeShared(), normal, 1, 2, "normals");
		//while(!viewer.wasStopped())
		// 				viewer.spinOnce();	
		int outlier_count = 0;
		for(int i = 0; i < normals->size(); ++ i)
		{
			std::vector<float> point_normal(3);
			point_normal[0] = normals->at(i).normal_x;
			point_normal[1] = normals->at(i).normal_y;
			point_normal[2] = normals->at(i).normal_z;
			std::vector<float> plane_normal(3);
			plane_normal[0] = plane.values[0];
			plane_normal[1] = plane.values[1];
			plane_normal[2] = plane.values[2];
			if(angleBtwVectors3d(point_normal.data(), plane_normal.data())> 5*M_PI/180 && angleBtwVectors3d(point_normal.data(), plane_normal.data()) < 175*M_PI/180)
			{
				outlier_count ++;
			}
		}
		if(outlier_count /static_cast<double>(normals->size()) > incorrect_normal_ratio_) //reject curr plane
			return true;
		else 
			return false;
	}
	
	void extract(pcl::PointCloud<T>& pc, std::vector<Eigen::Matrix3d> * covs = nullptr, std::vector<Eigen::Vector3d> * normals = nullptr, int arg=0) override
	{
		
		planeAndNormal_.resize(0);
		/// clear up covs and normals
		if(arg > 0)
		{
			covs->clear() ;
			normals->clear() ;
		}
		/// create segmentation object
		had::HadSegment<T> seg ;
		seg.setOptimizeCoefficients(true) ;
		seg.setModelType(had::HAD_SACMODEL_PLANE) ;
		seg.setMethodType(had::HAD_SACMETHOD_RANSAC) ;
		//seg.setMaxIterations(1000) ;
		seg.setDistanceThreshold(distance_threshold_) ;
		typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>()) ;
		
		/// crate filtering object
		pcl::ExtractIndices<T> extract ;
		pcl::ModelCoefficientsPtr coeff(new pcl::ModelCoefficients()) ;
		std::vector<pcl::ModelCoefficients> planes ;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices()) ;
		
		pcl::PointCloud<T> ori_pc;
		pcl::copyPointCloud(pc, ori_pc) ;
		
		/// extract planes one by one
		size_t N = pc.size() ;
		pcl::PointCloud<T> pc_left, pc_outliers ;
		pcl::copyPointCloud(pc, pc_left) ;
		pc.clear() ;
		double ratio_left = 0.3 ;
		std::vector< std::array<int,3> > colors ;
		colors.push_back(std::array<int,3>{{255, 0, 0}}) ;
		colors.push_back(std::array<int,3>{{0, 255, 0}}) ;
		colors.push_back(std::array<int,3>{{0, 0, 255}}) ;
		std::array<int, 3> color_unselected{{178, 102, 255}} ;
		int cnt = 0, cnt_bad = 0 ;
		
		/// create viewer object
		boost::shared_ptr<had::PCLViewer> viewer ;
		if(show_)
			viewer = boost::shared_ptr<had::PCLViewer>(new had::PCLViewer("All Planes")) ;
		char cloud_name[50] ;
		
		typename pcl::PointCloud<T>::Ptr pc_orig(new pcl::PointCloud<T>()) ;
		pcl::copyPointCloud(pc_left, *pc_orig) ;
		
		/// add original cloud to viewer (optional)
		if(show_cloud_orig_)
		{
			std::array<int,3> color_orig{{100, 100, 100}} ;
			sprintf(cloud_name, "cloud_orig") ;
			if(show_)
				viewer->add_cloud_to_viewer<T>(pc_orig, color_orig, cloud_name) ;
		}
		int planeID = 0;
		
		while(true)
		{
			/// segment the largest plane from the remaining pointcloud 
			seg.setInputCloud(pc_left.makeShared()) ;
			tree->setInputCloud(pc_left.makeShared()) ;
			seg.setSamplesMaxDist(max_search_radius_, tree) ; 
			seg.segment(*inliers, *coeff) ;
// 			double search_radius = 0. ;
// 			seg.getModel()->getSamplesMaxDist(search_radius) ;
// 			printf("ransac search radius is %f \n", search_radius) ;
			if(inliers->indices.size() == 0)
			{
				printf("error: failed to extract a plane from pointcloud \n") ;
				break ;
			}
			
			/// extract the inliers
			typename pcl::PointCloud<T>::Ptr pc_inliers(new pcl::PointCloud<T>()) ;
			extract.setInputCloud(pc_left.makeShared()) ;
			extract.setIndices(inliers) ;
			extract.setNegative(false) ;
			extract.filter(*pc_inliers) ;
			printf("extracted a plane %d of %zu points \n", cnt, pc_inliers->size()) ;
			
			/// check termination conditon
			if(pc_inliers->size() < min_inliers_ratio_ * N)
				break ;
			
			bool good_normals = !reject_trees(ori_pc, *pc_inliers, *coeff);						 
			
			if(good_normals)
			{
				//down_sample(*pc_inliers, *coeff) ;
				for(auto &p : pc_inliers->points)
				{
					p.id = planeID;
				}				
				planeID ++;
				if(arg > 0)
				{
					Eigen::Matrix3d cov_plane ;
					Eigen::Vector3d normal_plane ;
					compute_covaraince(*pc_inliers, cov_plane, normal_plane) ; 
					covs->push_back(cov_plane) ;
					normal_plane = Eigen::Vector3d(coeff->values[0],coeff->values[1],coeff->values[2]) ;
					normals->push_back(normal_plane) ;
				}
				//std::pair<pcl::PointCloud<T>, Eigen::Vector3f> pair_plane_normal;
				//pair_plane_normal.first = *pc_inliers;
				//Eigen::Vector3f tmpnormal(coeff->values[0],coeff->values[1],coeff->values[2]);
				//pair_plane_normal.second = tmpnormal;
				//planeAndNormal_.push_back(pair_plane_normal);
				pc += *pc_inliers ;
				planes.push_back(*coeff) ;
				printf("total %zu planar points so far \n", pc.size()) ;
				/// add to viewer
				sprintf(cloud_name, "cloud_%d", cnt) ;			
				
				if(show_)
				{
					viewer->add_cloud_to_viewer<T>(pc_inliers, colors[cnt%colors.size()], cloud_name) ;
					viewer->spin_once();
				}
				++cnt ;
				
				printf("---------------- total %d planes selected ----------- \n", cnt) ;	
			}						 
			
			/// get remaining pointcloud for the next iteration
			extract.setNegative(true) ;
			extract.filter(pc_outliers) ;
			pc_left.swap(pc_outliers) ;
			
			
			if(pc_left.size() < min_residual_ratio_ * N)
				break ;
			
		}
		
		printf("%d planes rejected due to diverging normals \n", cnt_bad) ;
		std::sort(pc.points.begin(), pc.points.end(), LidarPlanes::comparator_timestamp) ;
		if(show_)
			viewer->show() ;
	}			 
	
	bool parallel_plane(const std::vector<pcl::ModelCoefficients>& planes, const pcl::ModelCoefficients& p)
	{
		double angle_diff = 0. ;
		bool pplane = false ;
		//printf("new plane : %g %g %g %g \n", p.values[0], p.values[1], p.values[2], p.values[3]) ;
		for(const auto& q : planes)
		{
			//printf("plane : %g %g %g %g \n", q.values[0], q.values[1], q.values[2], q.values[3]) ;
			angle_diff = angleBtwVectors3d(q.values.data(), p.values.data()) ;
			/// angle_diff is in the interval of [0, pi], and both planes with 0 and pi normal are parallel
			if(fabs(angle_diff) < min_normal_diff_)
			{
				pplane = true ;
				break ;
				
			}
			
			if(fabs(angle_diff-M_PI) <min_normal_diff_)
			{
				pplane = true ;
				break ;
			}
		}
		
		printf("parallel_plane status %d, angle_diff %f, minimum angle_diff %f \n", pplane, angle_diff, min_normal_diff_) ;
		
		return pplane ; 
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
	
	static bool comparator_distance(const T& p1, const T& p2)
	{
		return (p1.distance < p2.distance) ;
	}
	
	static bool comparator_timestamp(const T& p1, const T& p2)
	{
		int64_t curr_utime1 = 0 ;
		memcpy(&curr_utime1, &p1.utime1, sizeof(int32_t)) ;
		memcpy((char*)&curr_utime1+4, &p1.utime2, sizeof(int32_t)) ;
		int64_t curr_utime2 = 0 ;
		memcpy(&curr_utime2, &p2.utime1, sizeof(int32_t)) ;
		memcpy((char*)&curr_utime2+4, &p2.utime2, sizeof(int32_t)) ;
		return (curr_utime1 < curr_utime2) ;
	}
	
	void down_sample_distance(pcl::PointCloud<T>& cloud, const pcl::ModelCoefficients& plane) 
	{
		std::sort(cloud.points.begin(), cloud.points.end(), LidarPlanes::comparator_distance) ;
		pcl::PointCloud<T> cloud_out ;
		cloud_out.reserve(max_plane_points_) ; 
		for(int i=0; i<cloud.size() && i<max_plane_points_; ++i)
		{
			cloud_out.push_back(cloud[i]) ;
			//printf("distance %d : %f \n", i, cloud[i].distance) ;
		}
		cloud.swap(cloud_out) ;
	}
	
	void compute_covaraince(const pcl::PointCloud<T>& cloud, Eigen::Matrix3d& cov_out, Eigen::Vector3d& normal_out,  double epsilon = 1e-5)
	{
		Eigen::Matrix3d cov ;
		Eigen::Vector3d mean ;
		
		// Zero out the cov and mean
		cov.setZero ();
		mean.setZero ();
		
		// Find the covariance matrix
		size_t N = cloud.size() ;
		for(size_t j = 0; j < N; j++) {
			const T &pt = cloud[j] ;
			
			mean[0] += pt.x;
			mean[1] += pt.y;
			mean[2] += pt.z;
			
			cov(0,0) += pt.x*pt.x;
			
			cov(1,0) += pt.y*pt.x;
			cov(1,1) += pt.y*pt.y;
			
			cov(2,0) += pt.z*pt.x;
			cov(2,1) += pt.z*pt.y;
			cov(2,2) += pt.z*pt.z;    
		}
		
		mean /= static_cast<double> (N);
		// Get the actual covariance
		for (int k = 0; k < 3; k++)
			for (int l = 0; l <= k; l++) 
			{
				cov(k,l) /= static_cast<double> (N);
				cov(k,l) -= mean[k]*mean[l];
				cov(l,k) = cov(k,l);
			}
			
			// Compute the SVD (covariance matrix is symmetric so U = V')
			Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU);
		cov.setZero ();
		Eigen::Matrix3d U = svd.matrixU ();
		// Reconstitute the covariance matrix with modified singular values using the column     // vectors in V.
		for(int k = 0; k < 3; k++) {
			Eigen::Vector3d col = U.col(k);
			double v = 1.; // biggest 2 singular values replaced by 1
			if(k == 2)   // smallest singular value replaced by gicp_epsilon
			{
				v = epsilon;
				normal_out = col ;
			}
			cov+= v * col * col.transpose(); 
		}
		
		cov_out = cov ;
	}
	
	void down_sample(pcl::PointCloud<T>& cloud, const pcl::ModelCoefficients& plane) 
	{
		std::sort(cloud.begin(), cloud.end(), LidarPlanes::comparator_distance) ;
		size_t max_index = floor(cloud.size() * max_distance_ratio_) ;
		max_index = max_index < cloud.size() ? max_index : (cloud.size()-1) ;
		cloud.erase(cloud.begin()+max_index, cloud.end()) ; 
		
		if(max_plane_points_ > 0)
		{
			pcl::PointCloud<T> cloud_out ;
			float x_boundary_min = 10e10;
			float x_boundary_max = -10e10;
			float y_boundary_min = 10e10; 
			float y_boundary_max = -10e10; 	
			float z_boundary_min = 10e10;
			float z_boundary_max = -10e10;
			
			for(size_t i=0; i<cloud.size(); ++i)
			{
				if(cloud.at(i).x > x_boundary_max)
					x_boundary_max = cloud.at(i).x;
				if(cloud.at(i).x < x_boundary_min)
					x_boundary_min = cloud.at(i).x;
				if(cloud.at(i).y > y_boundary_max)
					y_boundary_max = cloud.at(i).y;
				if(cloud.at(i).y < y_boundary_min)
					y_boundary_min = cloud.at(i).y;
				if(cloud.at(i).z > z_boundary_max)
					z_boundary_max = cloud.at(i).z;
				if(cloud.at(i).z < z_boundary_min)
					z_boundary_min = cloud.at(i).z;
			}
			int stepnums = pow(max_plane_points_,1.0/2);
			float x_steps = (x_boundary_max - x_boundary_min)/stepnums;
			float y_steps = (y_boundary_max - y_boundary_min)/stepnums;
			float z_steps = (z_boundary_max - z_boundary_min)/stepnums;
			VoxelGridNotMean< T > sor;
			sor.setInputCloud (cloud.makeShared());
			sor.setLeafSize (x_steps, y_steps, z_steps);
			sor.filter (cloud_out);
			cloud.swap(cloud_out) ;
			cout << cloud.size() << endl;
		}
		else
		{
			float m_voxelGridleafseiz = 0.2;
			VoxelGridNotMean<T> sor;
			sor.setLeafSize (m_voxelGridleafseiz, m_voxelGridleafseiz, m_voxelGridleafseiz);
			sor.setInputCloud (cloud.makeShared());
			sor.filter (cloud);
		}
	}
	
public:
	double distance_threshold_ ;
	double min_inliers_ratio_ ;    
	double min_residual_ratio_ ;
	double min_normal_diff_ ;
	double max_search_radius_ ;
	int max_plane_points_ ;
	double max_distance_ratio_ ;
	bool show_cloud_orig_ ;    
	bool show_ ;
	double incorrect_normal_ratio_;
	bool manual_select_;
	boost::shared_ptr<boost::thread> mThread ;
	bool threadbegin_;
} ;



#endif
