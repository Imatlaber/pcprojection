/* Copyright (c) AutoNavi - All Rights Reserved 
 * Author: shuo <shuo@alibaba-inc.com>
 * Description: customized segmentation class based on pcl
 */

#ifndef _PCL_VIEWER_HPP_
#define _PCL_VIEWER_HPP_

#include <vector>
#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>


namespace had
{
    
    class PCLViewer 
    {
        public:

        typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerPtr ;

        PCLViewer(std::string name = "pcl_viewer")
        {
            viewer_ = ViewerPtr(new pcl::visualization::PCLVisualizer(name)) ;
        }

        virtual ~ PCLViewer() 
        {
        }

        virtual void initialize()
        {
            viewer_->setBackgroundColor(0, 0, 0) ;
			viewer_->setSize(760,480);
            viewer_->addCoordinateSystem(1.0) ;
            viewer_->initCameraParameters() ;
        }

        template<typename T>
        void add_cloud_to_viewer(const typename pcl::PointCloud<T>::ConstPtr cloud,
                                 std::array<int, 3> rgb,
                                 std::string cloud_name)
        {
            pcl::visualization::PointCloudColorHandlerCustom<T> color(cloud, rgb[0], rgb[1], rgb[2]) ;
            if(cloud_lookup_.find(cloud_name) == cloud_lookup_.end())
            {
                viewer_->addPointCloud<T>(cloud, color, cloud_name) ;
                cloud_lookup_[cloud_name] = 1 ;
            }
            else
                viewer_->updatePointCloud<T>(cloud, color, cloud_name) ;

            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name) ;
        }
   
        template<typename T1, typename T2>
        void add_arrow(const T1& pt1,
                       const T2& pt2,
                       std::array<int,3> rgb,
                       const std::string& id = "arrow")
        {
            viewer_->addArrow(pt1, pt2, rgb[0]/255.0, rgb[1]/255.0, rgb[2]/255.0, 0, id) ;
        }

        template<typename T1, typename T2>
        void add_line( const T1& pt1,
                       const T2& pt2,
                       std::array<int,3> rgb,
                       const std::string& id = "line")
        {
            viewer_->addLine(pt1, pt2, rgb[0]/255.0, rgb[1]/255.0, rgb[2]/255.0, id) ;
        }

        void spin_once()
        {
              viewer_->spinOnce(100) ;
              boost::this_thread::sleep(boost::posix_time::microseconds(100000)) ;
        }

        virtual void show() 
        {
            while(!viewer_->wasStopped())
            {
              viewer_->spinOnce(100) ;
              boost::this_thread::sleep(boost::posix_time::microseconds(1000)) ;
            }
        }

        public:

        ViewerPtr viewer_ ;
        std::map<std::string, int> cloud_lookup_ ;        

    } ;













}

#endif // _PCL_VIEWER_HPP_
