#include <vector>
#include <array>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>
#include <iostream>
#include <string>
#include <stdint.h>
#include "utilities/drivers/velodyne/pcl_point_types.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <lcm/lcm-cpp.hpp>
#include "utilities/hadif/param.hpp"
#include "utilities/hadif/trans.hpp"
#include "had_calib.h"
#include "utilities/hadif/math_helper.hpp"
#include "common/pcl_viewer/pcl_viewer.hpp"
#include "perception/lidar_features/lidar_feature_plane.hpp"
#include "common/adapters/odom_adapter.hpp"
#include <opencv2/opencv.hpp>

bool _export_mode = false;

struct VeloPoint
{
    VeloPoint():
        x(0),
        y(0),
        z(0),
        a(0),
        d(0)
    {}

    double x ;
    double y ;
    double z ;
    double a ; // azimuth
    double d ; // distance
    int    r ; // ring
    int64_t utime ; //timestamp
} ;

typedef std::array<Eigen::Vector3d, 32> VeloFiring ;

typedef std::vector<VeloFiring> VeloRev ;
    
struct Box
{
    std::array<Eigen::Vector3d, 4> corners ;
    Eigen::Vector3d normal ; 
} ;

typedef std::vector<Box> Scene ;

typedef std::vector<Eigen::Matrix4d> Trajectory ;

typedef std::map<std::string, double> CalibParam ;

typedef std::map<std::string, double> TrajParam ;

typedef std::map<std::string, double> GroundTruth ;

class VeloSim
{
    public:
    
    VeloSim():
        azimuth_step_(1)
    {
        rev_size_ = floor(360.0/azimuth_step_) ;
    }

	double azimuthStep() const 
	{
		return azimuth_step_ ;
	}

	double revSize() const 
	{
		return rev_size_ ;
	}

	Eigen::Matrix4d populatePose(const Eigen::Matrix4d& start,
					  double& theta,
					  double& x,
					  double& y, 
					  double& w,
					  double dt,
					  const TrajParam& traj_params)
    {
        /// use dubin's vehicel model to simulate trajectory
        /// constant speed with varying heading 
        double factor = M_PI/180.0 ;
        double r = traj_params.at("radius") ; // turning radius
        double speed = fabs(r * w) ; // m/s
        double noise_x = 0 ;
        double noise_y = 0 ;
        double noise_z = 0 ;
        double noise_roll = 0 ;
        double noise_pitch = 0 ;
        double noise_yaw = 0 ;
		if(fabs(dt) > 1e-10)
		{
			double max_turning_angle = traj_params.at("max_turning_angle") * factor;
        	// flip rotation direction if out of boundary
        	if(theta < - max_turning_angle || theta > max_turning_angle)
            {
                //printf(">>>>>>>>>>>>>>> w was %f; theta %f, max_turning_angle %f \n", w, theta, max_turning_angle) ;
        	    w = -w ;
                //printf("<<<<<<<<<<<<<<< w is %f; theta %f, max_turning_angle %f\n", w, theta, max_turning_angle) ;
            }
        	/// assuming constant velocity within each interval 
        	double dtheta = w * dt ;
        	theta += dtheta ;
            //printf("******************theta is %f \n", theta) ;
        	double vx = speed * cos(M_PI/2.0+theta) ;
        	double vy = speed * sin(M_PI/2.0+theta) ;
        	double dx = vx * dt ;
        	double dy = vy * dt ;
        	x += dx ;
        	y += dy ;
        	if((*traj_params_)["traj_noise"])
        	{
        	    double x_sigma = 0.014 ;
        	    double y_sigma = 0.014 ;
        	    double z_sigma = 0.05 ;
        	    double roll_sigma = 0.015*factor ;
        	    double pitch_sigma = 0.015*factor ;
        	    double yaw_sigma = 0.05*factor ;
        	    std::normal_distribution<double> noise_x_gauss(0.0, x_sigma) ;
        	    std::normal_distribution<double> noise_y_gauss(0.0, y_sigma) ;
        	    std::normal_distribution<double> noise_z_gauss(0.0, z_sigma) ;
        	    std::normal_distribution<double> noise_roll_gauss(0.0, roll_sigma) ;
        	    std::normal_distribution<double> noise_pitch_gauss(0.0, pitch_sigma) ;
        	    std::normal_distribution<double> noise_yaw_gauss(0.0, yaw_sigma) ;
        	    noise_x = noise_x_gauss(gen_real_) ;
        	    noise_y = noise_y_gauss(gen_real_) ;
        	    noise_z = noise_z_gauss(gen_real_) ;
        	    noise_roll = noise_roll_gauss(gen_real_) ;
        	    noise_pitch = noise_pitch_gauss(gen_real_) ;
        	    noise_yaw = noise_yaw_gauss(gen_real_) ;
        	}
		}
        Eigen::Matrix4d curr_trans = Eigen::Matrix4d::Identity() ;
        curr_trans.topLeftCorner(3,3) = (Eigen::AngleAxisd(theta+noise_yaw, Eigen::Vector3d::UnitZ())
                                       *Eigen::AngleAxisd(noise_pitch, Eigen::Vector3d::UnitY())
                                       *Eigen::AngleAxisd(noise_roll, Eigen::Vector3d::UnitX())).matrix() ;
        curr_trans.topRightCorner(3,1) = Eigen::Vector3d(x+noise_x, y+noise_y, noise_z) ;
			
		Eigen::Matrix4d result = start * curr_trans ;
		return result ;
    }
    
	void populateTraj(const Eigen::Matrix4d& start, const TrajParam& traj_params, Trajectory& traj)
    {
        traj.clear() ;
        
        /// use dubin's vehicel model to simulate trajectory
        /// constant speed with varying heading 
        
        double factor = M_PI/180.0 ;
        double dt = traj_params.at("sampling_interval") ; // sampling interval
        double w = traj_params.at("angular_speed") * factor ; // angular speed rad/s
        double r = traj_params.at("radius") ; // turning radius
        double speed = fabs(r * w) ; // m/s
        double theta = 0. ; // starting heading
        double x = 0. ; // starting x position
        double y = 0. ; // starting y position 
        double noise_x = 0 ;
        double noise_y = 0 ;
        double noise_z = 0 ;
        double noise_roll = 0 ;
        double noise_pitch = 0 ;
        double noise_yaw = 0 ;
        Eigen::Matrix4d trans = start ;
        //traj.push_back(trans) ;
        int N = traj_params.at("number_intervals") ;
        double max_turning_angle = traj_params.at("max_turning_angle") * factor;
        for(int i=0; i<N; ++i)
        {
            // flip rotation direction if out of boundary
            if(theta < - max_turning_angle || theta > max_turning_angle)
                w = -w ;

            /// assuming constant velocity within each interval 
            double dtheta = w * dt ;
            theta += dtheta ;
            double vx = speed * cos(M_PI/2.0+theta) ;
            double vy = speed * sin(M_PI/2.0+theta) ;
            double dx = vx * dt ;
            double dy = vy * dt ;
            x += dx ;
            y += dy ;
            if((*traj_params_)["traj_noise"])
            {
                double x_sigma = 0.014 ;
                double y_sigma = 0.014 ;
                double z_sigma = 0.05 ;
                double roll_sigma = 0.015*factor ;
                double pitch_sigma = 0.015*factor ;
                double yaw_sigma = 0.05*factor ;
                std::normal_distribution<double> noise_x_gauss(0.0, x_sigma) ;
                std::normal_distribution<double> noise_y_gauss(0.0, y_sigma) ;
                std::normal_distribution<double> noise_z_gauss(0.0, z_sigma) ;
                std::normal_distribution<double> noise_roll_gauss(0.0, roll_sigma) ;
                std::normal_distribution<double> noise_pitch_gauss(0.0, pitch_sigma) ;
                std::normal_distribution<double> noise_yaw_gauss(0.0, yaw_sigma) ;
                noise_x = noise_x_gauss(gen_real_) ;
                noise_y = noise_y_gauss(gen_real_) ;
                noise_z = noise_z_gauss(gen_real_) ;
                noise_roll = noise_roll_gauss(gen_real_) ;
                noise_pitch = noise_pitch_gauss(gen_real_) ;
                noise_yaw = noise_yaw_gauss(gen_real_) ;
            }
            Eigen::Matrix4d curr_trans = Eigen::Matrix4d::Identity() ;
            curr_trans.topLeftCorner(3,3) = (Eigen::AngleAxisd(theta+noise_yaw, Eigen::Vector3d::UnitZ())
                                           *Eigen::AngleAxisd(noise_pitch, Eigen::Vector3d::UnitY())
                                           *Eigen::AngleAxisd(noise_roll, Eigen::Vector3d::UnitX())).matrix() ;
            curr_trans.topRightCorner(3,1) = Eigen::Vector3d(x+noise_x, y+noise_y, noise_z) ;
            trans  = start * curr_trans ;
           
            //double vx_body = speed * cos(M_PI/2.0+dtheta) ;
            //double vy_body = speed * sin(M_PI/2.0+dtheta) ;
            //double dx_body = vx_body * dt ;
            //double dy_body = vy_body * dt ;
            //Eigen::Matrix4d dtrans = Eigen::Matrix4d::Identity() ;
            //dtrans.topLeftCorner(3,3) = Eigen::AngleAxisd(dtheta, Eigen::Vector3d::UnitZ()).matrix() ;
            //dtrans.topRightCorner(3,1) = Eigen::Vector3d(dx_body, dy_body, 0) ;
            //trans *= dtrans ;
            
            traj.push_back(trans) ;
            //printf("trajecftory point: x %g y %g theta %g \n", x, y, theta) ;
            //printf("trans point: x %g y %g \n", trans(0,3), trans(1,3)) ;
        }
    }

    void popuateVeloFiring(VeloFiring& firing, Eigen::Matrix3d * rot = nullptr)
    {
        static const std::array<double, 32> arr = {{-30.67, -9.33, -29.33, -8.00,
                                                   -28.00, -6.66, -26.66, -5.33,
                                                   -25.33, -4.00, -24.00, -2.67,
                                                   -22.67, -1.33, -21.33, 0.00,
                                                   -20.00, 1.33, -18.67, 2.67,
                                                   -17.33, 4.00, -16.00, 5.33,
                                                   -14.67, 6.67, -13.33, 8.00,
                                                   -12.00, 9.33, -10.67, 10.67}};
    
        for(size_t i=0; i<arr.size(); ++i)
        {
            //printf("array %zu ; %g \n", i, arr[i]) ;
            double ele = arr[i]*M_PI/180.0 ;
            Eigen::Vector3d ray(cos(ele), 0., sin(ele)) ;
			if(rot)
			{
				firing[i] = (*rot) * ray ;
			}
			else
				firing[i] = ray ;
        }
    }
    
    void populateVeloRev(VeloRev& rev)
    {
        VeloFiring firing0 ;
        popuateVeloFiring(firing0) ;
    
        int N = floor(360.0/azimuth_step_) ;
        rev.resize(N) ;
      
        for(int i=0; i<N; ++i)
        {
            //std::cout << "------------firing------- " << i << std::endl ;
            double azimuth = i*azimuth_step_*M_PI/180.0 ;
            Eigen::Matrix3d rot = Eigen::AngleAxisd(azimuth, Eigen::Vector3d::UnitZ()).matrix() ;
            auto& firing = rev[i] ;
            for(size_t j=0; j<firing0.size(); ++j)
            {
                firing[j] = rot * firing0[j] ;
                //std::cout << "ring " << j << std::endl ;
                //std::cout << firing[j] << std::endl ;
            }
        }
        
    } ;
    
    void rayTracing(const Box& box, 
                    double azimuth,
                    size_t ring,
                    const Eigen::Vector3d& ray, 
                    VeloPoint& point)
    {
        /// plane: (p-p0).n=0   p0: point on plane p, n: plane normal 
        /// line: p = d*l + l0  l0: point on line p   l: line direction 
        /// d = (p0-l0).n/l.n   if l.n == 0 then line is parallel to the plane 
        /// see https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
       
        //printf("ray %10g %10g %10g \n", ray[0], ray[1], ray[2]) ;

        double denom = ray.dot(box.normal) ;
        //printf("denom %10g \n", denom) ;
        if(fabs(denom) < 1e-3)
        {
    
            //printf("no intersection: line and palen are parallel  \n") ;
            return ;
        }
    
        double d = box.corners[0].dot(box.normal)/denom ;
        //printf("d %10g \n", d) ;
        /// d needs to be positive as it is the magitude in the line direction
        if(d < 0)
        {
           // printf("no intersection: distance is negative \n") ;
           return ; 
        }
    
        Eigen::Vector3d intersection = d*ray ;
        /// check if intersecion is inside the box
        if(!insideBox(box, intersection))
        {
            //printf("no intersection : outside box \n") ;
            return ;
        }
        /// if point.d != 0 then point is already a intersection. Choose the closest one
        if(point.d > 1e-3)
        {
            if(d > point.d)
            {
                return ;
            }
        }
 
        if((*traj_params_)["cloud_noise"])
        {
            double factor = M_PI/180.0 ;
            //printf("<<<< fonnd intersection ! %g %g %g \n", intersection[0], intersection[1], intersection[2]) ;
            double ele = atan2(intersection[2], sqrt(intersection[0]*intersection[0]+intersection[1]*intersection[1])) ;
            double ele_sigma = 0.133 * factor ; 
            double azi_sigma = 0.016 * factor ; 
            double dis_sigma = d * 0.02/25.0 ; 
            std::normal_distribution<double> noise_ele_gauss(0.0, ele_sigma) ; 
            std::normal_distribution<double> noise_azi_gauss(0.0, azi_sigma) ; 
            std::normal_distribution<double> noise_dis_gauss(0.0, dis_sigma) ; 
            double noise_ele = noise_ele_gauss(gen_real_) ;
            double noise_azi = noise_azi_gauss(gen_real_) ;
            double noise_dis = noise_dis_gauss(gen_real_) ;
            point.x = (d+noise_dis)*cos(ele+noise_ele)*cos(azimuth+noise_azi) ;
            point.y = (d+noise_dis)*cos(ele+noise_ele)*sin(azimuth+noise_azi) ;
            point.z = (d+noise_dis)*sin(ele+noise_ele) ;
        }
        else
        {
            point.x = intersection[0] ;
            point.y = intersection[1] ;
            point.z = intersection[2] ;
        }
        point.a = azimuth ;
        point.d = d ;
        point.r = (int)ring ;
    
    }
   
    bool insideBox(const Box& box, const Eigen::Vector3d& point)
    {
        bool inside = true ;
        double angle = 0. ;
        //printf("intersection %g %g %g \n", point[0], point[1], point[2]) ;

        for(size_t i=0; i<4; ++i)
        {
            size_t j = (i+1)%4 ;
            //printf("intersection %g %g %g \n", box.corners[i][0], box.corners[i][1], box.corners[i][2]) ;
            Eigen::Vector3d vi = box.corners[i]-point ;
            Eigen::Vector3d vj = box.corners[j]-point ;
            //printf("vi %g %g %g \n", vi[0], vi[1], vi[2]) ;
            //printf("vj %g %g %g \n", vj[0], vj[1], vj[2]) ;
            double angle_i = angleBtwVectors3d(vi.data(), vj.data()) ; 
            //printf("angle %g \n", angle_i) ;
            angle += angle_i ;
            
        }
        
        //printf("angle %g \n", angle) ;

        if(fabs(angle - M_PI*2) > 1e-3)
            inside = false ;

        return inside ;
    }

    void rayTracing(const Scene& scene, 
                    double azimuth,
                    size_t ring,
                    const Eigen::Vector3d& ray, 
                    VeloPoint& point)
    {
        for(const auto& box : scene)
        {
            rayTracing(box, azimuth, ring, ray, point) ;        
        }
    }
    
    void simCloudRev(const VeloRev& rev, const Scene& scene, std::vector<VeloPoint>& cloud)
    {
        int N = rev.size() * 32 ;
        cloud.clear() ;
        cloud.resize(N) ;
    
        for(size_t i=0; i<rev.size(); ++i)
        { 
            double azimuth = i*azimuth_step_ * M_PI/180.0 ;
            for(size_t j=0; j<rev[i].size(); ++j)
            {
                size_t index = i*rev[i].size()+j ;        
                auto& point = cloud[index] ;
                point.d = 0 ;
                rayTracing(scene, azimuth, j, rev[i][j], point) ; 
            }
        }
    }

    void simCloudFiring(const VeloFiring& firing, double azimuth, const Scene& scene, std::vector<VeloPoint>& cloud_firing, int64_t utime)
    {
        int N = firing.size();
        cloud_firing.clear() ;
        cloud_firing.resize(N) ;
    
        for(int j=0; j<N; ++j)
        {
            auto& point = cloud_firing[j] ;
            point.d = 0 ;
            point.utime = utime ;
            rayTracing(scene, azimuth, j, firing[j], point) ; 
        }
    }

    void transformCloud(const Eigen::Matrix4d& trans_b2w, 
                        const Eigen::Matrix4d& trans_l2b, 
                        const pcl::PointCloud<PCLVelodyne>& cloud_l,
                        pcl::PointCloud<PCLVelodyne>& cloud_w)
    {
        Eigen::Matrix4d trans_l2w = trans_b2w * trans_l2b ;
        pcl::transformPointCloud(cloud_l, cloud_w, trans_l2w) ;
    }

    void transformScene(const Eigen::Matrix4d& trans_b2w, 
                        const Eigen::Matrix4d& trans_l2b, 
                        Scene& scene)
    {
        Eigen::Matrix4d trans_l2w = trans_b2w * trans_l2b ;
        Eigen::Matrix4d trans_w2l = trans_l2w.inverse() ;

        for(auto& box : scene)
        {
            
            transformBox(trans_w2l, box) ;
        }

    }

    void transformBox(const Eigen::Matrix4d& trans_w2l,
                      Box& box)
    {
        for(auto& corner : box.corners)
        {
            corner = trans_w2l.topLeftCorner(3,3)*corner + trans_w2l.topRightCorner(3,1) ;
        }
        
        box.normal = trans_w2l.topLeftCorner(3,3)*box.normal ;
    }

    void printScene(const Scene& scene)
    {
        for(size_t i=0; i<scene.size(); ++i)
        {
            printf("--------------- plane %zu -------------\n", i) ;
            printBox(scene[i]) ;
        }
    }

    void printBox(const Box& box)
    {
        for(size_t i=0; i<box.corners.size(); ++i)
        {
            printf("corner %zu: %10g %10g %10g \n", i, box.corners[i][0], box.corners[i][1], box.corners[i][2]) ;
        }
        printf("normal %10g %10g %10g \n", box.normal[0], box.normal[1], box.normal[2]) ;
    }

    void exportParams(const CalibParam& params, 
                      const GroundTruth& gt,
                      const GroundTruth& guess,
                      const TrajParam& traj_params,
                      const Scene& scene,
                      std::string fileName = "param.yml")
    {
        cv::FileStorage fs(fileName, cv::FileStorage::WRITE) ;

        fs << "export_mode" << 0 ;

        cvWriteComment(*fs, "======================== Ground Truth =========================", 0) ; 

        for(const auto& param : gt)
        {
            fs << param.first << param.second ;
        }
        
        cvWriteComment(*fs, "======================== Initial Guess =========================", 0) ; 

        for(const auto& param : guess)
        {
            fs << param.first << param.second ;
        }
        
        cvWriteComment(*fs, "======================== Calibration Parameters ================", 0) ; 

        for(const auto& param : params)
        {
            fs << param.first << param.second ;
        }
        
        cvWriteComment(*fs, "======================== Trajectory Parameters ==================", 0) ; 

        for(const auto& param : traj_params)
        {
            fs << param.first << param.second ;
        }
        
        cvWriteComment(*fs, "======================== Scene Parameters  =======================", 0) ; 

        fs << "scene" << "[" ;
        for(const auto& box : scene)
        {
            cv::Mat mat = (cv::Mat_<double>(4,3) << box.corners[0][0], box.corners[0][1], box.corners[0][2],  
                                                    box.corners[1][0], box.corners[1][1], box.corners[1][2],
                                                    box.corners[2][0], box.corners[2][1], box.corners[2][2],
                                                    box.corners[3][0], box.corners[3][1], box.corners[3][2]) ;

            fs << "{" ;
            fs << "mat" << mat ;
            fs << "}" ;
        }

        fs << "]" ;
    }

    void readParams(std::string fileName, 
                    GroundTruth& gt,
                    GroundTruth& guess,
                    CalibParam& calib_params,
                    TrajParam& traj_params, 
                    Scene& scene)
    {
        calib_params_ = &calib_params ;
        traj_params_ = &traj_params ;

        cv::FileStorage fs(fileName, cv::FileStorage::READ) ;
        fs["export_mode"] >> _export_mode ;
        
        /// read ground truth
        fs["x"] >> gt["x"] ;
        fs["y"] >> gt["y"] ;
        fs["z"] >> gt["z"] ;
        fs["roll"] >> gt["roll"] ;
        fs["pitch"] >> gt["pitch"] ;
        fs["yaw"] >> gt["yaw"] ;

        /// read initial guess
        fs["x_init"] >> guess["x_init"] ;
        fs["y_init"] >> guess["y_init"] ;
        fs["z_init"] >> guess["z_init"] ;
        fs["roll_init"] >> guess["roll_init"] ;
        fs["pitch_init"] >> guess["pitch_init"] ;
        fs["yaw_init"] >> guess["yaw_init"] ;

        /// read calibration param
        fs["calib_limit"] >> calib_params["calib_limit"] ;
        fs["gicp_epsilon"] >> calib_params["gicp_epsilon"] ;
        fs["max_normal_diff"] >> calib_params["max_normal_diff"] ;
        fs["max_correspondence_distance"] >> calib_params["max_correspondence_distance"] ;
        fs["max_nearest_search"] >> calib_params["max_nearest_search"] ;
        fs["correspondence_randomness"] >> calib_params["correspondence_randomness"] ;
        fs["rotation_epsilon"] >> calib_params["rotation_epsilon"] ;
        fs["translation_epsilon"] >> calib_params["translation_epsilon"] ;
        fs["max_iteration"] >> calib_params["max_iteration"] ;
        fs["max_convergence_error"] >> calib_params["max_convergence_error"] ;
        fs["rpy_only"] >> calib_params["rpy_only"] ;
        fs["show_cloud_lidar"] >> calib_params["show_cloud_lidar"] ;
        fs["show_cloud_world"] >> calib_params["show_cloud_world"] ;
        fs["external_cov"] >> calib_params["external_cov"] ;
         
        /// read trajectory params
        fs["x0"] >> traj_params["x0"] ;
        fs["y0"] >> traj_params["y0"] ;
        fs["z0"] >> traj_params["z0"] ;
        fs["roll0"] >> traj_params["roll0"] ;
        fs["pitch0"] >> traj_params["pitch0"] ;
        fs["yaw0"] >> traj_params["yaw0"] ;
        fs["sampling_interval"] >> traj_params["sampling_interval"] ;
        fs["angular_speed"] >> traj_params["angular_speed"] ;
        fs["radius"] >> traj_params["radius"] ;
        fs["max_turning_angle"] >> traj_params["max_turning_angle"] ; 
        fs["number_intervals"] >> traj_params["number_intervals"]  ; 
        fs["cloud_noise"] >> traj_params["cloud_noise"] ;
        fs["traj_noise"] >> traj_params["traj_noise"] ;
        fs["moving_within_rev"] >> traj_params["moving_within_rev"] ;
        
        /// read scene params
        cv::FileNode scene_fn = fs["scene"] ;
        printf("scene size %zu \n", scene_fn.size()) ;
        for(cv::FileNodeIterator it=scene_fn.begin(); it != scene_fn.end(); ++it)
        {
            cv::Mat mat ;
            (*it)["mat"] >> mat ;
            Box box ;
            box.corners[0] = Eigen::Vector3d(mat.at<double>(0,0), mat.at<double>(0,1), mat.at<double>(0,2)) ;
            box.corners[1] = Eigen::Vector3d(mat.at<double>(1,0), mat.at<double>(1,1), mat.at<double>(1,2)) ;
            box.corners[2] = Eigen::Vector3d(mat.at<double>(2,0), mat.at<double>(2,1), mat.at<double>(2,2)) ;
            box.corners[3] = Eigen::Vector3d(mat.at<double>(3,0), mat.at<double>(3,1), mat.at<double>(3,2)) ;
            Eigen::Vector3d v1 = box.corners[1]-box.corners[0] ;
            Eigen::Vector3d v2 = box.corners[3]-box.corners[0] ;
            Eigen::Vector3d normal = cross(v1, v2) ;
            normal = normalize3d(normal) ;
            box.normal = normal ;
            scene.push_back(box) ;
        }
    }


    private:
    std::default_random_engine gen_real_ ;

    public:
    double azimuth_step_ ;
    CalibParam * calib_params_ ;
    TrajParam  * traj_params_ ;
	int rev_size_ ;    
} ;

int main(int argc, char * argv[])
{
    std::string paramName = "param.yml";
    if(argc > 1)
        paramName = std::string(argv[1]) ;    
    else
    {
        printf("usage: %s param_file \n", argv[0]) ;
        return 0 ;
    }

    double factor = M_PI/180.0 ;
    
    /// create velodyne simulator
    VeloSim sim ;

    /// populate velodyne rays in one revolution
    VeloRev rev ;
    sim.populateVeloRev(rev) ;
  
    /// load parameters from file
    Scene scene ; 
    GroundTruth gt ;
    GroundTruth guess ;
    CalibParam calib_params ;
    TrajParam traj_params ; 
    sim.readParams(paramName, gt, guess, calib_params, traj_params, scene) ;

    /// write to a temporary file for checking
    std::string paramNameTmp = paramName + ".exported" ;
    sim.exportParams(calib_params, gt, guess, traj_params, scene, paramNameTmp) ;
    
    if(_export_mode) 
    {
        sim.exportParams(calib_params, gt, guess, traj_params, scene, paramName) ;
        return 0 ;
    }

    /// populate trajectory
    Eigen::Matrix4d pose0 = Eigen::Matrix4d::Identity() ;
    double roll0 = traj_params["roll0"] * factor ;
    double pitch0 = traj_params["pitch0"] * factor ;
    double yaw0 = traj_params["yaw0"] * factor ;
    pose0.topLeftCorner(3,3) = (Eigen::AngleAxisd(yaw0, Eigen::Vector3d::UnitZ()) 
                                *Eigen::AngleAxisd(pitch0, Eigen::Vector3d::UnitY())
                                *Eigen::AngleAxisd(roll0, Eigen::Vector3d::UnitX())).matrix() ;
    pose0.topRightCorner(3,1) = Eigen::Vector3d(traj_params["x0"], traj_params["y0"], traj_params["z0"]) ;

    pcl::PointCloud<PCLVelodyne> traj_cloud ;
    
    /// populate ground truth
    Eigen::Matrix<double, 6, 1> true_xyzrpy ;
    true_xyzrpy << gt["x"], gt["y"], gt["z"], gt["roll"]*factor, gt["pitch"]*factor, gt["yaw"]*factor ;
    /// set static lidar to body transformation
    Eigen::Matrix4d trans_l2b = Eigen::Matrix4d::Identity() ; // lidar to body 
    //trans_l2b.topLeftCorner(3,3) = (Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-M_PI/6.0, Eigen::Vector3d::UnitX())).matrix() ;
    trans_l2b.topLeftCorner(3,3) = (Eigen::AngleAxisd(true_xyzrpy(5), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(true_xyzrpy(4), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(true_xyzrpy(3), Eigen::Vector3d::UnitX())).matrix() ;
    trans_l2b.topRightCorner(3,1) = true_xyzrpy.head<3>() ;

    /// populate initial guess
    Eigen::Matrix4f init = Eigen::Matrix4f::Identity() ;
    Eigen::Matrix<float, 6, 1> guess_xyzrpy ;
    guess_xyzrpy << guess["x_init"], guess["y_init"], guess["z_init"], guess["roll_init"]*factor, guess["pitch_init"]*factor, guess["yaw_init"]*factor ;
    init.topLeftCorner(3,3) = (Eigen::AngleAxisf(guess_xyzrpy(5), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(guess_xyzrpy(4), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(guess_xyzrpy(3), Eigen::Vector3f::UnitX())).matrix() ;
    init.topRightCorner(3,1) = guess_xyzrpy.head<3>() ;

    /// create viewers
    boost::shared_ptr<had::PCLViewer> viewer_l, viewer_w ;
    if(calib_params["show_cloud_lidar"])
    {
        viewer_l = boost::shared_ptr<had::PCLViewer>(new had::PCLViewer) ;
        viewer_l->initialize() ;
    }
    if(calib_params["show_cloud_world"])
    {
        viewer_w = boost::shared_ptr<had::PCLViewer>(new had::PCLViewer) ;
        viewer_w->initialize() ;
    }
    std::array<int,3> color{{255, 0, 0}} ;
    std::array<int,3> color_traj{{0, 255, 0}} ;
   
    /// create feature extractors
    lcm::LCM lcm ;
    param_t param(&lcm) ;
    trans_t trans(lcm.getUnderlyingLCM(), &param) ;
    LidarUnorgPlane<PCLVelodyne> feat_plane(&lcm, &trans, "features.plane") ;

    /// create calibration solver 
    had::CalibGeneralizedIterativeClosestPoint<PCLVelodyne, PCLVelodyne> calib(&lcm, &param, &trans) ;
    /// choose to use scan time or point time 
    calib.setUsePointTime(true) ;
    /// full 6DPF estimate or roll/pitch/yaw estimate only
    calib.setRpyOnly(calib_params["rpy_only"]) ;
    /// correspondence parameters
    calib.setMaxNormalDiffSamePlane(calib_params["max_normal_diff"]) ;
    calib.setMaxCorrespondenceDistance(calib_params["max_correspondence_distance"]) ;
    calib.setMaxNearestSearchPoints(calib_params["max_nearest_search"]) ;
    /// covariance computation parameters
    calib.setCorrespondenceRandomness(calib_params["correspondence_randomness"]) ;
    calib.setGicpEpsilon(calib_params["gicp_epsilon"]) ;
    /// convergence parameters
    calib.setRotationEpsilon(calib_params["rotation_epsilon"]) ;
    calib.setTransformationEpsilon(calib_params["translation_epsilon"]) ;
	calib.setMaxConvergenceError(calib_params["max_convergence_error"]) ;
    calib.setMaximumIterations(calib_params["max_iteration"]) ;

    /// generate point cloud 
    int cnt = 0, calib_cnt=0, interval = 10 ;
    //printf("number of poses %zu \n", traj.size()) ;	
    int N = traj_params["number_intervals"] ;
    double dt_rev = traj_params.at("sampling_interval") ; // sampling interval
    double c_w = traj_params.at("angular_speed") * factor ; // angular speed rad/s
	double c_theta=0., c_x=0., c_y=0. ;
	Eigen::Matrix4d pose ;
    double tstamp=0., dtstamp=dt_rev/sim.revSize() ; 
    Trajectory traj ;
    std::vector<double> traj_tstamps ;

    for(int i=0; i<N; ++i)
    {
        ++cnt ;
		//pose = traj[i] ;
		std::cout << "---------------------" << std::endl ;
		std::cout << "Pose " << i << std::endl ;
		std::cout << pose << std::endl ;
        std::vector<VeloPoint> cloud ;
        //sim.simCloudRev(rev, scene_l, cloud) ;
		for(int j=0; j<sim.revSize(); ++j)
		{
            /// compute time stamp
            tstamp += dtstamp ;
            traj_tstamps.push_back(tstamp) ;
			double dt =0. ;
            if(traj_params.at("moving_within_rev"))
                dt = dt_rev/sim.revSize() ;
            else
			    dt = (j == 0 ? dt_rev : 0) ;

			pose = sim.populatePose(pose0, 
							c_theta,
							c_x,
							c_y,
							c_w,
							dt,
							traj_params) ;
            traj.push_back(pose) ;
			Scene scene_l = scene ;
			sim.transformScene(pose, trans_l2b, scene_l) ; 
			VeloFiring firing ;
			std::vector<VeloPoint> cloud_firing ;
            double azimuth = j*sim.azimuthStep()*M_PI/180.0 ;
            Eigen::Matrix3d rot = Eigen::AngleAxisd(azimuth, Eigen::Vector3d::UnitZ()).matrix() ;
			sim.popuateVeloFiring(firing, &rot) ;
			sim.simCloudFiring(firing, azimuth, scene_l, cloud_firing, tstamp*1e6) ;
			cloud.insert(cloud.end(), cloud_firing.begin(), cloud_firing.end()) ;
		}
        printf("cloud size %zu \n", cloud.size()) ;
        pcl::PointCloud<PCLVelodyne> pcl_cloud_lidar, pcl_cloud_world ;
        for(const auto& point : cloud)
        {
            if(point.d > 1e-3)
            {
                PCLVelodyne pcl_point(point.x, point.y, point.z) ;
                pcl_point.distance = point.d ;
                memcpy(&pcl_point.utime1, &point.utime, sizeof(int64_t)) ; 
                pcl_cloud_lidar.push_back(pcl_point) ;
            }
            //printf("point : %g %g %g \n", point.x, point.y, point.z) ;
        }
        std::vector<Eigen::Matrix3d> covs ;
        std::vector<Eigen::Vector3d> normals ;
        int num_planes = 0 ;
        feat_plane.extract(pcl_cloud_lidar, &covs, &normals, 1) ;
        num_planes = (int)covs.size() ;
        static pcl::PointCloud<PCLVelodyne> pcl_cloud_lidar_prev = pcl_cloud_lidar ;
        static Eigen::Matrix4d pose_prev = pose ;
        static std::vector<Eigen::Matrix3d> covs_prev = covs ;
        static std::vector<Eigen::Vector3d> normals_prev = normals ;
        static int num_planes_prev = (int)covs.size() ;
        
        if(cnt == 1)
        {
            printf("------------------ pair id %d ---------------- \n", calib_cnt) ;
            //sim.printScene(scene_l) ;
            
            continue ;
        }

        int calib_limit = calib_params["calib_limit"] ;
        printf("calib_limit %d \n", calib_limit) ;

        if(calib_cnt > calib_params["calib_limit"])
        {
            printf("<<<< calib_cnt is %d \n", calib_cnt) ;
            break ;
        }
        
        if(cnt % interval == 0)
        {
            printf("------------------ pair id %d ---------------- \n", calib_cnt) ;
            //sim.printScene(scene_l) ;
          
            int min_num_planes = 2 ;
            if(num_planes < min_num_planes || num_planes_prev < min_num_planes)
            {
                printf("*****num_planes %d or num_planes_prev %d is smaller than min_num_planes %d \n", num_planes, num_planes_prev, min_num_planes) ;
                 
            }
            else
            {
                if(calib_params["external_cov"])
                {
                    printf("[ use external covaraince ] \n") ;
                    calib.add_pair(pcl_cloud_lidar.makeShared(), 
                                  pose, 
                                  covs, 
                                  normals, 
                                  pcl_cloud_lidar_prev.makeShared(), 
                                  pose_prev, 
                                  covs_prev, 
                                  normals_prev) ;  
                }
                else
                {
                    calib.add_pair(pcl_cloud_lidar.makeShared(), 
                                  pose, 
                                  pcl_cloud_lidar_prev.makeShared(), 
                                  pose_prev) ;
                }

                pcl_cloud_lidar_prev = pcl_cloud_lidar ;
                pose_prev = pose ;
                covs_prev = covs ;
                normals_prev = normals ;
                num_planes_prev = num_planes ;
                ++calib_cnt ;
            }
        }
       
        sim.transformCloud(pose, trans_l2b, pcl_cloud_lidar, pcl_cloud_world) ; 
        traj_cloud.push_back(PCLVelodyne(pose(0,3),  pose(1,3), pose(2,3))) ;

        if(calib_params["show_cloud_lidar"])
        {
            viewer_l->add_cloud_to_viewer<PCLVelodyne>(pcl_cloud_lidar.makeShared(), color, "cloud_lidar") ;
            viewer_l->spin_once() ;     
        }
        if(calib_params["show_cloud_world"])
        {
            viewer_w->add_cloud_to_viewer<PCLVelodyne>(pcl_cloud_world.makeShared(), color, "cloud_world") ;
            viewer_w->add_cloud_to_viewer<PCLVelodyne>(traj_cloud.makeShared(), color_traj, "traj_world") ;
            viewer_w->spin_once() ;     
        }
    }

    /// create odometry object
    std::vector<OdomPoint> poses ;     
    for(size_t i=0; i<traj.size(); ++i)
    {
        const auto& trans = traj[i] ;
        double tstamp = traj_tstamps[i] ;
        OdomPoint pose ;
        pose.time = tstamp * 1e6 ;
        Eigen::Vector3d ypr = trans.topLeftCorner<3,3>().eulerAngles(2, 1, 0) ;
        Eigen::Vector3d xyz = trans.topRightCorner<3,1>() ;
        pose.pos[0] = xyz[0] ;
        pose.pos[1] = xyz[1] ;
        pose.pos[2] = xyz[2] ;
        pose.rpy[0] = ypr[2] ;
        pose.rpy[1] = ypr[1] ;
        pose.rpy[2] = ypr[0] ;
        poses.push_back(pose) ;
        //printf("**********write time stamp %ld \n",  static_cast<int64_t>(pose.time)) ;
        //std::cout << trans << std::endl ;
    }

    /// create odometry object
    OdomFrames odom_frames(&lcm) ;
    odom_frames.read_to_frames(poses, "ODOM") ;

    /// run calibration
    calib.exec(init) ; 
    Eigen::Matrix<float, 6, 1> result = calib.getFinalXYZRPY() ;
    printf("final result: x %f, y %f, z %f, roll %f, pitch %f, yaw %f \n", result[0],result[1],result[2],result[3]/factor,result[4]/factor,result[5]/factor) ;
    double error = calib.computeError(result.cast<double>(), "final solution") ;
    std::cout << "error: " << error << std::endl ;
    double true_error = calib.computeError(true_xyzrpy, "ground truth") ;
    std::cout << "true error: " << true_error << std::endl ;
    
    //calib.snoop(result.cast<double>(), "estimate") ;

    return 0 ;
}




