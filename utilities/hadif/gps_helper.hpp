/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: GPS computation utilities
 */

#include "math_helper.hpp"
#include <assert.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core_c.h>
#include <proj_api.h>

using std::sin;
using std::cos;
using std::atan2;
using std::sqrt;

struct latlonheight_t 
{
    double lat;
    double lon;
    double height ;
} ;

class GpsHelperBase
{
    public:

    virtual int compute_local_transformation(latlonheight_t llh) = 0 ;
    virtual int latlonheight2local(const double latlon[3], double xyzl[3]) = 0 ;
    virtual int local2latlonheight(const double xyzl[3], double latlon[3]) = 0 ;
    
} ;

class GpsHelperVanilla : public GpsHelperBase
{
    public:
    
    
    
    struct latlon_linearize_params_t 
    {
        double origin_latlon[3];
        double ax[3], ay[3], az[3];
        double r0[3];
        double height ;
    } ;
    
    int compute_local_transformation(latlonheight_t llh) override
    {
        compute_latlon_linearize(llh, orig_) ;

        return 0 ;
    }

    int compute_latlon_linearize(latlonheight_t llh,
                                 latlon_linearize_params_t& p) 
    {
        p.origin_latlon[0] = llh.lat;
        p.origin_latlon[1] = llh.lon;
        p.origin_latlon[2] = DEFAULT_ELEVATION;
        p.height = llh.height ;
    
        double x_axis[3] = {0};
        x_axis[0] = p.origin_latlon[0];
        x_axis[1] = p.origin_latlon[1] + 1e-4; // TODO what does the constant 1e-4 represent !?
        x_axis[2] = DEFAULT_ELEVATION;
    
        double x1[3], num, den, den_sq, tmp1;
        latlon2xyz(p.origin_latlon, p.r0);
        latlon2xyz(x_axis, x1);
        double* r0 = p.r0;
    
        den_sq = SQ(r0[0]) + SQ(r0[1]) + SQ(r0[2]);
        x1[0] -= r0[0];
        x1[1] -= r0[1];
        x1[2] -= r0[2];
        tmp1 = (x1[0] * r0[0] + x1[1] * r0[1] + x1[2] * r0[2]) / den_sq;
        x1[0] -= tmp1 * r0[0];  // order is important
        x1[1] -= tmp1 * r0[1];
        x1[2] -= tmp1 * r0[2];
        // x1 now is the vector from origin to x_axis projected onto the tangent
        // plane of the sphere at origin.
    
        num = 1.0 / sqrt(SQ(x1[0]) + SQ(x1[1]) + SQ(x1[2]));
        p.ax[0] = x1[0] * num;
        p.ax[1] = x1[1] * num;
        p.ax[2] = x1[2] * num;
    
        // az[] is used for local to global transformation
        den = sqrt(den_sq);
        num = 1.0 / den;
        p.az[0] = r0[0] * num;
        p.az[1] = r0[1] * num;
        p.az[2] = r0[2] * num;
    
        // TODO call existing cross product function
        p.ay[0] = p.az[1] * p.ax[2] - p.az[2] * p.ax[1];
        p.ay[1] = p.az[2] * p.ax[0] - p.az[0] * p.ax[2];
        p.ay[2] = p.az[0] * p.ax[1] - p.az[1] * p.ax[0];
    
        num = 1.0 / sqrt(SQ(p.ay[0]) + SQ(p.ay[1]) + SQ(p.ay[2]));
        p.ay[0] *= num;
        p.ay[1] *= num;
        p.ay[2] *= num;
    
        // printf("r0: (%f,%f,%f)\n", r0[0], r0[1], r0[2]);
        // printf("ax: (%f,%f,%f)\n", p.ax[0], p.ax[1], p.ax[2]);
        // printf("ay: (%f,%f,%f)\n", p.ay[0], p.ay[1], p.ay[2]);
        // printf("az: (%f,%f,%f)\n", p.az[0], p.az[1], p.az[2]);
        return 0;
    }
    
    int latlon2xyz(const double latlon[3], double xyz[3])
    {
        double lat = latlon[0];
        double lon = latlon[1];

        double slat, clat, slon, clon;
        double aslat, bclat, abclat, den;
    
        slat = sin(lat * DEG2RAD);
        clat = cos(lat * DEG2RAD);
        slon = sin(lon * DEG2RAD);
        clon = cos(lon * DEG2RAD);
    
        aslat = SEMI_MAJOR * slat;
        bclat = SEMI_MINOR * clat;
        abclat = SEMI_MAJOR * bclat;
        den = sqrt(aslat * aslat + bclat * bclat);
        xyz[0] = abclat * clon / den;
        xyz[1] = abclat * slon / den;
        xyz[2] = SEMI_MINOR * aslat / den ;
    
        return 0;
    }
    
    /**
     * @brief xyz2latlon_static
     * @param xyz
     * @param latlon, latlon[2] is not assigned
     */
    int xyz2latlon(const double xyz[3], double latlon[3]) 
    {
        double tmp;
        latlon[1] = atan2(xyz[1], xyz[0]) / DEG2RAD;
        tmp = sqrt(SQ(xyz[0]) + SQ(xyz[1]));
        latlon[0] = atan2(xyz[2], tmp) / DEG2RAD;
    }
    
    int latlonheight2local(const double llh[3],
                     double xyzl[3]) override
    {
        latlon2local_static(llh, orig_, xyzl) ; 
        xyzl[2] = llh[2]-orig_.height ;;

        return 0 ;
    }

    void latlon2local_static(const double latlon[3],
                             const latlon_linearize_params_t& p, 
                             double xyzl[3]) 
    {
        double xyz[3];
        latlon2xyz(latlon, xyz);
        // printf("xyz: (%f,%f,%f)\n", xyz[0], xyz[1], xyz[2]);
        // TODO reimplementation of dot product
        xyzl[0] = p.ax[0] * xyz[0] + p.ax[1] * xyz[1] + p.ax[2] * xyz[2];  // x
        xyzl[1] = p.ay[0] * xyz[0] + p.ay[1] * xyz[1] + p.ay[2] * xyz[2];  // x
        xyzl[2] = 0;
        // printf("xyzl: (%f,%f,%f)\n", xyzl[0], xyzl[1], xyzl[2]);
    }
    
    int local2latlonheight(const double xyzl[3],
                     double llh[3]) override
    {
        local2latlon_static(xyzl, orig_, llh) ;
        llh[2] = xyzl[2]+orig_.height ;

        return 0 ;
    }

    void local2latlon_static(const double xyzl[3],
                             const latlon_linearize_params_t& p, 
                             double latlon[3]) 
    {
        double xyz[3], a, b, c, alpha, alpha1, alpha2;
        double discrim, first, second, tmp;
    
        xyz[0] = p.ax[0] * xyzl[0] + p.ay[0] * xyzl[1] + p.r0[0];
        xyz[1] = p.ax[1] * xyzl[0] + p.ay[1] * xyzl[1] + p.r0[1];
        xyz[2] = p.ax[2] * xyzl[0] + p.ay[2] * xyzl[1] + p.r0[2];
    
        // TODO reimplementation of vector*scalar and vector norm
        a = SQ(p.r0[0] / SEMI_MAJOR) + SQ(p.r0[1] / SEMI_MAJOR) +
            SQ(p.r0[2] / SEMI_MINOR);
        b = -2.0 * ((p.r0[0] * xyz[0] + p.r0[1] * xyz[1]) / SQ(SEMI_MAJOR) +
                    p.r0[2] * xyz[2] / SQ(SEMI_MINOR));
        c = SQ(xyz[0] / SEMI_MAJOR) + SQ(xyz[1] / SEMI_MAJOR) +
            SQ(xyz[2] / SEMI_MINOR) - 1.0;
        discrim = SQ(b) - 4.0 * a * c;
        assert(discrim >= 0.0);
        tmp = 0.5 / a;
        first = -b * tmp;
        second = sqrt(discrim) * tmp;
        alpha1 = first + second;
        alpha2 = first - second;
        if (fabs(alpha1) < fabs(alpha2)) {
            alpha = alpha1;
        } else {
            alpha = alpha2;
        }
    
        xyz[0] -= alpha * p.r0[0];
        xyz[1] -= alpha * p.r0[1];
        xyz[2] -= alpha * p.r0[2];
    
        xyz2latlon(xyz, latlon);
    }

    protected:

    latlon_linearize_params_t orig_ ;
    // TODO DEFAULT_ELEVATION is no long term solution
    const double DEFAULT_ELEVATION = 0 ;
    const double SEMI_MAJOR         = 6378137.0;     // units: meter
    const double SEMI_MINOR         = 6356752.31414; // units: meter
    const double DEG2RAD            = M_PI / 180.0;

} ;

class GpsHelperProj : public GpsHelperBase
{
    public:

	struct double4x4
	{
		double data[16];
	} ;

	struct double4
	{
		double data[4];
	} ;

    GpsHelperProj():
        use_local_transformation_(false)
    {
        InitProjParam() ; 
        
    }

    GpsHelperProj(double trans[16]):
        use_local_transformation_(true)
    {
        InitProjParam() ; 
        memcpy(&orig_, trans, sizeof(double)*16) ;         
    }

    int InitProjParam()
    {
    	projWGS84_ = pj_init_plus(projStrWGS84_);
    	projCartesian_ = pj_init_plus(projStrCartesian_);
    	return 0;
    }

    int compute_local_transformation(latlonheight_t llh) override
    {
        if(!use_local_transformation_)
        {
            double lon = llh.lon ;
            double lat = llh.lat ;
            double h = llh.height;

            double lon_rad = lon * DEG_TO_RAD_ ;
            double lat_rad = lat * DEG_TO_RAD_ ;
            
            double vh = a_ / sqrt(1-f_*(2-f_)*sin(lat_rad)*sin(lat_rad)) ;
            
            double X_WGS84 = (vh+h)*cos(lat_rad)*cos(lon_rad) ;
            double Y_WGS84 = (vh+h)*cos(lat_rad)*sin(lon_rad) ;
            double Z_WGS84 = (vh*(1-f_*(2-f_))+h)*sin(lat_rad) ;
            
            double trans[16] = 
            { -sin(lon_rad), -cos(lon_rad)*sin(lat_rad), cos(lon_rad)*cos(lat_rad), X_WGS84,
               cos(lon_rad), -sin(lon_rad)*sin(lat_rad), sin(lon_rad)*cos(lat_rad), Y_WGS84,
               0,              cos(lat_rad),             sin(lat_rad),              Z_WGS84,
               0,              0,                        0,                         1
            } ;
        
            for(int i=0; i<4; ++i)
                for(int j=0; j<4; ++j)
                {
                    int index_src = i*4+j ;
                    int index_tgt = j*4+i ;
                    orig_.data[index_tgt] = trans[index_src] ;                                     
                }
        }

        return 0 ;
    }

    int WGS84ToCartesian(double lon,double lat,double h,double& x,double& y,double& z)
    {
    	x = lon*DEG_TO_RAD_;
    	y = lat*DEG_TO_RAD_;
    	z = h;
    	return pj_transform(projWGS84_,projCartesian_,1,1,&x,&y,&z);
    }

    int Cartesian2WGS84(double x,double y,double z,double& lon,double& lat,double& h)
    {
    	lon = x;
    	lat = y;
    	h = z;
    	int ret = pj_transform(projCartesian_, projWGS84_,1,1,&lon,&lat,&h);
    	lon *= RAD_TO_DEG_;
    	lat *= RAD_TO_DEG_;
    	return ret;
    }

    // inverse func
    double4x4 Inverse(const double4x4& matIn)
    {
    	double4x4 outMat;
    	
    	CvMat dstMat = cvMat(4, 4, CV_64FC1,(double*)outMat.data);
    
    	CvMat srcMat = cvMat(4, 4, CV_64FC1,(double*)matIn.data);
    	cvInvert(&srcMat, &dstMat,  CV_LU);
           
    	return outMat;
    }
    
    
    double4 Transform(const double4& vecIn, const double4x4 & matIn)
    {
    	double4 outMat;
    	CvMat mat1 = cvMat(1, 4, CV_64FC1,(double*)vecIn.data);
    	CvMat mat2 = cvMat(4, 4, CV_64FC1,(double*)matIn.data);
    	CvMat dstMat = cvMat(1, 4, CV_64FC1,(double*)outMat.data);
    	cvMatMul(&mat1, &mat2, &dstMat);
    	return outMat;
    }
    
    int Cartesian2Project(double x,double y,double z,const double4x4& proj2CartesianMatrix,double& outx,double& outy,double& outz)
    {
    	// 
    	double4x4 mat = Inverse(proj2CartesianMatrix);
    	double4 tmp;
    	tmp.data[0] = x;
    	tmp.data[1] = y;
    	tmp.data[2] = z;
    	tmp.data[3] = 1.0;
    
    	double4 out = Transform(tmp,mat);
    	outx = out.data[0];
    	outy = out.data[1];
    	outz = out.data[2];
    	return 0;
    }

    int Project2Cartesian(double x,double y,double z,const double4x4& proj2CartesianMatrix,double& outx,double& outy,double& outz)
    {
    	double4 tmp;
    	tmp.data[0] = x;
    	tmp.data[1] = y;
    	tmp.data[2] = z;
    	tmp.data[3] = 1.0;
    
    	double4 out = Transform(tmp,proj2CartesianMatrix);
    	outx = out.data[0];
    	outy = out.data[1];
    	outz = out.data[2];
    	return 0;
    }
    
    int latlonheight2local(const double latlon[3], double xyzl[3]) override
    {
        WGS842Project(latlon[1], latlon[0], latlon[2], orig_, xyzl[0], xyzl[1], xyzl[2]) ;
		
// 		double tmpvalue = xyzl[0];
// 		xyzl[0] = xyzl[1];
// 		xyzl[1] = tmpvalue;
// 		xyzl[2] = xyzl[2]*(-1.0);
		return 0 ;
    }

    
    int WGS842Project(double lon,double lat,double h,const double4x4& proj2CartesianMatrix,double& outx,double& outy,double& outz)
    {
    	WGS84ToCartesian(lon,lat,h,outx,outy,outz);
    	return Cartesian2Project(outx,outy,outz,proj2CartesianMatrix,outx,outy,outz);
    }
    
    int local2latlonheight(const double xyzl[3], double latlon[3]) override
    {
        Project2WGS84(xyzl[0], xyzl[1], xyzl[2], orig_, latlon[1], latlon[0], latlon[2]) ;

        return 0 ;
    }

    int Project2WGS84(double x,double y,double z,const double4x4& proj2CartesianMatrix,double& lon,double& lat,double& h)
    {
    	Project2Cartesian(x,y,z,proj2CartesianMatrix,lon,lat,h);
    	return Cartesian2WGS84(lon,lat,h,lon,lat,h);
    }
   
    public:

    const double RAD_TO_DEG_ = 57.29577951308232 ;
    const double DEG_TO_RAD_ = 0.0174532925199432958 ;
    const double a_ = 6378137 ;
    const double f_ = 1/298.257223563 ;
    const char* projStrCartesian_ = "+proj=geocent +datum=WGS84";
    const char* projStrWGS84_ = "+proj=latlong +datum=WGS84 +units=m";
    projPJ projWGS84_;
    projPJ projCartesian_ ;
    bool use_local_transformation_ ;    
    double4x4 orig_ ;
} ;








