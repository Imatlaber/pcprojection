/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef _HAD_CALIB_H_
#define _HAD_CALIB_H_

#include <map>
#include <stdint.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/bfgs.h>
#include <memory>
#include <lcm/lcm-cpp.hpp>
#include <bot_core/bot_core.h>
#include <lcm/lcm-cpp.hpp>
#include "common/pcl_viewer/pcl_viewer.hpp"
#include "utilities/hadif/param.hpp"
#include "utilities/hadif/trans.hpp"
#include "utilities/hadif/misc.hpp"

typedef Eigen::Matrix<float, 6, 1> Vector6f;

namespace had
{
  /** \brief CalibGeneralizedIterativeClosestPoint is an ICP variant that implements the 
    * generalized iterative closest point algorithm as described by Alex Segal et al. in 
    * http://www.robots.ox.ac.uk/~avsegal/resources/papers/Generalized_ICP.pdf
    * The approach is based on using anistropic cost functions to optimize the alignment 
    * after closest point assignments have been made.
    * The original code uses GSL and ANN while in ours we use an eigen mapped BFGS and 
    * FLANN.
    * \author Nizar Sallem
    * \ingroup registration
    */
  template <typename PointSource, typename PointTarget>
  class CalibGeneralizedIterativeClosestPoint : public pcl::IterativeClosestPoint<PointSource, PointTarget>
  {
    public:
      using pcl::IterativeClosestPoint<PointSource, PointTarget>::reg_name_;
      using pcl::IterativeClosestPoint<PointSource, PointTarget>::getClassName;
      using pcl::IterativeClosestPoint<PointSource, PointTarget>::indices_;
      using pcl::IterativeClosestPoint<PointSource, PointTarget>::target_;
      using pcl::IterativeClosestPoint<PointSource, PointTarget>::input_;
      using pcl::IterativeClosestPoint<PointSource, PointTarget>::tree_;
      using pcl::IterativeClosestPoint<PointSource, PointTarget>::tree_reciprocal_;
      using pcl::IterativeClosestPoint<PointSource, PointTarget>::nr_iterations_;
      using pcl::IterativeClosestPoint<PointSource, PointTarget>::max_iterations_;
      using pcl::IterativeClosestPoint<PointSource, PointTarget>::previous_transformation_;
      using pcl::IterativeClosestPoint<PointSource, PointTarget>::final_transformation_;
      using pcl::IterativeClosestPoint<PointSource, PointTarget>::transformation_;
      using pcl::IterativeClosestPoint<PointSource, PointTarget>::transformation_epsilon_;
      using pcl::IterativeClosestPoint<PointSource, PointTarget>::converged_;
      using pcl::IterativeClosestPoint<PointSource, PointTarget>::corr_dist_threshold_;
      using pcl::IterativeClosestPoint<PointSource, PointTarget>::inlier_threshold_;
      using pcl::IterativeClosestPoint<PointSource, PointTarget>::min_number_correspondences_;
      using pcl::IterativeClosestPoint<PointSource, PointTarget>::update_visualizer_;

      typedef pcl::PointCloud<PointSource> PointCloudSource;
      typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
      typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

      typedef pcl::PointCloud<PointTarget> PointCloudTarget;
      typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
      typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

      typedef pcl::PointIndices::Ptr PointIndicesPtr;
      typedef pcl::PointIndices::ConstPtr PointIndicesConstPtr;

      typedef std::vector< Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > MatricesVector;
      typedef boost::shared_ptr< MatricesVector > MatricesVectorPtr;
      typedef boost::shared_ptr< const MatricesVector > MatricesVectorConstPtr;
      
      typedef typename pcl::Registration<PointSource, PointTarget>::KdTree InputKdTree;
      typedef typename pcl::Registration<PointSource, PointTarget>::KdTreePtr InputKdTreePtr;

      typedef typename pcl::Registration<PointSource, PointTarget>::KdTree KdTreeTgt ;
      typedef typename pcl::Registration<PointSource, PointTarget>::KdTreePtr KdTreePtrTgt ;
      typedef typename pcl::Registration<PointSource, PointTarget>::KdTreeReciprocal  KdTreeSrc ;
      typedef typename pcl::Registration<PointSource, PointTarget>::KdTreeReciprocalPtr KdTreePtrSrc ;

      typedef boost::shared_ptr< CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget> > Ptr;
      typedef boost::shared_ptr< const CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget> > ConstPtr;

      typedef Eigen::Matrix<double, 6, 1> Vector6d;

		struct CalibPair
		{
			Eigen::Matrix4d trans_src_ ;
			Eigen::Matrix4d trans_tgt_ ;
			Eigen::Matrix4f trans_src_4f_ ;
			Eigen::Matrix4f trans_tgt_4f_ ;
			Eigen::Matrix3d rot_src_ ;
			Eigen::Matrix3d rot_tgt_ ;
			Eigen::Matrix3d rot_diff_ ;
			PointCloudSourceConstPtr pc_src_ ;
			PointCloudTargetConstPtr pc_tgt_ ;
			std::vector<int> indices_src_ ;
			std::vector<int> indices_tgt_ ;
			KdTreePtrSrc tree_src_ ;
			KdTreePtrTgt tree_tgt_ ;
			MatricesVector cov_src_ ;
			MatricesVector cov_tgt_ ;
			MatricesVector mahalanobis_ ;
            MatricesVector timed_rot_diff_ ;
            std::vector<Eigen::Vector3f> normal_src_ ;
            std::vector<Eigen::Vector3f> normal_tgt_ ;
            std::vector<Eigen::Matrix3d> cov_src_plane_ ;
            std::vector<Eigen::Matrix3d> cov_tgt_plane_ ;
            std::vector<Eigen::Vector3d> normal_src_plane_ ;
            std::vector<Eigen::Vector3d> normal_tgt_plane_ ;
            std::map<int64_t, Eigen::Matrix4f> trans_lookup_src_ ;
            std::map<int64_t, Eigen::Matrix4f> trans_lookup_tgt_ ;



		} ;

      /** \brief Empty constructor. */
      CalibGeneralizedIterativeClosestPoint () 
        : k_correspondences_(20)
        , gicp_epsilon_(1e-5)
        , rotation_epsilon_(2e-3)
        , mahalanobis_(0)
        , max_inner_iterations_(20)
        , standalone_(true)
        , draw_cloud_lidar_(false)
        , draw_normal_lidar_(false) 
        , draw_cloud_world_(false)
        , draw_cloud_world_final_(false)
        , functor_(this) 
        , max_normal_diff_(0.5)
        , max_nearest_search_(10)
        , estimate_rpy_only_(false)
		, max_convergence_error_(100.)
        , resolution_us_(50)
        , use_point_time_(true)
      {
        min_number_correspondences_ = 4;
        reg_name_ = "CalibGeneralizedIterativeClosestPoint";
        max_iterations_ = 200;
        transformation_epsilon_ = 5e-4;
        corr_dist_threshold_ = 5.;
        rigid_transformation_estimation_ = 
		boost::bind (&CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::estimateRigidTransformationBFGS, 
                       this, _1, _2, _3, _4, _5); 
		calib_estimation_ =
		boost::bind(&CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::estimateCalibBFGS, this, _1, _2) ;

        lcm_ = new lcm::LCM ;
        param_ = new param_t(lcm_) ; 
        trans_ = new trans_t(lcm_->getUnderlyingLCM(), param_) ;

        load_params() ;        
      }
     
      CalibGeneralizedIterativeClosestPoint (lcm::LCM * lcm, 
                                             param_t * param,
                                             trans_t * trans) 
        : k_correspondences_(20)
        , gicp_epsilon_(1e-5)
        , rotation_epsilon_(2e-3)
        , mahalanobis_(0)
        , max_inner_iterations_(20)
        , lcm_(lcm)
        , param_(param)
        , trans_(trans)
        , standalone_(false)
        , draw_cloud_lidar_(false)
        , draw_normal_lidar_(false) 
        , draw_cloud_world_(false)
        , draw_cloud_world_final_(false)
        , functor_(this) 
        , max_normal_diff_(0.5)
        , max_nearest_search_(10)
        , estimate_rpy_only_(false)
		, max_convergence_error_(100.)
        , resolution_us_(50)
        , use_point_time_(true)
      {
        min_number_correspondences_ = 4;
        reg_name_ = "CalibGeneralizedIterativeClosestPoint";
        max_iterations_ = 200;
        transformation_epsilon_ = 5e-4;
        corr_dist_threshold_ = 5.;
        rigid_transformation_estimation_ = 
		boost::bind (&CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::estimateRigidTransformationBFGS, 
                       this, _1, _2, _3, _4, _5); 
		calib_estimation_ =
		boost::bind(&CalibGeneralizedIterativeClosestPoint<PointSource, PointTarget>::estimateCalibBFGS, this, _1, _2) ;

        load_params() ;        
      }
     
      virtual ~ CalibGeneralizedIterativeClosestPoint()
      { 
        if(!standalone_)
            return ;
        /// make sure lcm is last one to free otherwise it will hang on lcm's destructor
        if(trans_)
            delete trans_ ;
        if(param_)
            delete param_ ;
        if(lcm_) 
            delete lcm_ ;
      }

      /** \brief Provide a pointer to the input dataset
        * \param cloud the const boost shared pointer to a PointCloud message
        */
      PCL_DEPRECATED ("[pcl::registration::CalibGeneralizedIterativeClosestPoint::setInputCloud] setInputCloud is deprecated. Please use setInputSource instead.")
      void
      setInputCloud (const PointCloudSourceConstPtr &cloud);

      /** \brief Provide a pointer to the input dataset
        * \param cloud the const boost shared pointer to a PointCloud message
        */
      inline void
      setInputSource (const PointCloudSourceConstPtr &cloud)
      {

        if (cloud->points.empty ())
        {
          PCL_ERROR ("[pcl::%s::setInputSource] Invalid or empty point cloud dataset given!\n", getClassName ().c_str ());
          return;
        }
        PointCloudSource input = *cloud;
        // Set all the point.data[3] values to 1 to aid the rigid transformation
        for (size_t i = 0; i < input.size (); ++i)
          input[i].data[3] = 1.0;
        
        pcl::IterativeClosestPoint<PointSource, PointTarget>::setInputSource (cloud);
        input_covariances_.reset ();
      }

      /** \brief Provide a pointer to the covariances of the input source (if computed externally!). 
        * If not set, CalibGeneralizedIterativeClosestPoint will compute the covariances itself.
        * Make sure to set the covariances AFTER setting the input source point cloud (setting the input source point cloud will reset the covariances).
        * \param[in] target the input point cloud target
        */
      inline void 
      setSourceCovariances (const MatricesVectorPtr& covariances)
      {
        input_covariances_ = covariances;
      }
      
      /** \brief Provide a pointer to the input target (e.g., the point cloud that we want to align the input source to)
        * \param[in] target the input point cloud target
        */
      inline void 
      setInputTarget (const PointCloudTargetConstPtr &target)
      {
        pcl::IterativeClosestPoint<PointSource, PointTarget>::setInputTarget(target);
        target_covariances_.reset ();
      }

      /** \brief Provide a pointer to the covariances of the input target (if computed externally!). 
        * If not set, CalibGeneralizedIterativeClosestPoint will compute the covariances itself.
        * Make sure to set the covariances AFTER setting the input source point cloud (setting the input source point cloud will reset the covariances).
        * \param[in] target the input point cloud target
        */
	    inline void 
      setTargetCovariances (const MatricesVectorPtr& covariances)
      {
        target_covariances_ = covariances;
      }
     
		inline void
		setTransSrc(const Eigen::Matrix4d& trans_src)
		{
			trans_src_ = trans_src ;
		}

		inline void
		setTransTgt(const Eigen::Matrix4d& trans_tgt)
		{
			trans_tgt_ = trans_tgt ;
		}

      /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using an iterative
        * non-linear Levenberg-Marquardt approach.
        * \param[in] cloud_src the source point cloud dataset
        * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
        * \param[in] cloud_tgt the target point cloud dataset
        * \param[in] indices_tgt the vector of indices describing the correspondences of the interst points from \a indices_src
        * \param[out] transformation_matrix the resultant transformation matrix
        */
      void
      estimateRigidTransformationBFGS (const PointCloudSource &cloud_src,
                                       const std::vector<int> &indices_src,
                                       const PointCloudTarget &cloud_tgt,
                                       const std::vector<int> &indices_tgt,
                                       Eigen::Matrix4f &transformation_matrix);
      
      /** \brief \return Mahalanobis distance matrix for the given point index */
      inline const Eigen::Matrix3d& mahalanobis(size_t index) const
      {
        assert(index < mahalanobis_.size());
        return mahalanobis_[index];
      }

      /** \brief Computes rotation matrix derivative.
        * rotation matrix is obtainded from rotation angles x[3], x[4] and x[5]
        * \return d/d_rx, d/d_ry and d/d_rz respectively in g[3], g[4] and g[5]
        * param x array representing 3D transformation
        * param R rotation matrix
        * param g gradient vector
        */
      void
      computeRDerivative(const Vector6d &x, const Eigen::Matrix3d &R, Vector6d &g) const;

      /** \brief Set the rotation epsilon (maximum allowable difference between two 
        * consecutive rotations) in order for an optimization to be considered as having 
        * converged to the final solution.
        * \param epsilon the rotation epsilon
        */
      inline void 
      setRotationEpsilon (double epsilon) { rotation_epsilon_ = epsilon; }

      /** \brief Get the rotation epsilon (maximum allowable difference between two 
        * consecutive rotations) as set by the user.
        */
      inline double 
      getRotationEpsilon () { return (rotation_epsilon_); }

      /** \brief Set the number of neighbors used when selecting a point neighbourhood
        * to compute covariances. 
        * A higher value will bring more accurate covariance matrix but will make 
        * covariances computation slower.
        * \param k the number of neighbors to use when computing covariances
        */
      void
      setCorrespondenceRandomness (int k) { k_correspondences_ = k; }

      /** \brief Get the number of neighbors used when computing covariances as set by 
        * the user 
        */
      int
      getCorrespondenceRandomness () { return (k_correspondences_); }

      /** set maximum number of iterations at the optimization step
        * \param[in] max maximum number of iterations for the optimizer
        */
      void
      setMaximumOptimizerIterations (int max) { max_inner_iterations_ = max; }

      ///\return maximum number of iterations at the optimization step
      int
      getMaximumOptimizerIterations () { return (max_inner_iterations_); }

    protected:

      /** \brief The number of neighbors used for covariances computation. 
        * default: 20
        */
      int k_correspondences_;

      /** \brief The epsilon constant for gicp paper; this is NOT the convergence 
        * tolerence 
        * default: 0.001
        */
      double gicp_epsilon_;

      /** The epsilon constant for rotation error. (In GICP the transformation epsilon 
        * is split in rotation part and translation part).
        * default: 2e-3
        */
      double rotation_epsilon_;

      /** \brief base transformation */
      Eigen::Matrix4f base_transformation_;

      /** \brief Temporary pointer to the source dataset. */
      const PointCloudSource *tmp_src_;

      /** \brief Temporary pointer to the target dataset. */
      const PointCloudTarget  *tmp_tgt_;

      /** \brief Temporary pointer to the source dataset indices. */
      const std::vector<int> *tmp_idx_src_;

      /** \brief Temporary pointer to the target dataset indices. */
      const std::vector<int> *tmp_idx_tgt_;

      
      /** \brief Input cloud points covariances. */
      MatricesVectorPtr input_covariances_;

      /** \brief Target cloud points covariances. */
      MatricesVectorPtr target_covariances_;

      /** \brief Mahalanobis matrices holder. */
      std::vector<Eigen::Matrix3d> mahalanobis_;
      
      /** \brief maximum number of optimizations */
      int max_inner_iterations_;

      /** \brief compute points covariances matrices according to the K nearest 
        * neighbors. K is set via setCorrespondenceRandomness() methode.
        * \param cloud pointer to point cloud
        * \param tree KD tree performer for nearest neighbors search
        * \param[out] cloud_covariances covariances matrices for each point in the cloud
        */
      template<typename PointT>
      void computeCovariances(typename pcl::PointCloud<PointT>::ConstPtr cloud, 
                              const typename pcl::search::KdTree<PointT>::Ptr tree,
                              MatricesVector& cloud_covariances) ;

    template<typename PointT>
    void computeCovariancesAndNormals(typename pcl::PointCloud<PointT>::ConstPtr cloud, 
                        const typename pcl::search::KdTree<PointT>::Ptr tree,
                        MatricesVector& cloud_covariances, 
                        std::vector<Eigen::Vector3f>& normals);

    template<typename PointT>
    void computeCovariancesAndNormals(typename pcl::PointCloud<PointT>::ConstPtr cloud, 
                        const typename pcl::search::KdTree<PointT>::Ptr tree,
                        MatricesVector& cloud_covariances, 
                        std::vector<Eigen::Vector3f>& normals, 
                        const std::vector<Eigen::Matrix3d>& cov_plane,
                        const std::vector<Eigen::Vector3d>& normal_plane);


      /** \return trace of mat1^t . mat2 
        * \param mat1 matrix of dimension nxm
        * \param mat2 matrix of dimension nxp
        */
      inline double 
      matricesInnerProd(const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2) const
      {
        double r = 0.;
        size_t n = mat1.rows();
        // tr(mat1^t.mat2)
        for(size_t i = 0; i < n; i++)
          for(size_t j = 0; j < n; j++)
            r += mat1 (j, i) * mat2 (i,j);
        return r;
      }
	

      /** \brief Rigid transformation computation method  with initial guess.
        * \param output the transformed input point cloud dataset using the rigid transformation found
        * \param guess the initial guess of the transformation to compute
        */
      void 
      computeTransformation (PointCloudSource &output, const Eigen::Matrix4f &guess);

      /** \brief Search for the closest nearest neighbor of a given point.
        * \param query the point to search a nearest neighbour for
        * \param index vector of size 1 to store the index of the nearest neighbour found
        * \param distance vector of size 1 to store the distance to nearest neighbour found
        */
      inline bool 
      searchForNeighbors (const PointSource &query, std::vector<int>& index, std::vector<float>& distance)
      {
        int k = tree_->nearestKSearch (query, 1, index, distance);
        if (k == 0)
          return (false);
        return (true);
      }

      /// \brief compute transformation matrix from transformation matrix
      void applyState(Eigen::Matrix4f &t, const Vector6d& x) const;
      
      /// \brief optimization functor structure
      struct OptimizationFunctorWithIndices : public BFGSDummyFunctor<double,6>
      {
        OptimizationFunctorWithIndices (const CalibGeneralizedIterativeClosestPoint* gicp)
          : BFGSDummyFunctor<double,6> (), gicp_(gicp) {}
        double operator() (const Vector6d& x);
        void  df(const Vector6d &x, Vector6d &df);
        void fdf(const Vector6d &x, double &f, Vector6d &df);

        const CalibGeneralizedIterativeClosestPoint *gicp_;
      };
      
      boost::function<void(const pcl::PointCloud<PointSource> &cloud_src,
                           const std::vector<int> &src_indices,
                           const pcl::PointCloud<PointTarget> &cloud_tgt,
                           const std::vector<int> &tgt_indices,
                           Eigen::Matrix4f &transformation_matrix)> rigid_transformation_estimation_;



	public:
	inline void initCalibMatrices()
	{
	  rot_src_ = trans_src_.topLeftCorner<3,3>() ;
	  rot_tgt_ = trans_tgt_.topLeftCorner<3,3>() ;
	  trans_src_4f_ = trans_src_.cast<float>() ;
	  trans_tgt_4f_ = trans_tgt_.cast<float>() ;
	  Eigen::Matrix3d T1 = trans_src_.topLeftCorner<3,3> () ;
	  Eigen::Matrix3d T0 = trans_tgt_.topLeftCorner<3,3> () ;
	  rot_diff_ = T1-T0 ;
	}
	Eigen::Matrix4d trans_src_ ;
	Eigen::Matrix4d trans_tgt_ ;
	Eigen::Matrix3d rot_src_ ;
	Eigen::Matrix3d rot_tgt_ ;
	Eigen::Matrix4f trans_src_4f_ ;
	Eigen::Matrix4f trans_tgt_4f_ ;
	Eigen::Matrix3d rot_diff_ ;

    lcm::LCM * lcm_ ;
    param_t * param_ ;
    trans_t * trans_ ;
    bool standalone_ ;
    bool draw_cloud_lidar_ ;
    bool draw_normal_lidar_ ;
    bool draw_cloud_world_ ;
    bool draw_cloud_world_final_ ;
    int correspondence_type_ ;
    int resolution_us_ ;
    bool use_point_time_ ;


	public:
	void add_pair(PointCloudSourceConstPtr pc_src, 
                    const Eigen::Matrix4d& trans_src, 
                    PointCloudTargetConstPtr pc_tgt, 
                    const Eigen::Matrix4d& trans_tgt) ; 
	void add_pair(PointCloudSourceConstPtr pc_src, 
                    const Eigen::Matrix4d& trans_src, 
                    const std::vector<Eigen::Matrix3d>& cov_src,
                    const std::vector<Eigen::Vector3d>& normal_src,
                    PointCloudTargetConstPtr pc_tgt, 
                    const Eigen::Matrix4d& trans_tgt,
                    const std::vector<Eigen::Matrix3d>& cov_tgt,
                    const std::vector<Eigen::Vector3d>& normal_tgt) ; 
	void exec(const Eigen::Matrix4f& guess) ;
	bool initCalibPairs() ;
	bool initCalibPair(CalibPair& p) ;
    Eigen::Matrix3d computeMahalanobis(const Eigen::Matrix3d& C1,
                                       const Eigen::Matrix3d& R1, 
                                       const Eigen::Matrix3d& C0, 
                                       const Eigen::Matrix3d& R0, 
                                       const Eigen::Matrix3d& R) ;
    void updateMahalanobis(const Eigen::Matrix3d& curr_rot) ;
	bool computeCorrespondences(const Eigen::Matrix4f& curr_trans, const Eigen::Matrix3d& curr_rot) ;
	bool computeCorrespondences(const Vector6d& x) ;
	bool computeCorrespondence(const Eigen::Matrix4f& curr_trans, 
                                const Eigen::Matrix3d& curr_rot, 
                                CalibPair& p) ;
	bool computeCorrespondence2(const Eigen::Matrix4f& curr_trans, 
                                const Eigen::Matrix3d& curr_rot, 
                                CalibPair& p,
                                int pair_index) ;
	bool computeCorrespondence3(const Eigen::Matrix4f& curr_trans, 
                                const Eigen::Matrix3d& curr_rot, 
                                CalibPair& p) ;
	double computeError(const Vector6d& x, std::string solName="solution x") ;  
    template<typename PointT>
	bool searchNearestNeighbors(const pcl::search::KdTree<PointT>& tree,  
                                const PointT &query, 
                                int num, 
                                std::vector<int>& index, 
                                std::vector<float>& distance) ;
    bool estimateCalibBFGS (const std::vector<CalibPair>& pairs, 
                            Eigen::Matrix4f& trans) ;
    Vector6f getFinalXYZRPY() ;
    Vector6f getXYZRPY(const Eigen::Matrix4f& trans) ;
    void setGicpEpsilon(double e) ;
    void load_params() ;
    inline void setMaxNormalDiffSamePlane(double diff) { max_normal_diff_ = diff ; }
    inline void setMaxNearestSearchPoints(int num) { max_nearest_search_ = num ; }
    inline void setRpyOnly(bool rpy_only) {  estimate_rpy_only_ = rpy_only; }
	inline void setMaxConvergenceError(double max_error) { max_convergence_error_ = max_error ; }
    inline void setMaxIterations(int max_ite) { max_iterations_ = max_ite ; }
    inline void setUsePointTime(bool use_point_time) { use_point_time_ = use_point_time ; }

    template<typename PointT>
    void drawNormal(boost::shared_ptr<had::PCLViewer> viewer, 
                    const pcl::PointCloud<PointT>& cloud,
                    const MatricesVector& covariances,
                    const std::array<int,3> color,
                    const char * base_name) ;

    void drawCloudWorld(const Eigen::Matrix4f& curr_trans,
                        const CalibPair& pair,
                        std::string winName="cloud in world frame") ;
    void snoop(const Vector6d& x, std::string solName = "solution x") ;
    void snoop2(const Vector6d& x, std::string solName = "solution x") ;

    template<class T>
    Eigen::Matrix4f get_trans(const T& point,
                              const std::map<int64_t, Eigen::Matrix4f>& trans_lookup, 
                              const Eigen::Matrix4f& trans_fixed) const 
    {
        Eigen::Matrix4f trans ;
        if(use_point_time_)
        {
	        int64_t utime = 0 ;
	        memcpy(&utime, &point.utime1, sizeof(int32_t)) ;
	        memcpy((char*)&utime+4, &point.utime2, sizeof(int32_t)) ;
            int64_t curr_key = utime / resolution_us_ ;
            if(trans_lookup.find(curr_key) != trans_lookup.end())
            {
                trans = trans_lookup.at(curr_key) ;
                //std::cout << "+++++++++++ reading utime " << utime << std::endl ;
                //std::cout << trans << std::endl ;
            }
            else
            {
                printf("Error: failed to obtain transformation for key %ld \n", curr_key) ;
                exit(1) ;
            }
        }
        else
        {
            trans = trans_fixed ;
        }

        return trans ;
    }

    
    template<class T>
    void set_trans(const T& point,
                              std::map<int64_t, Eigen::Matrix4f>& trans_lookup) 
    {
	    int64_t utime = 0 ;
	    memcpy(&utime, &point.utime1, sizeof(int32_t)) ;
	    memcpy((char*)&utime+4, &point.utime2, sizeof(int32_t)) ;
        int64_t curr_key = utime / resolution_us_ ;
        Eigen::Matrix4f trans ;
        if(trans_lookup.find(curr_key) == trans_lookup.end())
        {
            BotTrans bot_trans ;
            if(!bot_frames_get_trans_with_utime(trans_->frames, 
                                                "body",
                                                "global",
                                                utime, 
                                                &bot_trans))
            {
                printf("failed to obtain transformation from body to global frame \n") ;
                exit(1) ;
            }
            trans = ext_bot_trans_to_matrix4d(&bot_trans).cast<float>() ;
            trans_lookup[curr_key] = trans ;
        }
    }

    template<class T>
    void transformTimedPointCloud(const pcl::PointCloud<T>& cloud_in,
                                  const std::map<int64_t, Eigen::Matrix4f>& trans_lookup,
                                  const Eigen::Matrix4f& trans_fixed,
                                  const Eigen::Matrix4f& trans_init,
                                  pcl::PointCloud<T>& cloud_out)
    {
        cloud_out.clear() ;
        for(const auto& p : cloud_in)
        {
            /// obtain transformation from body to world frame 
            Eigen::Matrix4f trans_b2w = get_trans(p, trans_lookup, trans_fixed) ; 
            Eigen::Affine3f trans_l2w(trans_b2w * trans_init) ;  
            auto p_out = pcl::transformPoint(p, trans_l2w) ;
            cloud_out.push_back(p_out) ;
        }
    }

	/// \brief optimization functor structure
	struct CalibOptimizationFunctorWithIndices : public BFGSDummyFunctor<double,6>
	{
	  CalibOptimizationFunctorWithIndices (const CalibGeneralizedIterativeClosestPoint* gicp)
	    : BFGSDummyFunctor<double,6> (), gicp_(gicp) {}
	  double operator() (const Vector6d& x);
	  void  df(const Vector6d &x, Vector6d &df);
	  void fdf(const Vector6d &x, double &f, Vector6d &df);
	
	  const CalibGeneralizedIterativeClosestPoint *gicp_;
	};
      
	std::vector<CalibPair> pairs_ ;
	std::shared_ptr< const std::vector<CalibPair> > pairs_ptr_ ;
	boost::function<bool(const std::vector<CalibPair>& pairs, Eigen::Matrix4f& trans)> calib_estimation_ ;
    CalibOptimizationFunctorWithIndices functor_ ;
    bool estimate_xyz_ ;
    double max_normal_diff_ ;
    int max_nearest_search_ ;
    bool estimate_rpy_only_ ;	
	double max_convergence_error_ ;
  };
}

#include "had_calib.hpp"

#endif  //#ifndef _HAD_CALIB_H_
