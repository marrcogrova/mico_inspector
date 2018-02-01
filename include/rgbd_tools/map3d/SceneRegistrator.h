////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#ifndef RGBDSLAM_MAP3D_SCENE_H_
#define RGBDSLAM_MAP3D_SCENE_H_

#include <atomic>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <rgbd_tools/map3d/keyframe.h>
#include <rgbd_tools/map3d/RansacP2P.h>
#include <rgbd_tools/map3d/BundleAdjuster.h>
#include <rgbd_tools/map3d/Word.h>
#include <rgbd_tools/map3d/LoopClosureDetector.h>
#include <rgbd_tools/map3d/Database.h>

#ifdef USE_DBOW2
    #include <DBoW2/DBoW2.h>
#endif

namespace rgbd{
    /// Class for SLAM
    /// Usage:
    /// @code
    ///
    /// @endcode
    template<typename PointType_>
    class SceneRegistrator{
    public: // Public interface.
        /// \brief constructor
        /// Initializes members and threads.
        SceneRegistrator();

        /// \brief Add a new keyframe to the scene.
        /// \_kf: key frame to be added.
        bool addKeyframe(std::shared_ptr<Keyframe<PointType_>> &_kf);

        /// \brief get copy of internal list of keyframes.
        /// \return Copy of internal list of keyframes.
        std::vector<std::shared_ptr<Keyframe<PointType_>>> keyframes();

        pcl::PointCloud<PointType_> map();
        pcl::PointCloud<PointType_> featureMap();

        std::shared_ptr<Keyframe<PointType_>> lastFrame() const;

        std::map<int, std::shared_ptr<Word>> worldDictionary();

        void reset(){
            mDatabase.reset();
            mLastKeyframe = nullptr;
            mMap.clear();
        }

        // ---- Getters ----
        /// \brief Get minimum error set as stopping criteria for the Bundle Adjustment process.
        /// \return minimum error
        double      baMinError       () const;

        /// \brief Get number of iterations set as stopping criteria for the Bundle Adjustment process.
        /// \return iterations
        unsigned    baIterations     () const;

        /// \brief Get minumim number of times that a points needs to be observed to be used in the Bundle Adjustment
        /// \return number of aparitions
        unsigned    baMinAparitions  () const;

        /// \brief Get factor param used to filter descriptors.
        /// \return factor
        double descriptorDistanceFactor     () const;

        /// \brief Get confidence set for a RANSAC process used for filter descriptors.
        /// \return confidence
        int ransacIterations             () const;

        /// \brief Get maximum allowed reprojection error for a RANSAC process used for filter descriptors.
        /// \return maximum reprojection error.
        double ransacMaxDistance   () const;

        /// \brief get minimum number of inliers for considering the cloud for the ransac process used for rought alignment and outlier filtering
        /// \return  maximum reprojection error.
        int ransacMinInliers () const;

        /// \brief get maximum transformation allowed for icp convergence criteria
        /// \return maxExpsilon.
        double icpMaxTransformationEpsilon() const;

        /// \brief get maximum distance used for searching correspondences.
        /// \return distance.
        double icpMaxCorrespondenceDistance() const;

        /// \brief get distance for voxel filtering
        /// \return distance.
        double icpVoxelDistance() const;

        /// \brief get maximum fitnes score for icp convergence
        /// \return  maximum reprojection error.
        double icpMaxFitnessScore() const;

        /// \brief  get maximum iterations for icp convergence
        /// \return maximum reprojection error.
        int icpMaxIterations() const;

        /// \brief see if icp enabled/disabled
        /// \return : true to enable false to disable
        bool icpEnabled() const;

        // ---- Setters ----
        /// \brief Set minimum error set as stopping criteria for the Bundle Adjustment process.
        /// \param _error: minimum error.
        void baMinError         (double _error);

        /// \brief Set number of iterations set as stopping criteria for the Bundle Adjustment process.
        /// \param _iterations iterations.
        void baIterations       (unsigned _iterations);

        /// \brief Set minumim number of times that a points needs to be observed to be used in the Bundle Adjustment
        /// \param _aparitions: number of aparitions.
        void baMinAparitions    (unsigned _aparitions);

        /// \brief Set factor param used to filter descriptors.
        /// \param _factor: factor.
        void descriptorDistanceFactor       (double _factor);

        /// \brief Set number of iterations for the ransac process used for rought alignment and outlier filtering
        /// \param _iterations: confidence.
        void ransacIterations               (int _iterations);

        /// \brief Set max distance allowed for considering inliers for the ransac process used for rought alignment and outlier filtering
        /// \param _maxDistance: maximum reprojection error.
        void ransacMaxDistance      (double _maxDistance);

        /// \brief Set minimum number of inliers for considering the cloud for the ransac process used for rought alignment and outlier filtering
        /// \param _minInliers: maximum reprojection error.
        void ransacMinInliers       (int _minInliers);

        /// \brief Set maximum transformation allowed for icp convergence criteria
        /// \param _maxExpsilon.
        void icpMaxTransformationEpsilon        (double _maxEpsilon);

        /// \brief Set maximum distance used for searching correspondences.
        /// \param _distance.
        void icpMaxCorrespondenceDistance       (double _distance);

        /// \brief Set distance for voxel filtering
        /// \param _distance.
        void icpVoxelDistance(double _distance);

        /// \brief Set maximum fitnes score for icp convergence
        /// \param _minInliers: maximum reprojection error.
        void icpMaxFitnessScore(double _maxScore);

        /// \brief  Set maximum iterations for icp convergence
        /// \param _minInliers: maximum reprojection error.
        void icpMaxIterations (int _maxIters);

        /// \brief enable/disable icp
        /// \param _enable: true to enable false to disable
        void icpEnabled(bool _enable);

        /// \brief init vocabulary
        /// \param _path: path to file containing the vocabulary
        /// \return true if initialized, false if not.
        bool initVocabulary(std::string _path);

    private: // Private methods.
        bool matchDescriptors(const cv::Mat &_des1, const cv::Mat &_des2, std::vector<cv::DMatch> &_inliers);

        // Compute roughtly but robustly the transformation between given keyframes.
        bool transformationBetweenFeatures(std::shared_ptr<Keyframe<PointType_>> &_previousKf, std::shared_ptr<Keyframe<PointType_>> &_currentKf, Eigen::Matrix4f &_transformation);
        // Assuming that keyframes are close enough, refine the transformation between both keyframes.
        bool refineTransformation(std::shared_ptr<Keyframe<PointType_>> &_previousKf, std::shared_ptr<Keyframe<PointType_>> &_currentKf, Eigen::Matrix4f &_transformation);
    private: // Members.
        Database<PointType_> mDatabase;
        std::shared_ptr<Keyframe<PointType_>>                   mLastKeyframe;

        pcl::PointCloud<PointType_> mMap;

        BundleAdjuster<PointType_> mBA;
        bool mUpdateMapVisualization = false;

        // Ransac parameters
		rgbd::RansacP2P<PointType_> mRansacAligner;
        double      mFactorDescriptorDistance = 8;
        int         mRansacIterations = 100;
        double      mRansacMaxDistance = 0.03;
        int         mRansacMinInliers = 8;

        // Icp parameters
        bool        mIcpEnabled = false;
        double      mIcpMaxTransformationEpsilon = 1e-6;
        double      mIcpMaxCorrespondenceDistance = 0.03;
        double      mIcpVoxelDistance = 0.01;
        double      mIcpMaxFitnessScore = 1;
        unsigned    mIcpMaxIterations = 30;

        LoopClosureDetector<PointType_> mLoopClosureDetector;
        bool mDoLoopClosure  = false;
    };
}

#include "SceneRegistrator.inl"

#endif
