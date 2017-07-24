
#ifndef RGBDSLAM_MAP3D_SCENE_H_
#define RGBDSLAM_MAP3D_SCENE_H_

#include "keyframe.h"

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "RansacP2P.h"
namespace rgbd{
    template<typename PointType_>
    class SceneRegistrator{
    public: // Public interface.
        /// \brief Add a new keyframe to the scene.
        /// \_kf: key frame to be added.
        bool addKeyframe(const Keyframe<PointType_> &_kf);

        /// \brief get copy of internal list of keyframes.
        /// \return Copy of internal list of keyframes.
        std::vector<Keyframe<PointType_>, Eigen::aligned_allocator <Keyframe<PointType_>>> keyframes() const;

        /// \brief Optimize scene with current keyframes (no loop closure)
        bool optimize();

        pcl::PointCloud<PointType_> map() const;

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
        /// \param _minInliers: maximum reprojection error.
        int ransacMinInliers () const;

        /// \brief get maximum transformation allowed for icp convergence criteria
        /// \param _maxExpsilon.
        double icpMaxTransformationEpsilon() const;

        /// \brief get maximum distance used for searching correspondences.
        /// \param _distance.
        double icpMaxCorrespondenceDistance() const;

        /// \brief get distance for voxel filtering
        /// \param _distance.
        double icpVoxelDistance() const;

        /// \brief get maximum fitnes score for icp convergence
        /// \param _minInliers: maximum reprojection error.
        double icpMaxFitnessScore() const;

        /// \brief  get maximum iterations for icp convergence
        /// \param _minInliers: maximum reprojection error.
        int icpMaxIterations() const;

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


    private: // Private methods.
        bool matchDescriptors(const std::vector<cv::Point2f> &_kps1, const std::vector<cv::Point2f> &_kps2, const cv::Mat &_des1, const cv::Mat &_des2, std::vector<cv::DMatch> &_inliers, cv::Mat &_R = cv::Mat(), cv::Mat &_T = cv::Mat());
        void initDataBA(const Keyframe<PointType_> &_firstKf);
        void extendDataBA(const Keyframe<PointType_> &_currentKf);

        // Compute roughtly but robustly the transformation between given keyframes.
        bool transformationBetweenFeatures(const Keyframe<PointType_> &_previousKf, const Keyframe<PointType_> &_currentKf, Eigen::Matrix4f &_transformation);
        // Assuming that keyframes are close enough, refine the transformation between both keyframes.
        bool refineTransformation(const Keyframe<PointType_> &_previousKf, const Keyframe<PointType_> &_currentKf, Eigen::Matrix4f &_transformation);

    private: // Members.
        std::vector<Keyframe<PointType_>, Eigen::aligned_allocator <Keyframe<PointType_>>>       mKeyframes;

        std::vector<cv::Point3f>                mSceneFeaturePoints;
        cv::Mat                                 mCovisibilityMatrix;
        std::vector<std::vector<int>>           mDescriptorToCovisibilityIndices;
        std::vector<std::vector<cv::Point2f>>   mSceneFeatureProjections;

        pcl::PointCloud<PointType_> mMap;

        // Parameters of Bundle Adjustment.
        double      mBaMinError = 1e-10;
        unsigned    mBaIterations = 100;
        unsigned    mBaMinAparitions = 3;

        // Ransac parameters
		rgbd::RansacP2P<PointType_> mRansacAligner;
        double      mFactorDescriptorDistance = 8;
        int         mRansacIterations = 100;
        double      mRansacMaxDistance = 0.03;
        int         mRansacMinInliers = 8;

        // Icp parameters
        double mIcpMaxTransformationEpsilon = 1e-6;
        double mIcpMaxCorrespondenceDistance = 0.03;
        double mIcpVoxelDistance = 0.01;
        double mIcpMaxFitnessScore = 1;
        unsigned mIcpMaxIterations = 30;
    };
}

#include "SceneRegistrator.inl"

#endif
