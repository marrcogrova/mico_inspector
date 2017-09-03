////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////


#ifndef RGBDTOOLS_MAP3D_BUNDLEADJUSTER_H_
#define RGBDTOOLS_MAP3D_BUNDLEADJUSTER_H_

#include <map3d/keyframe.h>

namespace rgbd{
    template<typename PointType_>
    class BundleAdjuster{
    public:
        bool optimize();
        void keyframes(std::vector<std::shared_ptr<Keyframe<PointType_>>> &_keyframes);
        void keyframes(typename std::vector<std::shared_ptr<Keyframe<PointType_>>>::iterator &_begin, typename std::vector<std::shared_ptr<Keyframe<PointType_>>>::iterator &_end);

        // ---- Getters ----
        /// \brief Get minimum error set as stopping criteria for the Bundle Adjustment process.
        /// \return minimum error.
        double      minError       () const;

        /// \brief Get number of iterations set as stopping criteria for the Bundle Adjustment process.
        /// \return iterations.
        unsigned    iterations     () const;

        /// \brief Get minumim number of times that a points needs to be observed to be used in the Bundle Adjustment.
        /// \return number of aparitions.
        unsigned    minAparitions  () const;

        // ---- Setters ----
        /// \brief Set minimum error set as stopping criteria for the Bundle Adjustment process.
        /// \param _error: minimum error.
        void minError         (double _error);

        /// \brief Set number of iterations set as stopping criteria for the Bundle Adjustment process.
        /// \param _iterations iterations.
        void iterations       (unsigned _iterations);

        /// \brief Set minumim number of times that a points needs to be observed to be used in the Bundle Adjustment.
        /// \param _aparitions: number of aparitions.
        void minAparitions    (unsigned _aparitions);

        /// \brief Get keyframes. Optimized of optimize() is call and success.
        /// \return internal stored keyframes.
        std::vector<Keyframe<PointType_>, Eigen::aligned_allocator <Keyframe<PointType_>>> keyframes();

    private:
        void cleanData();
        bool prepareData();

    private:
        std::vector<std::shared_ptr<Keyframe<PointType_>>> mKeyframes;

        // Parameters of Bundle Adjustment.
        double      mBaMinError = 1e-10;
        unsigned    mBaIterations = 100;
        unsigned    mBaMinAparitions = 3;

        std::vector<cv::Point3f>                mScenePoints;
        std::vector<std::vector<int>>           mCovisibilityMatrix;
        std::vector<std::vector<cv::Point2f>>   mScenePointsProjection;
    };
}   // namespace rgbd

#include <map3d/BundleAdjuster.inl>

#endif //RGBDTOOLS_MAP3D_BUNDLEADJUSTER_H_
