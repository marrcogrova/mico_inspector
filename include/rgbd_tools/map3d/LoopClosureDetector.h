////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#ifndef RGBDSLAM_MAP3D_LOOPCLOSUREDETECTOR_H_
#define RGBDSLAM_MAP3D_LOOPCLOSUREDETECTOR_H_

#include <rgbd_tools/map3d/keyframe.h>
#include <rgbd_tools/map3d/Database.h>

#ifdef USE_DBOW2
    #include <DBoW2/DBoW2.h>
#endif


namespace rgbd{
    template<typename PointType_>
    class LoopClosureDetector{
    public:
        bool init(std::string _path);
        void update(std::shared_ptr<Keyframe<PointType_>> &_kf, Database<PointType_>&_database);
    private:
        // update similarity matrix, based on Smith-Waterman code.
        void updateSimilarityMatrix(std::shared_ptr<Keyframe<PointType_>> &_kf, Database<PointType_>&_database);
        // check for loop closures in similarity matrix and update kfs and world dictionary, based on Smith-Waterman code.
        void checkLoopClosures(Database<PointType_> &_database);
        bool transformationBetweenFeatures(std::shared_ptr<Keyframe<PointType_>> &_previousKf, std::shared_ptr<Keyframe<PointType_>> &_currentKf, Eigen::Matrix4f &_transformation);
        bool matchDescriptors(const cv::Mat &_des1, const cv::Mat &_des2, std::vector<cv::DMatch> &_inliers);
    private:
        // Bundle adjustmen thread
        cv::Mat mSimilarityMatrix, mCumulativeMatrix;

        #ifdef USE_DBOW2
            OrbVocabulary mVocabulary;
        #endif
        std::thread mBaThread;
        std::vector<std::shared_ptr<Keyframe<PointType_>>>      mKeyframesBa;
        std::atomic<bool> mAlreadyBaThread{false};
        unsigned mDistanceSearch = 50;
        unsigned mBaSequenceSize = 5;
    };

}

#include "LoopClosureDetector.inl"

#endif
