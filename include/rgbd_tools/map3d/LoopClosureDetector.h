//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------


#ifndef RGBDSLAM_MAP3D_LOOPCLOSUREDETECTOR_H_
#define RGBDSLAM_MAP3D_LOOPCLOSUREDETECTOR_H_

#include <rgbd_tools/map3d/DataFrame.h>
#include <rgbd_tools/map3d/Database.h>

#ifdef USE_DBOW2
    #include <DBoW2/DBoW2.h>
#endif


namespace rgbd{
    template<typename PointType_>
    class LoopClosureDetector{
    public:
        bool init(std::string _path);
        void update(Database<PointType_>&_database);
    private:
        // update similarity matrix, based on Smith-Waterman code.
        void updateSimilarityMatrix(std::shared_ptr<DataFrame<PointType_>> &_kf, Database<PointType_>&_database);
        // check for loop closures in similarity matrix and update kfs and world dictionary, based on Smith-Waterman code.
        void checkLoopClosures(Database<PointType_> &_database);
        bool transformationBetweenFeatures(std::shared_ptr<DataFrame<PointType_>> &_previousKf, std::shared_ptr<DataFrame<PointType_>> &_currentKf, Eigen::Matrix4f &_transformation);
        bool matchDescriptors(const cv::Mat &_des1, const cv::Mat &_des2, std::vector<cv::DMatch> &_inliers);
    private:
        // Bundle adjustmen thread
        cv::Mat mSimilarityMatrix, mCumulativeMatrix;

        #ifdef USE_DBOW2
            OrbVocabulary mVocabulary;
        #endif
        std::thread mBaThread;
        std::vector<std::shared_ptr<DataFrame<PointType_>>>      mKeyframesBa;
        std::atomic<bool> mAlreadyBaThread{false};
        unsigned mDistanceSearch = 50;
        unsigned mBaSequenceSize = 5;
    };

}

#include "LoopClosureDetector.inl"

#endif
