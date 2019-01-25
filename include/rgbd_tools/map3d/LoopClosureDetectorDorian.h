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

#ifndef RGBDTOOLS_MAP3D_LOOPCLOSUREDETECTORDORIAN_H_
#define RGBDTOOLS_MAP3D_LOOPCLOSUREDETECTORDORIAN_H_

#include <rgbd_tools/map3d/LoopClosureDetector.h>

#include <DBoW2/DBoW2.h> // defines BriefVocabulary
#include <DLoopDetector/DLoopDetector.h> // defines BriefLoopDetecto

namespace rgbd{
    template <DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Cout>
    class LoopClosureDetectorDorian: public LoopClosureDetector<DebugLevel_, OutInterface_>{
    public:
        /// Initialize loop closure detector with specific data for DLoopDetector by Dorian
        virtual bool init(cjson::Json &_config);

        /// Append a new image with an attached ID to the loop detector. It returns data related with the detection results.
        virtual LoopResult appendCluster(cv::Mat &_image, int _id);

        /// Append a new image with an attached ID to the loop detector. It returns data related with the detection results.
        virtual LoopResult appendCluster(const std::vector<cv::KeyPoint> &_kps, const cv::Mat &_descriptors, int _id);
    private:
        cv::Ptr<cv::ORB> mFeatureDetector ; // 666 Configurable?

        BriefVocabulary *mLoopClosureVoc;
        BriefLoopDetector *mLoopClosureDetector;

        DUtilsCV::GUI::tWinHandler win = "Current image";
        DUtilsCV::GUI::tWinHandler winplot = "Trajectory";

        DUtilsCV::Drawing::Plot::Style normal_style; // thickness
        DUtilsCV::Drawing::Plot::Style loop_style; // color, thickness
        DUtilsCV::Drawing::Plot  *implot;
        Eigen::Vector3f prevPos;

    };

};  // namespace rgbd


#include <rgbd_tools/map3d/LoopClosureDetectorDorian.inl>

#endif