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


#include <rgbd_tools/cjson/json.h>
#include <opencv2/opencv.hpp>
#include <rgbd_tools/object_detection/feature_based/ORBextractor.h>

namespace rgbd{

	class ImageFeatureManager{
	public:
		bool configure(const cjson::Json &_configuration);

		bool compute(const cv::Mat &_frame, std::vector<cv::Point2f> &_points, cv::Mat &_descriptors, cv::Rect _roi = cv::Rect(0,0, -1, -1));

		bool match(const cv::Mat &_desc1, const cv::Mat &_desc2, std::vector<cv::DMatch> &_matches);
	private:
		enum class eDetector {FAST, ORB, ORB_SLAM, SIFT, SURF};
		eDetector mDetector = eDetector::FAST;
		enum class eDescriptor {BRIEF, rBRIEF, ORB, ORB_SLAM, SIFT, SURF};
		eDescriptor mDescriptor = eDescriptor::SIFT;

		int mMaxMatchingDistance = 16;

		cv::Ptr<cv::FeatureDetector> mFeatureDetector;
		cv::Ptr<cv::FeatureDetector> mFeatureDescriptor;
		cv::Ptr<cv::DescriptorMatcher> mMatcher;
		ORB_SLAM2::ORBextractor *mpORBextractor; 
	};
}
