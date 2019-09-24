//---------------------------------------------------------------------------------------------------------------------
//  mico
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


#ifndef MICO_BASE_OBJECTDETECTION_ML_SLIDINGWINDOW_H_
#define MICO_BASE_OBJECTDETECTION_ML_SLIDINGWINDOW_H_

#include <vector>
#include <opencv2/opencv.hpp>

namespace mico {
	/// Implementation of sliding window extractor.
	class SlidingWindow {
	public:		//	Public interface
		/// Parameters for sliding window.
		struct Params {
			int wHeight;		/// Window's height.
			int wWidth;			/// Window's width.
			int vStep;			/// Vertical step.
			int hStep;			/// Horizontal step.
			int scaleSteps;		///	Number of scale steps (1 means 1 scale, i.e., do not scale image anytime).
			float scaleFactor;	/// Scale factor for every step.
		};

		/// Sliding window constructor.
		/// \params _params: parameters for sliding window method.
		SlidingWindow(Params _params);

		/// Set new params to method.
		/// \params _params: parameters for sliding window method.
		void params(Params _params);

		/// Get current params.
		/// \return  current params of method.
		Params params() const;

		/// Extract windows from images using current params.
		/// \return list of windows extracted from the image.
		std::vector<cv::Mat> subwindows(const cv::Mat &_image);

		/// Get rectangles
		std::vector<cv::Rect> grid(const cv::Mat &_image) const;
	private:	//	Members
		Params mParams;
	};	//	class SlidingWindow
}	//	namesapce rgbd_tools


#endif	//	RGBDTOOLS_OBJECTDETECTION_ML_SLIDINGWINDOW_H_