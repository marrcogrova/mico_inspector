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

#include <mico/base/object_detection/ml/SlidingWindow.h>

using namespace cv;
using namespace std;

namespace mico {
	//-----------------------------------------------------------------------------------------------------------------
	SlidingWindow::SlidingWindow(Params _params): mParams(_params){

	}

	//-----------------------------------------------------------------------------------------------------------------
	void SlidingWindow::params(Params _params) {
		mParams = _params;
	}

	//-----------------------------------------------------------------------------------------------------------------
	SlidingWindow::Params SlidingWindow::params() const{
		return mParams;
	}

	//-----------------------------------------------------------------------------------------------------------------
	std::vector<cv::Mat> SlidingWindow::subwindows(const cv::Mat &_image) {
		assert(_image.channels() == 1 && _image.rows != 0);
		std::vector<cv::Mat> windows;

		float scaleFactor = 1;	// Scale factor for each iteration
		for(int k = 0; k < mParams.scaleSteps; k++){
			// Resize image
			Mat scaledImg;
			_image.copyTo(scaledImg);
			resize(_image, scaledImg, Size(), scaleFactor, scaleFactor);

			// Compute number of windows in vertical and horizontal directions.
			int vImgs = (scaledImg.rows - mParams.wHeight)/mParams.vStep + 1;
			int hImgs = (scaledImg.cols - mParams.wWidth) /mParams.hStep + 1;

			// Stop scaling if image is smaller than window.
			if (scaledImg.rows < mParams.wHeight || scaledImg.cols < mParams.wWidth) {
				break;
			}

			for (int i = 0; i < vImgs; i++) {
				for (int j = 0; j < hImgs; j++) {
					Mat subWnd = scaledImg.rowRange(i*mParams.vStep, i*mParams.vStep + mParams.wHeight)
											.colRange(j*mParams.hStep, j*mParams.hStep + mParams.wWidth);
					windows.push_back(subWnd);
				}
			}

			scaleFactor *=mParams.scaleFactor;
		}

		return windows;
	}
	std::vector<cv::Rect> SlidingWindow::grid(const cv::Mat &_image) const {
		vector<Rect>  grid;

		float scaleFactor = 1;	// Scale factor for each iteration
		for(int k = 0; k < mParams.scaleSteps; k++){
			// Resize image
			Mat scaledImg;
			_image.copyTo(scaledImg);
			resize(_image, scaledImg, Size(), scaleFactor, scaleFactor);

			// Compute number of windows in vertical and horizontal directions.
			int vImgs = (scaledImg.rows - mParams.wHeight)/mParams.vStep + 1;
			int hImgs = (scaledImg.cols - mParams.wWidth) /mParams.hStep + 1;
	

			for (int i = 0; i < vImgs; i++) {
				for (int j = 0; j < hImgs; j++) {
					Rect2i rec(j*mParams.hStep/scaleFactor, i*mParams.vStep/scaleFactor, mParams.wWidth/scaleFactor, mParams.wHeight/scaleFactor);
					grid.push_back(rec);
				}
			}

			scaleFactor*=mParams.scaleFactor;
		}

		return grid;
	}
}	//	namesapce algorithm.