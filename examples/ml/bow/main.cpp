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
//	Single object detection program using "bag of words" paradigm based on code learned from ICCV short course example
//	References.



#include <fstream>
#include <chrono>
#include <rgbd_tools/object_detection/dnn/bow/BoW.h>
#include <rgbd_tools/object_detection/dnn/SlidingWindow.h>


using namespace rgbd;
using namespace cv;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------
void singleImageMultipleObject(BoW _objDetector, cv::Mat _image);
void singleImageSingleObject(BoW _objDetector, cv::Mat _image);
void filterPredictions(vector<vector<Rect>> &_predictions);

//---------------------------------------------------------------------------------------------------------------------
int main(int _argc, char ** _argv) {
	// Create and train detector.
	BoW objDetector;

	Params params;

	params.imageSizeTrain	= 300;
	params.extractorType	= Params::eExtractorType::SIFT;
	params.descriptorType	= Params::eDescriptorType::SIFT;
	params.nScalesTrain		= 1;
	params.scaleFactor		= 0.5;
	params.svmType			= cv::ml::SVM::Types::C_SVC;
	params.svmKernel		= cv::ml::SVM::KernelTypes::RBF;
	params.vocSize			= 300;
	params.gamma			= 0.16384;
	params.c				= 2.25;

	objDetector.params(params);
	
	if(std::string(_argv[1]) == "train"){
		std::cout << "Training" << std::endl;
		objDetector.train(	_argv[2],
							_argv[3]);
		objDetector.save(_argv[4]);
	}else if(std::string(_argv[1]) == "test") {
		std::cout << "Predict" << std::endl;
		objDetector.load(_argv[2]);

		VideoCapture camera(_argv[3]);
		Mat frame;
		for (;;) {
			camera >> frame;
			//frame = imread("C:/programming/demo_bagofwords/dataset/datasetTools/cv/screwdriver_sequence/img_" + to_string(index++) + ".jpg", CV_LOAD_IMAGE_GRAYSCALE);
			if (frame.rows != 0) {
				resize(frame, frame, Size(), 0.5,0.5);
				singleImageSingleObject(objDetector, frame);
			}
			else {
				break;
			}
		}
	}
}

//---------------------------------------------------------------------------------------------------------------------
void singleImageSingleObject(BoW _objDetector,  cv::Mat _image) {
	auto t0 =std::chrono::steady_clock::now();
	Mat dis;
	std::vector<cv::Rect> grid;
	_image.copyTo(dis);

	grid.push_back(cv::Rect(0,0,_image.rows,_image.cols));

	std::vector<std::pair<unsigned, float>> topics = _objDetector.evaluate(_image, grid);
	auto t1 = std::chrono::steady_clock::now();;

	std::cout << "Spent " << std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count() << " ms evaluating image" << std::endl;

	std::vector<cv::Scalar> colors = {cv::Scalar(255,0,0),cv::Scalar(0,255,0),cv::Scalar(0,0,255),cv::Scalar(255,0,255)};


	std::cout << "type: " << topics[0].first << ". Prob: " << topics[0].second << std::endl;

	cv::imshow("display", _image);
	waitKey();
}


//---------------------------------------------------------------------------------------------------------------------
void singleImageMultipleObject(BoW _objDetector,  cv::Mat _image) {
	auto t0 =std::chrono::steady_clock::now();
	Mat dis;
	std::vector<cv::Rect> grid;
	_image.copyTo(dis);

	SlidingWindow::Params paramsSw = { 70, 40, 35, 20, 1, 0.5 };
	SlidingWindow sw(paramsSw);
	grid = sw.grid(_image);

	std::vector<std::pair<unsigned, float>> topics = _objDetector.evaluate(_image, grid);
	auto t1 = std::chrono::steady_clock::now();;


	std::cout << "Spent " << std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count() << " seconds evaluating image" << std::endl;

	std::vector<cv::Scalar> colors = {cv::Scalar(255,0,0),cv::Scalar(0,255,0),cv::Scalar(0,0,255),cv::Scalar(255,0,255)};
	Mat display;
	cvtColor(_image, display, CV_GRAY2RGB);
	vector<vector<Rect>> predictions(4);
	for (unsigned i = 0; i < grid.size(); i++) {
		if (topics[i].second > 0.7) {
			// Store rectangles with higher probabilities
			predictions[topics[i].first].push_back(grid[i]);
		}
	}

	filterPredictions(predictions);

	for (unsigned topic = 0; topic < predictions.size(); topic++) {
		for (unsigned i = 0; i <predictions[topic].size(); i++) {
			cv::rectangle(display, predictions[topic][i], colors[topic], 3);
		}
	}

	cv::imshow("display", display);
	waitKey(3);
}

void filterPredictions(vector<vector<Rect>>& _predictions) {
	for (vector<Rect> &list : _predictions) {
		for (int i = 0; i < list.size(); i++) {
			for (int j = i+1; j < list.size(); j++) {
				double areaUnion = (list[i]&list[j]).area();
				double areaDifference = list[i].area() + list[j].area() - areaUnion;

				if (areaUnion / areaDifference > 0.14) {

					Rect aux = list[j] | list[i];
					list.erase(list.begin() + j);
					list.erase(list.begin() + i);
					list.push_back(aux);
					i = -1;
					break;
				}
			}
		}
	}
}
