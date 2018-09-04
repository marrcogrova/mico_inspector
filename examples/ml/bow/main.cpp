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
	params.extractorType	= Params::eExtractorType::ORB;
	params.descriptorType	= Params::eDescriptorType::SIFT;
	params.nScalesTrain		= 1;
	params.scaleFactor		= 0.5;
	params.svmType			= cv::ml::SVM::Types::C_SVC;
	params.svmKernel		= cv::ml::SVM::KernelTypes::RBF;
	params.vocSize			= 100;
	params.gamma			= 0.32768;
	params.c				= 5.0625;
	params.nu				= 0.0;


	objDetector.params(params);
	std::string mode(_argv[1]);
	if(mode == "train"){
		std::cout << "Training" << std::endl;
		objDetector.train(	_argv[2],
									_argv[3],
									_argv[4]);
		objDetector.save(_argv[5]);
	}else if(mode == "test") {
		std::cout << "Predict" << std::endl;

		bool singleObject = false;
		std::string typeDetec(_argv[2]);
		if(typeDetec == "single"){
			singleObject = true;
		}

		objDetector.load(_argv[3]);

		VideoCapture camera(_argv[4]);
		Mat frame;
		for (;;) {
			camera >> frame;
			//frame = imread("C:/programming/demo_bagofwords/dataset/datasetTools/cv/screwdriver_sequence/img_" + to_string(index++) + ".jpg", CV_LOAD_IMAGE_GRAYSCALE);
			if (frame.rows != 0) {
				if(singleObject){
					singleImageSingleObject(objDetector, frame);
				}else{
					singleImageMultipleObject(objDetector, frame);
				}
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
	Mat display;
	std::vector<cv::Rect> grid;
	_image.copyTo(display);

	SlidingWindow::Params paramsSw = { 150, 150, 40, 40, 1, 0.5 };
	SlidingWindow sw(paramsSw);
	grid = sw.grid(_image);

	// for (unsigned i = 0; i <grid.size(); i++) {
	// 	cv::Mat dis2 = display.clone();
	// 	cv::rectangle(dis2, grid[i], cv::Scalar(255,0,0), 1);
	// 	cv::imshow("display", dis2);
	// 	waitKey(10);
	// }

	std::vector<std::pair<unsigned, float>> topics = _objDetector.evaluate(_image, grid);
	auto t1 = std::chrono::steady_clock::now();;


	std::cout << "Spent " << std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count() << " seconds evaluating image" << std::endl;

	std::vector<cv::Scalar> colors = {cv::Scalar(255,0,0),cv::Scalar(0,0,255)};
	
	for (unsigned i = 0; i < topics.size(); i++) {
		cv::rectangle(display, grid[i], colors[topics[i].first], topics[i].first==0?3:1);
	}
	cv::imshow("display", display);
	waitKey();

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
