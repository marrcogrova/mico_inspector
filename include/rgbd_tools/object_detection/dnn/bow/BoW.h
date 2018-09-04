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

#ifndef RGBDTOOLS_OBJECTDETECTION_DNN_LDA_ALGORITHM_BOW_H_
#define RGBDTOOLS_OBJECTDETECTION_DNN_LDA_ALGORITHM_BOW_H_

#include <array>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>


namespace rgbd {
	struct Params {
		enum class eExtractorType {SIFT, SURF, FAST, ORB};
		enum class eDescriptorType {SIFT, SURF, ORB};

		unsigned imageSizeTrain;
		unsigned nScalesTrain;
		double scaleFactor;
		eExtractorType extractorType;
		eDescriptorType descriptorType;

		unsigned vocSize;
		cv::ml::SVM::Types svmType;
		cv::ml::SVM::KernelTypes svmKernel;
		double gamma;
		double c;
	};

	/// Base object detector class.
	class BoW {
	public:	// Public interface
			// Somewhere, set:
			//		a) Finder of Regions of interest
			//		b) Feature detetor (SIFT, or similar...)
			//		c) Model of objects

			/// Set parameters for bow model
		void params(Params _params);

		/// Train model with given dataset "/path/to/images%d.jpg"
		void train(std::string _imagePathTemplate, std::string _gtFile);

		/// Evaluate input
		std::vector<std::pair<unsigned, float>> evaluate(cv::Mat _image, std::vector<cv::Rect> _regions);

		/// Save model
		void save(std::string _name);

		/// Load model
		void load(std::string _name);

	private:	// Private methods
				// Randomize order of set in order to improve train performance.
				//void randomizeSet();		Supposed that set is randomly acquired.

				// Normalize input set (Size of images, number of images, etc...)
				//void normalizeSet();		Supposed that images in data set have same size and are of the same type.

				// Load image
		cv::Mat loadImage(std::string _filePatern, unsigned index);

		// Create train data.
		cv::Ptr<cv::ml::TrainData> createTrainData(const std::string &_imagePathTemplate, const std::string &_gtFile);

		// Find regions of interest in the given image and Compute features on given regions.
		cv::Mat computeFeatures(const cv::Mat &_frame,std::vector<cv::KeyPoint> &_keypoints);

		// Form codebook from given descriptors. Basically runs kmeans on data and set as vocabulary the centroids of the clusters.
		cv::Mat formCodebook(const cv::Mat &_descriptors);

		// Get descriptors and form histograms using precomputed vocabulary.
		void vectorQuantization(const std::vector<cv::Mat> &_descriptors, std::vector<cv::Mat> &_histograms);

		// Load ground truth
		cv::Mat loadGroundTruth(const std::string &_gtFile) const;

		// Train learning model
		void train(const cv::Ptr<cv::ml::TrainData> &_trainData);

		// Get a prediction using learned model
		void predict(const cv::Mat &_newData, float& _label, float &_prob);

	private:	// Private members
		cv::Mat mCodebook;
		//pLsa model;
		//BOViL::algorithms::LDA model;
		cv::Ptr<cv::ml::SVM> mSvm;

		Params mParams;
	};
}

#endif	// 
