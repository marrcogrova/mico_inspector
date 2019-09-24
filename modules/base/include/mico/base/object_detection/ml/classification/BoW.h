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


#ifndef MICO_BASE_OBJECTDETECTION_ML_CLASSIFICATION_BOW_H_
#define MICO_BASE_OBJECTDETECTION_ML_CLASSIFICATION_BOW_H_

#include <array>
#include <cassert>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <mico/base/object_detection/ml/classification/topicModeling/lda.h>

namespace mico {
	//-----------------------------------------------------------------------------------------------------------------
	class MlModel {
	public:
		// Train learning model
		virtual void trainModel(const cv::Ptr<cv::ml::TrainData> &_trainData) = 0;
		// Get a prediction using learned model
		virtual void predict(const cv::Mat &_newData, cv::Mat &_result) = 0;
		// Save Model
		virtual void save(const std::string _name) = 0;
		// Load Model
		virtual void load(const std::string _name) = 0;
	};

	// Examples of models
	class SvmModel : public MlModel {
	public:
		void setParams(double _c, double _gamma, cv::ml::SVM::Types _svmType, cv::ml::SVM::KernelTypes _svmKernel, bool _autoTrain = false);
		void trainModel(const cv::Ptr<cv::ml::TrainData> &_trainData);
		void predict(const cv::Mat &_newData, cv::Mat &_result);

		void save(const std::string _name);
		void load(const std::string _name);

	private:
		cv::Ptr<cv::ml::SVM> mSvm;
		bool mAutoTrain;
	};

	//-----------------------------------------------------------------------------------------------------------------
	class LdaModel : public MlModel {
	public:
	public:
		void setParams(double _alpha, double _beta);
		void trainModel(const cv::Ptr<cv::ml::TrainData> &_trainData);
		void predict(const cv::Mat &_newData, cv::Mat &_result);

		void save(const std::string _name);
		void load(const std::string _name);

	private:
		mico::LDA mModel;
		double mAlpha, mBeta;
	};

	/// Base object detector class.
	class BoW {
	public:	// Public interface
			// Somewhere, set:
			//		a) Finder of Regions of interest
			//		b) Feature detetor (SIFT, or similar...)
			//		c) Model of objects

		struct Params {
			enum class eExtractorType {SIFT, SURF, FAST, ORB};
			enum class eDescriptorType {SIFT, SURF, ORB};

			unsigned imageSizeTrain;
			unsigned nScalesTrain;
			double scaleFactor;
			eExtractorType extractorType;
			eDescriptorType descriptorType;

			unsigned vocSize;
		};

		/// Set parameters for bow model
		void params(Params _params);
		
		/// Get parameters of bow model;
		Params params() const;

		/// Set learning model
		void model(MlModel &_model);

		/// Get instance of learned model
		MlModel &model() const;

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
	private:	// Private members
		cv::Mat mCodebook;
		Params mParams;
		MlModel *mModel;
	};
}

#endif	
