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

#include <fstream>
#include <numeric>

#include <mico/base/object_detection/ml/classification/BoW.h>
#include <mico/base/object_detection/ml/classification/topicModeling/Corpus.h>
// #include <opencv2/xfeatures2d.hpp>

using namespace std;
using namespace cv;
using namespace cv::ml;

namespace mico {
	// FAST WRAPER
	class FASTwrapper: public FeatureDetector {
	public:
		static Ptr<FASTwrapper> create() { return Ptr<FASTwrapper>();}
		void detect(InputArray image, std::vector<KeyPoint>& keypoints, InputArray mask) {
			FAST(image, keypoints, 9);
		}
	};


	//-----------------------------------------------------------------------------------------------------------------
	void BoW::params(Params _params) {
		mParams = _params;
	}

	//-----------------------------------------------------------------------------------------------------------------
	BoW::Params BoW::params() const {
		return mParams;
	}

	//-----------------------------------------------------------------------------------------------------------------
	void BoW::model(MlModel & _model) {
		mModel = &_model;
	}

	//-----------------------------------------------------------------------------------------------------------------
	MlModel & BoW::model() const {
		return *mModel;
	}

	//-----------------------------------------------------------------------------------------------------------------
	void BoW::train(std::string _imagePathTemplate, std::string _gtFile) {
		Ptr<ml::TrainData> trainData =  createTrainData(_imagePathTemplate, _gtFile);
		mModel->trainModel(trainData);
	}

	//-----------------------------------------------------------------------------------------------------------------
	std::vector<std::pair<unsigned, float>> BoW::evaluate(Mat _image, vector<Rect> _regions) {
		std::vector<std::pair<unsigned, float>> topics;
		std::vector<Mat> descriptors;
		std::vector<Mat> histograms;
		
		vector<KeyPoint> keypoints;
		Mat imageDescriptors = computeFeatures(_image, keypoints);
		

		for (Rect region : _regions) {
			Mat regionDescriptors;
			for (unsigned i = 0; i < keypoints.size(); i++) {
				if (region.contains(keypoints[i].pt)) {
					regionDescriptors.push_back(imageDescriptors.row(i));
				}
			}
			descriptors.push_back(regionDescriptors);
		}

		if(descriptors.size() == 0)
			return topics;

		vectorQuantization(descriptors, histograms);
		Mat newData;
		for (Mat histogram : histograms) {
			double max;
			minMaxLoc(histogram, NULL, &max);
			histogram = histogram / max;

			Mat rotated;
			transpose(histogram, rotated);
			newData.push_back(rotated);
		}
		Mat results;
		mModel->predict(newData, results);
		for (unsigned i = 0; i < results.rows; i++) {
			topics.push_back(std::pair<unsigned, float>(results.at<float>(i,1),results.at<float>(i,0)));
		}
		return topics;
	}

	//-----------------------------------------------------------------------------------------------------------------
	void BoW::save(std::string _name) {
		FileStorage fs(_name + ".yml", FileStorage::WRITE);
		fs << "vocabulary" << mCodebook;
		fs.release();
		mModel->save(_name);
	}

	//-----------------------------------------------------------------------------------------------------------------
	void BoW::load(std::string _name) {
		FileStorage fs(_name + ".yml", FileStorage::READ);
		fs["vocabulary"] >> mCodebook;
		fs.release(); 
		mModel->load(_name);
	}

	//-----------------------------------------------------------------------------------------------------------------
	cv::Mat BoW::loadImage(std::string _filePatern, unsigned _index){
		unsigned imageId = _index;
		string fileName = _filePatern.substr(0, _filePatern.find("%d")) + to_string(_index) + _filePatern.substr(_filePatern.find("%d")+2, _filePatern.length());
		Mat img = imread(fileName, 0);	//	Load image using file path and as grayscale image.

		if(img.rows == 0)
			return img;

		double factor = 300.0/(img.rows > img.cols ? img.rows: img.cols);
		resize(img, img, Size(), factor, factor);
		return img;

	}

	//-----------------------------------------------------------------------------------------------------------------
	Ptr<ml::TrainData> BoW::createTrainData(const std::string &_imagePathTemplate, const std::string &_gtFile) {
		Mat descriptorsAll;
		vector<Mat> descriptorPerImg;
		unsigned index = 1;
		cv::VideoCapture imgStream(_imagePathTemplate);
		for (;;) {	// For each image in dataset
			Mat frame;// = loadImage(_imagePathTemplate, index++);
			imgStream>>frame;
			if (frame.rows == 0) {
				break;	// Can't open image.
			}
			double factor = 300.0/(frame.rows > frame.cols ? frame.rows: frame.cols);
			resize(frame, frame, Size(), factor, factor);
			double scaleFactor = 1;
			for (unsigned i = 0; i < mParams.nScalesTrain; i++) {
				Mat resizedImage;
				resize(frame, resizedImage, Size(), scaleFactor, scaleFactor);

				// Look for interest points to compute features in there.
				vector<KeyPoint> keypoints;
				Mat descriptors = computeFeatures(resizedImage, keypoints);

				Mat descriptors_32f;
				descriptors.convertTo(descriptors_32f, CV_32F, 1.0 / 255.0);

				descriptorsAll.push_back(descriptors_32f);
				descriptorPerImg.push_back(descriptors_32f);
				scaleFactor*=mParams.scaleFactor;
			}
		}
		// Form codebook from all descriptors
		mCodebook = formCodebook(descriptorsAll);
		descriptorsAll.release();

		// Compute histograms
		std::vector<cv::Mat> histograms;
		vectorQuantization(descriptorPerImg, histograms);

		// TRAIN DATA
		Mat X(histograms.size(), mCodebook.rows, CV_32FC1);
		for (int i = 0; i < histograms.size(); i++) {
			Mat rotated;
			transpose(histograms[i], rotated);
			rotated.copyTo(X.row(i));
		}

		std::cout << X.rows << ", " << X.cols << std::endl;

		Mat groundTruth = loadGroundTruth(_gtFile);
		std::cout << groundTruth.rows << ", " << groundTruth.cols << std::endl;

		return ml::TrainData::create(X, ml::SampleTypes::ROW_SAMPLE, groundTruth);
	}

	//-----------------------------------------------------------------------------------------------------------------
	Mat BoW::computeFeatures(const Mat &_frame, vector<KeyPoint> &_keypoints) {
		Ptr<FeatureDetector> detector;
		Ptr<FeatureDetector> descriptor;

		switch (mParams.extractorType) {
		case Params::eExtractorType::SIFT:
			// detector = xfeatures2d::SIFT::create();
			assert(false);
			break;
		case Params::eExtractorType::SURF:
			// detector = xfeatures2d::SURF::create();
			assert(false);
			break;
		case Params::eExtractorType::FAST:
			detector = FASTwrapper::create();
			break;
		case Params::eExtractorType::ORB:
			detector = ORB::create();
			break;
		}

		switch (mParams.descriptorType) {
		case Params::eDescriptorType::SIFT:
			// descriptor = xfeatures2d::SIFT::create();
			assert(false);
			break;
		case Params::eDescriptorType::SURF:
			// descriptor = xfeatures2d::SURF::create();
			assert(false);
			break;
		case Params::eDescriptorType::ORB:
			descriptor = ORB::create();
			break;
		}

		Mat descriptors;
		detector->detect(_frame, _keypoints);
		descriptor->compute(_frame, _keypoints, descriptors);

		return descriptors;
	}

	//-----------------------------------------------------------------------------------------------------------------
	Mat BoW::formCodebook(const Mat &_descriptors) {
		Mat codebook;
		int codebookSize = mParams.vocSize;
		TermCriteria criteria(1, 100, 0.001);
		vector<int> labels;
		std::cout << "Number of descriptors " << _descriptors.rows << std::endl;
		std::cout << "Performing kmeans" << std::endl;
		kmeans(_descriptors, codebookSize, labels, criteria, 1,KMEANS_PP_CENTERS, codebook);
		//std::cout << "Codebook size " << codebook.rows << std::endl;
		return codebook;
	}

	//-----------------------------------------------------------------------------------------------------------------
	void BoW::vectorQuantization(const std::vector<cv::Mat> &_descriptors, std::vector<cv::Mat> &_histograms) {
		std::cout << "Performing vector quantization" << std::endl;
		for (Mat mat : _descriptors) {
			_histograms.push_back(Mat(mCodebook.rows, 1 , CV_32F, Scalar(0)));
			for (int i = 0; i < mat.rows; i++) {
				double minDist = 999999;
				unsigned index = 0;

				// Compute dist of descriptor i to centroid j;
				for (int j = 0; j < mCodebook.rows; j++) {
					double dist = 0;
					for (int k = 0; k < mCodebook.cols; k++) {
						double a = mCodebook.at<float>(j,k) - mat.at<float>(i,k);
						dist += a*a;
					}
					//dist = sqrt(dist);
					if (dist < minDist) {
						minDist = dist;
						index = j;
					}
				}
				_histograms[_histograms.size()-1].at<float>(index) += 1;
			}
			int recount = (int) sum(_histograms[_histograms.size() -1])[0];
			assert(recount == mat.rows);
		}

		// Normalize histograms
		for (cv::Mat &histogram : _histograms) {
			double max;
			minMaxLoc(histogram, NULL, &max);
			histogram = histogram / max;
		}

		FileStorage histogramsFile("histogramPerImg.yml", FileStorage::WRITE);
		for (unsigned i = 0; i < _histograms.size(); i++) {
			histogramsFile << "hist_"+to_string(i) << _histograms[i];
		}

		std::cout << "Endl vector quantization" << std::endl;
	}

	//-----------------------------------------------------------------------------------------------------------------
	Mat BoW::loadGroundTruth(const std::string & _gtFile) const{
		Mat groundTruth(0, 1, CV_32SC1);
		ifstream gtFile(_gtFile);
		assert(gtFile.is_open());
		for (unsigned i = 0; !gtFile.eof();i++) {
			int gtVal;
			gtFile >> gtVal;
			for (unsigned j = 0; j < mParams.nScalesTrain; j++) {
				groundTruth.push_back(gtVal);
			}

		}
		return groundTruth;
	}

	//-----------------------------------------------------------------------------------------------------------------
	// MODELS
	//-----------------------------------------------------------------------------------------------------------------

	// SVM MODEL
	void SvmModel::setParams(double _c, double _gamma, cv::ml::SVM::Types _svmType, cv::ml::SVM::KernelTypes _svmKernel, bool _autoTrain) {
		mSvm = SVM::create();
		mSvm->setType(_svmType);
		mSvm->setKernel(_svmKernel);
		mAutoTrain = _autoTrain;
		if (!_autoTrain) {
			mSvm->setGamma(_gamma);
			mSvm->setC(_c);
		}
		
	}

	//-----------------------------------------------------------------------------------------------------------------
	void SvmModel::trainModel(const cv::Ptr<cv::ml::TrainData>& _trainData) {
		if (mAutoTrain) {
			mSvm->trainAuto(_trainData, 10, ParamGrid(1,1000,1.5), ParamGrid(0.00001,1,2));
		}
		else {
			mSvm->train(_trainData);
		}
	}

	//-----------------------------------------------------------------------------------------------------------------
	void SvmModel::predict(const cv::Mat & _newData, cv::Mat & _result) {
		mSvm->predict(_newData, _result);
	}

	//-----------------------------------------------------------------------------------------------------------------
	void SvmModel::save(const std::string _name){
		mSvm->save(_name + ".svm");
	}

	//-----------------------------------------------------------------------------------------------------------------
	void SvmModel::load(const std::string _name) {
		mSvm = Algorithm::load<SVM>(_name + ".svm");
	}


	// LDA MODEL
	void LdaModel::setParams(double _alpha, double _beta) {
		mAlpha = _alpha;
		mBeta = _beta;
	}

	//-----------------------------------------------------------------------------------------------------------------
	void LdaModel::trainModel(const cv::Ptr<cv::ml::TrainData>& _trainData) {
		mico::Corpus corpus;

		Mat samples = _trainData->getSamples();

		for (int i = 0; i < samples.rows; i++) {
			mico::Document doc;
			Mat row = samples.row(i);
			double minNonZero = 2;
			for (unsigned i = 0; i < row.cols; i++) {
				if (minNonZero > row.at<float>(i) && row.at<float>(i) != 0) {
					minNonZero = row.at<float>(i);
				}
			}
			row = row/minNonZero;
			for (int j = 0; j < samples.cols; j++) {
				for (int k = 0; k < row.at<float>(j); k++) {
					doc.addWord(j);
				}
			}
			doc.shuffle();
			corpus.addDocument(doc);
		}

		mModel.train(corpus, _trainData->getClassLabels().rows, samples.cols, mAlpha, mBeta, 200);
	}

	//-----------------------------------------------------------------------------------------------------------------
	void LdaModel::predict(const cv::Mat & _newData, cv::Mat & _result) {
		vector<bool> validData;

		mico::Corpus corpus;
		for (int i = 0; i < _newData.rows; i++) {
			mico::Document doc;
			Mat row = _newData.row(i);
			double minNonZero = 2;
			for (unsigned i = 0; i < row.cols; i++) {
				if (isnan(row.at<float>(i)) || isinf(row.at<float>(i))) {
					validData.push_back(false);
					break;
				}
				if (minNonZero > row.at<float>(i) && row.at<float>(i) != 0) {
					minNonZero = row.at<float>(i);
				}
			}
			if (validData.size () - 1 == i) {	// Bad data
				continue;
			}
			else {
				validData.push_back(true);
			}

			row = row/minNonZero;
			for (int j = 0; j < row.cols; j++) {
				for (int k = 0; k < row.at<float>(j); k++) {
					doc.addWord(k);
				}
			}
			doc.shuffle();
			corpus.addDocument(doc);
		}

		vector<vector<double>> probs  = mModel.evaluate(corpus);

		_result = Mat(validData.size(), 2, CV_32F);
		int probIndex = 0;
		for (unsigned i = 0; i < validData.size(); i++) {
			if (validData[i]) {
				unsigned maxIndex = 0;
				double maxProb = 0.0;
				for (unsigned j = 0; j < probs[probIndex].size(); j++) {
					if (probs[probIndex][j] > maxProb) {
						maxIndex = j;
						maxProb = probs[probIndex][j];
					}
				}
				_result.at<float>(i, 0) = maxProb;
				_result.at<float>(i, 1) = maxIndex;
				probIndex++;
			}
			else {
				_result.at<float>(i, 0) = 0.0;
				_result.at<float>(i, 1) = 1;
			}
		}
	}

	//-----------------------------------------------------------------------------------------------------------------
	void LdaModel::save(const std::string _name) {
		mModel.save(_name);
	}

	//-----------------------------------------------------------------------------------------------------------------
	void LdaModel::load(const std::string _name) {
		mModel.load(_name);
	}

}	// namespace algorithm