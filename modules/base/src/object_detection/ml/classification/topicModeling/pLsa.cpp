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

#include <mico/base/object_detection/ml/classification/topicModeling/pLsa.h>

using namespace cv;

namespace mico {
	//-----------------------------------------------------------------------------------------------------------------
	// Public iterface
	void pLsa::train(const cv::Mat &_wordCoOcurrence, unsigned _nTopics,unsigned _maxIter, float _liChange) {
		randomInit(_wordCoOcurrence.rows, _wordCoOcurrence.cols, _nTopics);

		double lastL = 99999;
		for (unsigned iter = 0; iter < _maxIter; iter++) {
			float L = iteration(_wordCoOcurrence);
			std::cout << L << std::endl;

			// Check convergency
			if (iter > 1 && abs(lastL - L) < _liChange) {
				break;
			}
			lastL = L;
		}

	}

	//-----------------------------------------------------------------------------------------------------------------
	cv::Mat pLsa::evaluate(const cv::Mat &_wordCoOcurrence, unsigned _maxIter, float _liChange) {
		randomInit(_wordCoOcurrence.rows, _wordCoOcurrence.cols, mNumberOfTopics);

		float lastL = 99999;
		Mat fixed_Pw_z;
		mPw_z.copyTo(fixed_Pw_z);

		for (unsigned iter = 0; iter < _maxIter; iter++) {
			float L = iteration(_wordCoOcurrence);
			std::cout << L << std::endl;
			
			fixed_Pw_z.copyTo(mPw_z);

			// Check convergency
			if (iter > 1 && abs(lastL - L) < _liChange) {
				break;
			}
			lastL = L;
		}

		return mPd_z;
	}

	//-----------------------------------------------------------------------------------------------------------------
	void pLsa::load(std::string _path) {
		FileStorage plsaFile(_path, FileStorage::READ);
		plsaFile["Pw_z"] >> mPw_z;
		plsaFile["Pd_z"] >> mPd_z;
		plsaFile["Pz"] >>	mPz;

		mVocabularySize = mPw_z.rows;
		mNumberOfTopics = mPw_z.cols;
	}

	//-----------------------------------------------------------------------------------------------------------------
	void pLsa::save(std::string _path) {
		FileStorage plsaFile(_path, FileStorage::WRITE);
		plsaFile << "Pw_z" << mPw_z;
		plsaFile << "Pd_z" << mPd_z;
		plsaFile << "Pz" << mPz;
	}

	//-----------------------------------------------------------------------------------------------------------------
	// Private methods
	void pLsa::randomInit(const unsigned _vocSize, const unsigned _nDocs, const unsigned _nTopics) {
		mVocabularySize = _vocSize;
		mNumberOfTopics = _nTopics;

		// Initialize conditional probabilities for EM
		mPz = Mat(1,mNumberOfTopics, CV_32F, Scalar(1.0));	// P(z)
		mPz = mPz/mNumberOfTopics;

		Mat C;

		// P(d|z)
		mPd_z = Mat(_nDocs,mNumberOfTopics, CV_32F);
		randu(mPd_z, Scalar::all(0.0), Scalar::all(1.0));	// random initialization
		
		#ifdef HAS_OPENCV_3
		reduce(mPd_z, C, 1, CV_REDUCE_SUM);
		#elif HAS_OPENCV_4
		reduce(mPd_z, C, 1, cv::ReduceTypes::REDUCE_SUM);
		#endif

		for (int i = 0; i < mNumberOfTopics; i++) {
			mPd_z.col(i) = mPd_z.col(i).mul(1/C);
		}

		// P(w|z) 
		mPw_z = Mat(mVocabularySize, mNumberOfTopics, CV_32F);	
		randu(mPw_z, Scalar::all(0.0), Scalar::all(1.0));	// random initialization
		
		#ifdef HAS_OPENCV_3
		reduce(mPw_z, C, 1, CV_REDUCE_SUM);
		#elif HAS_OPENCV_4
		reduce(mPw_z, C, 1, cv::ReduceTypes::REDUCE_SUM);
		#endif

		for (int i = 0; i < mNumberOfTopics; i++) {
			mPw_z.col(i) = mPw_z.col(i).mul(1/C);
		}

		// P(z|d,w)
		mPz_dw.clear();
		for (int i = 0; i < mNumberOfTopics; i++) {
			mPz_dw.push_back(Mat(mVocabularySize,_nDocs,CV_32F, Scalar(0)));
		}

	}

	//-----------------------------------------------------------------------------------------------------------------
	void pLsa::eStep() {
		for (int topic = 0; topic < mNumberOfTopics; topic++) {
			for (int doc = 0; doc < mPd_z.rows; doc++) {
				for (int word = 0; word < mVocabularySize ; word++) {
					float num = mPw_z.at<float>(word, topic)*mPd_z.at<float>(doc, topic);
					float den = 0;
					for (int k = 0; k < mNumberOfTopics; k++) {
						den += mPw_z.at<float>(word, k)*mPd_z.at<float>(doc, k);
					}

					mPz_dw[topic].at<float>(word, doc) = num/den;
				}
			}
		}
	}

	//-----------------------------------------------------------------------------------------------------------------
	void pLsa::mStep(const cv::Mat &_wordCoOcurrence) {
		//-- Update word prob 
		for (int topic = 0; topic < mNumberOfTopics; topic++) {
			for (int word = 0; word < mVocabularySize ; word++) {
				float num = 0;
				for (int i = 0; i < mPd_z.rows ; i++) {
					[[maybe_unused]] float num1 = _wordCoOcurrence.at<float>(word,i);
					[[maybe_unused]] float num2 = mPz_dw[topic].at<float>(word, i);
					num += _wordCoOcurrence.at<float>(word,i)*mPz_dw[topic].at<float>(word, i);
				}

				float den = 0;
				for (int j = 0; j < mVocabularySize; j++) {
					for (int i = 0; i < mPd_z.rows; i++) {
						den += _wordCoOcurrence.at<float>(j, i)*mPz_dw[topic].at<float>(j, i);
					}
				}
				mPw_z.at<float>(word, topic) = num/den;
			}
		}

		//-- Update topic prob
		for (int topic = 0; topic < mNumberOfTopics; topic++) {
			for (int doc = 0; doc < mPd_z.rows ; doc++) {
				float num = 0;
				for (int j = 0; j < mVocabularySize; j++) {
					num += _wordCoOcurrence.at<float>(j, doc)*mPz_dw[topic].at<float>(j, doc);
				}


				float den = 0;
				for (int j = 0; j < mVocabularySize; j++) {
					den += _wordCoOcurrence.at<float>(j, doc);
				}

				mPd_z.at<float>(doc, topic) = num/den;
			}
		}
	}

	//-----------------------------------------------------------------------------------------------------------------
	void pLsa::normalizeProbs() {
		Mat C(mPz_dw[0].rows, mPz_dw[0].cols, CV_32F, Scalar(0.0));
		for (Mat mat : mPz_dw) {
			C += mat;
		}
		for (int k = 0; k < mNumberOfTopics; k++) {
			mPz_dw[k] = mPz_dw[k].mul(1/C);
		}

		#ifdef HAS_OPENCV_3
		reduce(mPw_z, C, 1, CV_REDUCE_SUM);
		#elif HAS_OPENCV_4
		reduce(mPw_z, C, 1, cv::ReduceTypes::REDUCE_SUM);
		#endif
		for (int i = 0; i < mNumberOfTopics; i++) {
			mPw_z.col(i) = mPw_z.col(i).mul(1/C);
		}

		#ifdef HAS_OPENCV_3
		reduce(mPw_z, C, 1, CV_REDUCE_SUM);
		#elif HAS_OPENCV_4
		reduce(mPw_z, C, 1, cv::ReduceTypes::REDUCE_SUM);
		#endif
		for (int i = 0; i < mNumberOfTopics; i++) {
			mPd_z.col(i) = mPd_z.col(i).mul(1/C);
		}

		#ifdef HAS_OPENCV_3
		reduce(mPw_z, C, 1, CV_REDUCE_SUM);
		#elif HAS_OPENCV_4
		reduce(mPw_z, C, 1, cv::ReduceTypes::REDUCE_SUM);
		#endif
		mPz = mPz/C.at<float>(0);

	}

	//-----------------------------------------------------------------------------------------------------------------
	float pLsa::likelihood(const cv::Mat &_wordCoOcurrence) {
		float L = 0;
		for (int doc = 0 ; doc < mPd_z.rows ; doc++) {
			for (int word = 0; word < mVocabularySize; word++) {
				float n = _wordCoOcurrence.at<float>(word, doc);
				float aux = 0;
				for (int topic = 0; topic < mNumberOfTopics; topic++) {
					aux += mPz.at<float>(topic)*mPd_z.at<float>(doc, topic)*mPw_z.at<float>(word, topic);
				}
				L +=n*aux;
			}
		}

		return L;
	}
	//-----------------------------------------------------------------------------------------------------------------
	float pLsa::iteration(const cv::Mat &_wordCoOcurrence) {
		//std::cout << mPd_z << std::endl;
		//std::cout << mPz << std::endl;
		//std::cout << mPw_z << std::endl;
		//std::cout << _wordCoOcurrence << std::endl;

		eStep();
		mStep(_wordCoOcurrence);

		#ifdef HAS_OPENCV_3
		reduce(mPd_z, mPz, 0, CV_REDUCE_SUM);
		#elif HAS_OPENCV_4
		reduce(mPd_z, mPz, 0, cv::ReduceTypes::REDUCE_SUM);
		#endif

		normalizeProbs();
		
		return likelihood(_wordCoOcurrence);
	}
	//-----------------------------------------------------------------------------------------------------------------

}	//	namespace algorithm