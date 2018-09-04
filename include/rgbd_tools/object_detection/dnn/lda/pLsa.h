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

#ifndef RGBDTOOLS_OBJECTDETECTION_DNN_LDA_ALGORITHM_PLSA_H_
#define RGBDTOOLS_OBJECTDETECTION_DNN_LDA_ALGORITHM_PLSA_H_

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace rgbd {

	class pLsa {
	public:		// Public Interface
		/// Train model with given data.
		void train(const cv::Mat &_wordCoOcurrence, unsigned _nTopics, unsigned _maxIter = 100, float _liChange = 1);

		/// Evaluate given data using internal model.
		cv::Mat evaluate(const cv::Mat &_wordCoOcurrence, unsigned _maxIter = 100, float _liChange = 1);

		/// Store model.
		void load(std::string _path);

		/// Load previous trained model.
		void save(std::string _path);


	private:	// Private methods
		void	randomInit(const unsigned _vocSize, const unsigned _nDocs, const unsigned _nTopics);
		void	eStep();
		void	mStep(const cv::Mat &_wordCoOcurrence);
		void	normalizeProbs();
		float	likelihood(const cv::Mat &_wordCoOcurrence);
		float	iteration(const cv::Mat &_wordCoOcurrence);

	private:	// Members
		cv::Mat					mPw_z;	// P(w|z) Conditional probability of a specific word conditioned on the unobserved class variable z_k
		cv::Mat					mPd_z;	// P(d|z) Conditional probability of a document conditioned on the unobserved class variable _k
		cv::Mat					mPz;	// P(z)	Distribution of unobserved class variable z_k.
		std::vector<cv::Mat>	mPz_dw;	// P(z|d,w) Conditional probability of the unobserver class z_k conditioned on class word and document.

		int mVocabularySize;
		int mNumberOfTopics;

		const double cZeroOffset = 1e-7;	// Constants to avoid numerical problems.
		const double cEpsilon = 2.2204e-16;
	};

}	//	namespace algorithm

#endif	//	_ALGORITHM_PLSA_H_