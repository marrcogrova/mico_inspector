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


#ifndef RGBDTOOLS_OBJECTDETECTION_ML_CLASSIFICATION_TOPICMODELING_LDA_H_
#define RGBDTOOLS_OBJECTDETECTION_ML_CLASSIFICATION_TOPICMODELING_LDA_H_

#include <vector>
#include <rgbd_tools/object_detection/ml/classification/topicModeling/Document.h>

namespace rgbd {
	class Corpus;	// Forward declaration.

	class LDA {
	public:		// Public Interface
		/// Train generative model with given corpus and parametres.
		/// \param _corpus: Set of documents (set of words).
		///	\param _nTopics: number of topic for generative model.
		///	\param _vocabSize: size of vocabulary.
		///	\param _alpha: prior hyperparameter for topic distribution.
		///	\param _beta: prior hyperparameter for word distribution.
		///	\param _iterations: number of iteration to consider convergence.
		void train(Corpus _corpus, unsigned _nTopics, unsigned _vocabSize, double _alpha, double _beta, unsigned _iterations);

		/// Evaluate words using computed generative model.
		/// \param _words: list of indexes of words in the vocabulary.
		/// \return list of topics of words.
		std::vector<std::vector<double>> evaluate(Corpus _corpus, unsigned _nIters = 100);

		/// Store model into textfile.
		void save(std::string _path);

		/// Load model from textfile.
		void load(std::string _path);
	private:
		void initVariables(Corpus _corpus);
		void initCount(Corpus _corpus);

		unsigned sampleZ(unsigned _m, unsigned _n, unsigned _word);

	private:	//	Members
		// Parameters
		unsigned mVocabSize;				// Size of vocabulary.
		unsigned mNumTopics;				// Number of topics.
		double mAlpha;						// Hyper-parameter: Prior of theta distribution.
		double mBeta;						// Hyper-parameter: Prior of phi distribution.

		// Count variables
		std::vector<std::vector<unsigned>> mNwz;	// Count of words related to each topic.
		std::vector<std::vector<unsigned>> mNzm;	// Count of word's topic over each document.
		std::vector<unsigned> mNz;				// Count of word's topic over whole corpus.
		std::vector<unsigned> mNwSum;			// Collapsed mNwz;
		// Initialize Markov Chaizn
		std::vector<std::vector<unsigned>> mZ;

		// Statistics
		std::vector<std::vector<double>> mTheta;	/// Distribution of topics over documents.
		std::vector<std::vector<double>> mPhi;		/// Distribution of words over topics.
	};	// class LDA;

}	//	namespace rgbd

#endif	