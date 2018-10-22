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

#ifndef RGBDTOOLS_OBJECTDETECTION_ML_CLASSFICATION_TOPICMODELING_CORPUS_H_
#define RGBDTOOLS_OBJECTDETECTION_ML_CLASSFICATION_TOPICMODELING_CORPUS_H_

#include <rgbd_tools/object_detection/ml/classification/topicModeling/Document.h>

#include <vector>

namespace rgbd {
	class Corpus {
	public:
		/// Add a document to the corpus
		/// \param _document: document to be added to the corpus
		void addDocument(Document _document) { mDocuments.push_back(_document); };

		/// Get count of total words in document.
		/// \return Sum of all words in all documents
		unsigned totalWords() {
			unsigned numWords = 0;
			for (Document doc : mDocuments) {
				numWords += doc.lenght();
			}
			return numWords;
		};

		/// Get number of documents in the corpus
		/// \return Number of documents in the corpus
		unsigned numDocs() { return mDocuments.size(); };

		/// Access to a document
		/// \param index to the desired document.
		Document document(unsigned _index) { return mDocuments[_index]; };

		/// Shuffle word list.
		void shuffle(){ std::random_shuffle(mDocuments.begin(), mDocuments.end()); };

	private:
		std::vector<Document> mDocuments;
	};	//	class Corpus

}	//	namespace rgbd

#endif	