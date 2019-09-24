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

#ifndef MICO_BASE_OBJECTDETECTION_ML_CLASSIFICATION_TOPICMODELING_DOCUMENTS_H_
#define MICO_BASE_OBJECTDETECTION_ML_CLASSIFICATION_TOPICMODELING_DOCUMENTS_H_

#include <vector>
#include <algorithm>

namespace mico {
	class Document {
	public:
		/// Default constructor;
		Document() {};

		/// Construct a document with a list of words
		Document(std::vector<unsigned> _words) : mWords(_words) {};

		/// Add new word to document
		void addWord(unsigned _word) { mWords.push_back(_word); };

		/// Shuffle word list.
		void shuffle() { std::random_shuffle(mWords.begin(), mWords.end()); };

		/// Get a copy of word list
		std::vector<unsigned>	words()	const { return mWords; };

		/// Access to a single word
		int word(unsigned _index) const { return mWords[_index]; };

		/// Get number of words in document.
		unsigned			lenght() { return mWords.size(); };
	private:
		std::vector<unsigned> mWords;
	};	// class Document

}	//	namespace mico 
#endif	