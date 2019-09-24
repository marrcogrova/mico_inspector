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

#include <mico/base/segmentation/color_clustering/types/SegmentedRLEObject.h>

#include <algorithm>    // std::sort
#include <vector>


namespace mico {
	SegmentedRLEObject::SegmentedRLEObject(LineRLE ini) {
		mUpperLeft.x = 9999;
		mDownRight.x = 0;
		mUpperLeft.y = 9999;
		mDownRight.y = 0;
		size = 0;
		color = ini.color;
		addLineObjRLE(ini);

	}

	void SegmentedRLEObject::addLineObjRLE(LineRLE aux) {
		obj.push_back(aux);

		if (mUpperLeft.y > aux.i)
			mUpperLeft.y = aux.i;
		if (mUpperLeft.x > aux.js)
			mUpperLeft.x = aux.js;
		if (mDownRight.y < aux.i)
			mDownRight.y = aux.i;
		if (mDownRight.x < aux.je)
			mDownRight.x = aux.je;
		size += aux.size + 1;
	}

	void SegmentedRLEObject::addRLEFamily(SegmentedRLEObject& family) {
		for (int k = 0; k < family.getLines(); k++) {
			LineRLE aux = family.getRLEObj(k);
			obj.push_back(aux);
			if (mUpperLeft.y > aux.i)
				mUpperLeft.y = aux.i;
			if (mUpperLeft.x > aux.js)
				mUpperLeft.x = aux.js;
			if (mDownRight.y < aux.i)
				mDownRight.y = aux.i;
			if (mDownRight.x < aux.je)
				mDownRight.x = aux.je;
			size += aux.size + 1;
		}
	}

	int SegmentedRLEObject::getLines() const {
		return obj.size();
	}

	LineRLE SegmentedRLEObject::getRLEObj(int k) const {
		return obj[k];
	}

	Vec2i SegmentedRLEObject::upperLeft() const {
		return mUpperLeft;
	}

	Vec2i SegmentedRLEObject::downRight() const {
		return mDownRight;
	}

	unsigned int SegmentedRLEObject::getColor() const {
		return color;
	}

	unsigned int SegmentedRLEObject::getSize() const {
		return size;
	}

	unsigned int SegmentedRLEObject::getBBSize() const {
		return (mDownRight.x - mUpperLeft.x) * (mDownRight.y - mUpperLeft.y);
	}

	Vec2i SegmentedRLEObject::getCentroid() const {
		return Vec2i((mUpperLeft.x + mDownRight.x)/2, (mUpperLeft.y + mDownRight.y)/2);
	}

	bool sortFunction(LineRLE a, LineRLE b) {
		return a.i < b.i || (a.i == b.i && a.je < b.js) ? true : false;
	}

	void SegmentedRLEObject::sortObj() {
		std::sort(obj.begin(), obj.end(), sortFunction);
	}
} 

