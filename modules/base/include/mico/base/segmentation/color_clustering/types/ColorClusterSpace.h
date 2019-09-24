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

#include <mico/base/segmentation/color_clustering/types/Colors.h>

#include <math.h>
#include <stdint.h>
#include <string>

#ifndef MICO_BASE_SEGMENTATION_COLORCLUSTERING_TYPES_COLORCLUSTERSPACE_H_
#define MICO_BASE_SEGMENTATION_COLORCLUSTERING_TYPES_COLORCLUSTERSPACE_H_

#define LOG2 0.3010299957

namespace mico {
	/** Class that store a set of clusters in a color space. 777 ad example with image and so on...
	*/
	class ColorClusterSpace {
	public:
		int size;	//	 666 add a method to get size and set private.
		c3u *clusters; //	 666 add a method to get clusters and set private.

		/** \brief
		*/
		ColorClusterSpace(int, unsigned char* _AClass, unsigned char* _BClass, unsigned char* _CClass, const c3u* _colors);
		
		/** \brief
		*/
		~ColorClusterSpace();

	private:
		unsigned char *AClass;
		unsigned char *BClass;
		unsigned char *CClass;

	public:
		/** \brief compute color mermbership.	777 add example.
		*	@param _a: first channel value.
		*	@param _b: second channel value.
		*	@param _c: third channel value.
		*/
		int operator()(unsigned char* _a, unsigned char*_b, unsigned char*_c){
			c3u col(*_a, *_b, *_c);

			int color =  whichColor(col);	
			if(color != -1){
				*_a = clusters[color].a;
				*_b = clusters[color].b;
				*_c = clusters[color].c;
			} else{
				*_a = 0U;
				*_b = 0U;
				*_c = 0U;
			}

			return color;
		};

		int whichColor(c3u& _color){
			int i = (_color.a * (size - 1) / 180); // 666 TODO: improve (gets 5%)
			int j = _color.b*(size - 1) >> 7;
			j = (j>>1) + (j&1);
			int k = _color.c*(size - 1) >> 7;
			k = (k>>1) + (k&1);
			

			int res = AClass[i] & BClass[j] & CClass[k]; //Supposing that colors are not over-layed there's only one possible solution and log2(x) returns an integer /

			int aux = 0;

			if (!res)
				return -1;

			while (!(res & 0x01)) {
				res = res >> 1;
				aux += 1;
			}

			return aux;
		}
	};
} // namespace mico .

#endif // RGBDTOOLS_SEGMENTATION_COLORCLUSTERING_TYPES_COLORCLUSTERSPACE_H_
