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

#ifndef MICO_BASE_SEGMENTATION_COLORCLUSTERING_TYPES_COLORS_H_
#define MICO_BASE_SEGMENTATION_COLORCLUSTERING_TYPES_COLORS_H_

namespace mico {
	// ----- Traits -----		666 TODO: hacer con constexpr cuando salga para el compilador de vs

	/** 777 need review.
	*/
	template <typename T> struct ColorTypeInfo{

	};

	/** 777 need review.
	*/
	template <> struct ColorTypeInfo<unsigned char>{
		static unsigned char getMAX(){ return 255; };
		static unsigned char getMIN(){ return 0; };
	};

	/** 777 need review.
	*/
	template <> struct ColorTypeInfo<float>{
		static float getMAX(){ return 1.0f; };
		static float getMIN(){ return 0.0f; };
	};

	// ----- Types and structures -----
	/** 777 need review.
	*/
	template<typename T_> struct color3 {
			color3(){
				a = b = c = T_(0);
			};

			color3(T_ _a, T_ _b, T_ _c){
				a = _a; b = _b; c = _c;
			};
			
			T_ a, b, c;
		};

	/** 777 need review.
	*/
	typedef color3<unsigned char> c3u;
	/** 777 need review.
	*/
	typedef color3<float> c3f;
	/** 777 need review.
	*/
	typedef color3<double> c3d;


	// ----- Functions -----
	/** 777 need review.
	*/
	template<typename T> color3<T> PixelRGB2HSV(const color3<T> &_color){
		color3<T> HSV;
		T MAX, MIN;

		_color.a > _color.b ? (_color.a > _color.c ? MAX = _color.a : MAX = _color.c) : (_color.b > _color.c ? MAX = _color.b : MAX = _color.c);
		_color.a < _color.b ? (_color.a < _color.c ? MIN = _color.a : MIN = _color.c) : (_color.b < _color.c ? MIN = _color.b : MIN = _color.c);
	
		if(MAX == MIN)
			HSV.a = 0;
		else if (_color.a == MAX && _color.b >= _color.c)
			HSV.a = (60*(_color.b - _color.c)/(MAX - MIN) + 0)/360 * ColorTypeInfo<T>::getMAX();
		else if (_color.a == MAX && _color.b < _color.c)
			HSV.a = (60*(_color.b - _color.c)/(MAX - MIN) + 360)/360 * ColorTypeInfo<T>::getMAX();
		else if (_color.b == MAX)
			HSV.a = (60*(_color.c - _color.a)/(MAX - MIN) + 120)/360 * ColorTypeInfo<T>::getMAX();
		else if (_color.c == MAX)
			HSV.a = (60*(_color.a - _color.b)/(MAX - MIN) + 240)/360 * ColorTypeInfo<T>::getMAX();


		MAX == 0 ? HSV.b = 0 : HSV.b = (1 - MIN/MAX) * ColorTypeInfo<T>::getMAX();
		
		HSV.c = MAX;
		
		return HSV;
	}
}	// namespace rgbs


#endif	//RGBDTOOLS_SEGMENTATION_COLORCLUSTERING_TYPES_COLORS_H_