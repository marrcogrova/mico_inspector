/////////////////////////////////////////////////////////////////////////////////////////
//	BOVIL: segmentation
//
//		Author: Pablo Ramón Soria
//		Date:	2014-02-13
//
/////////////////////////////////////////////////////////////////////////////////////////



#ifndef _BOVIL_ALGORITHMS_COLORCLUSTERING_H_
#define _BOVIL_ALGORITHMS_COLORCLUSTERING_H_

#include <vector>
#include <functional>

#include "rgbd_tools/segmentation/color_clustering/types/BasicTypes.h"
#include "rgbd_tools/segmentation/color_clustering/types/ColorSpaceHSV8.h"
#include "rgbd_tools/segmentation/color_clustering/types/SegmentedRLEObject.h"


namespace BOViL{
	namespace algorithms{
		//-------------------------------------------------------------------------------------------------------------
		// The template is the type of image pointer, and function is the segmentate pixel format

		/** generic interaface of pixel transformation functions.
		*/
		template<typename T> color3<T>  pixelXXX2XXX(color3<T> _col){
			return _col;
		}
	

		//-------------------------------------------------------------------------------------------------------------
		// Private functions definition
		/** 777 Move to .inl
		*/
		template<typename T_>
		void changeColor(T_ *_image, unsigned _i, unsigned _j, unsigned _width, std::function<color3<T_>(const color3<T_> _col)> _functionColorSpace);

		/** 777 Move to .inl
		*/
		template<typename T_>
		std::vector<LineRLE> simplifyLine(T_ *_image, unsigned _i, unsigned _width, std::function<color3<T_>(const color3<T_> _col)> _functionColorSpace, std::function<int(T_ *_a, T_ *_b, T_ *_c)> _functionSegmentation);

		/** 777 Move to .inl
		*/
		template<typename T_>
		void joinLines(unsigned _i, std::vector<std::vector<LineRLE>> &_rleList);

		//-------------------------------------------------------------------------------------------------------------
		// Algorithm
		/** Algorithm to segmentate image using CMU segmentation algorithm (777 add reference). 777 Add example of use. 
		*	777 move definition to .inl.
		*/
		template<typename T> void ColorClustering(	T *_image, 
													int _width,
													int _height, 
													unsigned int _sizeThreshold, 
													std::vector<ImageObject> &_objects,
													std::function<int (T *_a, T *_b, T *_c)> _functionSegmentation,
													std::function<color3<T> (const color3<T> _col)> _functionColorSpace = nullptr){

			std::vector<std::vector<LineRLE>> aRLE;		// Matrix with every RLE encoded objects
			std::vector<SegmentedRLEObject> objects;	// Auxiliary object that store Segmented objects while they are been growing.

			for (int i = 0; i < _height; i++){	// Main vertical loop on the image
				// First horizontal loop. It will segmentate every pixel and encode it in a RLE if it's a desired color.
				std::vector<LineRLE> simplifiedLine = simplifyLine<T>(_image, i, _width, _functionColorSpace, _functionSegmentation);
				aRLE.push_back(simplifiedLine);

				// Now starts the harder step, it takes the lineRLE objects of the current and previous row and connect them in order to collect every piece of the complete object
				joinLines<T>(i, aRLE);

			}

			// Re-assing overlaped parents
			for(unsigned int i = 0 ; i < aRLE.size() ; i++){
				for(unsigned int j = 0 ; j < aRLE[i].size() ; j++){
					if(aRLE[i][j].hasParent){
						LineRLE auxRLE = aRLE[aRLE[i][j].pi][aRLE[i][j].pj];

						int loopCounter = 0;
						while(auxRLE.hasParent && loopCounter < 30){ // 666 TODO: optimice
							aRLE[i][j].pi = auxRLE.pi;
							aRLE[i][j].pj = auxRLE.pj;

							auxRLE = aRLE[auxRLE.pi][auxRLE.pj];

							loopCounter++;
						}
						if (aRLE[aRLE[i][j].pi][aRLE[i][j].pj].iObj == -1) {
							aRLE[aRLE[i][j].pi][aRLE[i][j].pj].iObj = objects.size();
							SegmentedRLEObject obj(aRLE[aRLE[i][j].pi][aRLE[i][j].pj]);
							objects.push_back(obj);
						}
					}
				}	// for i
			}	// for j

			for(unsigned int i = 0 ; i < aRLE.size() ; i++){
				for(unsigned int j = 0 ; j < aRLE[i].size() ; j++){
					if(aRLE[i][j].hasParent){
						objects[aRLE[aRLE[i][j].pi][aRLE[i][j].pj].iObj].addLineObjRLE(aRLE[i][j]);
					}
				}	// for i
			}	// for j
			
			// Threshold filtering
			for(unsigned int i = 0; i < objects.size() ; i ++){
				if(objects[i].getSize() >= _sizeThreshold)
					_objects.push_back(ImageObject(objects[i].upperLeft(), objects[i].downRight(), objects[i].getSize(), objects[i].getColor()));
			}
		}	// ColorClustering
		
		//-------------------------------------------------------------------------------------------------------------
		// "Private" functions
		template<typename T_>
		void changeColor(T_ *_image, unsigned _i, unsigned _j, unsigned _width, std::function<color3<T_>(const color3<T_> _col)> _functionColorSpace){
			color3<T_> c3(*(_image + _i * _width * 3 + 3 * _j + 0),
				*(_image + _i * _width * 3 + 3 * _j + 1),
				*(_image + _i * _width * 3 + 3 * _j + 2));

			c3 = _functionColorSpace(c3);

			*(_image + _i * _width * 3 + 3 * _j + 0) = c3.a;
			*(_image + _i * _width * 3 + 3 * _j + 1) = c3.b;
			*(_image + _i * _width * 3 + 3 * _j + 2) = c3.c;
		}

		//-------------------------------------------------------------------------------------------------------------
		template<typename T_>
		std::vector<LineRLE> simplifyLine(T_ *_image, unsigned _i, unsigned _width, std::function<color3<T_>(const color3<T_> _col)> _functionColorSpace, std::function<int(T_ *_a, T_ *_b, T_ *_c)> _functionSegmentation){
			std::vector<LineRLE> simplifiedLine;
			int currentColor, colorRLE = -1;
			unsigned js = 0;
			for (unsigned j = 0; j < _width; j++){
				// Change color space if argument is received.
				if (_functionColorSpace != nullptr)
					changeColor<T_>(_image, _i, j, _width, _functionColorSpace);

				// This function segmentate the pixel and return the color of those.
				currentColor = _functionSegmentation(_image + _i * _width * 3 + 3 * j + 0, _image + _i * _width * 3 + 3 * j + 1, _image + _i * _width * 3 + 3 * j + 2);

				// If first pixel of row, get it's color as initial color
				if (j == 0) {
					colorRLE = currentColor;
					js = 0;
				}
				else 	if (j == _width - 1){ // If it's the last pixel of the row finallize the last RLE line of this row.
					simplifiedLine.push_back(LineRLE(_i, js, j, colorRLE));
				}
				else {	// Any other intermedial pixel
					if (currentColor != colorRLE){	// If the color of the current pixel is not the same that the current lineRLE's color then create the previous lineRLE and start a new.
						simplifiedLine.push_back(LineRLE(_i, js, j, colorRLE));
						colorRLE = currentColor;
						js = j;
					}
				}
			}
			// End of first horizontal loop for RLE encoding and segmentation.
			return simplifiedLine;
		}

		template<typename T_>
		void joinLines(unsigned _i, std::vector<std::vector<LineRLE>> &_rleList){
			if (_i){	//First line cannot have parents
				unsigned int pcRLE = 0, ppRLE = 0; 		// Index of the lineRLE objects on the i (current) and  i-1 (previous) rows of aRLE
				unsigned int jc = _rleList[_i][pcRLE].size, jp = _rleList[_i - 1][ppRLE].size;	// Index of current row's column and previous row's column.

				bool condition = true;
				while (condition){	// Connecting RLEs
					if (!(_rleList[_i - 1][ppRLE].color == -1 || _rleList[_i][pcRLE].color == -1)) {
						if (_rleList[_i - 1][ppRLE].color == _rleList[_i][pcRLE].color &&	// If lineRLEs have same color and have some column in common...
							_rleList[_i - 1][ppRLE].je >= _rleList[_i][pcRLE].js &&
							_rleList[_i - 1][ppRLE].js <= _rleList[_i][pcRLE].je){
							if (!_rleList[_i][pcRLE].hasParent){			// If current lineRLE has no parent
								if (_rleList[_i - 1][ppRLE].hasParent){		// Use previous parent
									_rleList[_i][pcRLE].pi = _rleList[_i - 1][ppRLE].pi;
									_rleList[_i][pcRLE].pj = _rleList[_i - 1][ppRLE].pj;
									_rleList[_i][pcRLE].hasParent = true;

								}
								else {			// Previous lineRLE is the parent
									_rleList[_i][pcRLE].pi = _i - 1;
									_rleList[_i][pcRLE].pj = ppRLE;
									_rleList[_i][pcRLE].hasParent = true;

								}
							}
							else{	// If current lineRLE has parent --> OVERLAP
								if (_rleList[_i - 1][ppRLE].hasParent){		// There is a "family" of RLE lines
									if ((_rleList[_i - 1][ppRLE].pi != _rleList[_i][pcRLE].pi) || (_rleList[_i - 1][ppRLE].pj != _rleList[_i][pcRLE].pj)){
										_rleList[_rleList[_i - 1][ppRLE].pi][_rleList[_i - 1][ppRLE].pj].pi = _rleList[_i][pcRLE].pi;
										_rleList[_rleList[_i - 1][ppRLE].pi][_rleList[_i - 1][ppRLE].pj].pj = _rleList[_i][pcRLE].pj;
										_rleList[_rleList[_i - 1][ppRLE].pi][_rleList[_i - 1][ppRLE].pj].hasParent = true;
									}


								}
								else {			// There is an orphan RLE line
									_rleList[_i - 1][ppRLE].pi = _rleList[_i][pcRLE].pi;
									_rleList[_i - 1][ppRLE].pj = _rleList[_i][pcRLE].pj;
									_rleList[_i - 1][ppRLE].hasParent = true;

								}
							}
						}
					}	// Connecting RLEs
					if (pcRLE >= _rleList[_i].size() - 1 && ppRLE >= _rleList[_i - 1].size() - 1){
						condition = false;
						continue;
					}
					if (jp >= jc){
						pcRLE++;
						jc += _rleList[_i][pcRLE].size;
					}
					else if (jp <= jc){
						ppRLE++;
						jp += _rleList[_i - 1][ppRLE].size;
					}
				}	// while(1).
			}	// If not first row.

		}	// Vertical loop for each row
		

	}	// namespace algorithms
}	// namespace BOViL

#endif	// _BOVIL_ALGORITHMS_COLORCLUSTERING_H_
