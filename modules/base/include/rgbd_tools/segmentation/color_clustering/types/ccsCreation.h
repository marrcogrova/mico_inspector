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

#ifndef RGBDTOOLS_SEGMENTATION_COLORCLUSTERING_TYPES_CCSCREATION_H_
#define RGBDTOOLS_SEGMENTATION_COLORCLUSTERING_TYPES_CCSCREATION_H_

#include <rgbd_tools/segmentation/color_clustering/types/ColorClusterSpace.h>

namespace rgbd {

	/**	\brief This method provide a Color clustered space that could be used to segmentate one single color.
	*
	* @param _c1Top: top threshold of first channel.
	* @param _c1Bottom: bottom threshold of first channel.
	* @param _c2Top: top threshold of second channel.
	* @param _c2Bottom: bottom threshold of second channel.
	* @param _c3Top: top threshold of third channel.
	* @param _c3Bottom: bottom threshold of third channel.
	* @param _c1Max: max value of first channel
	* @param _c2Max: max value of second channel
	* @param _c3Max: max value of third channel
	* @param _subdivision: Resolution of the space.
	*/

	ColorClusterSpace * createSingleClusteredSpace(	unsigned char _c1Bottom,
													unsigned char _c1Top,
													unsigned char _c2Bottom,
													unsigned char _c2Top,
													unsigned char _c3Bottom,
													unsigned char _c3Top,
													unsigned char _c1Max,
													unsigned char _c2Max,
													unsigned char _c3Max,
													unsigned char _subdivision ){

		// Channels thresholds.
		unsigned char *AClass = new unsigned char [_subdivision];
		unsigned char *BClass = new unsigned char [_subdivision];
		unsigned char *CClass = new unsigned char [_subdivision];

		unsigned char incC1 = _c1Max/_subdivision;
		unsigned char incC2 = _c2Max/_subdivision;
		unsigned char incC3 = _c3Max/_subdivision;

		for (unsigned i = 0; i < _subdivision; i++){
			AClass[i] = (i*incC1 >= _c1Bottom ) && (i*incC1 <= _c1Top) ? 1 : 0;
			BClass[i] = (i*incC2 >= _c2Bottom ) && (i*incC1 <= _c2Top) ? 1 : 0;
			CClass[i] = (i*incC3 >= _c3Bottom ) && (i*incC1 <= _c3Top) ? 1 : 0;
		}

		// Color dictionary.
		c3u colors[8];
		colors[0] = c3u((_c1Top + _c1Bottom) / 2,
						(_c2Top + _c2Bottom) / 2,
						(_c3Top + _c3Bottom) / 2);

		return new ColorClusterSpace(_subdivision, AClass, BClass, CClass, colors);
	}

	//--------------------------------------------------------------------------------------------------------------------
	ColorClusterSpace * createSingleSparseCluster(	const std::vector<std::pair<unsigned char, unsigned char>> & _c1Limits,
													const std::vector<std::pair<unsigned char, unsigned char>> & _c2Limits,
													const std::vector<std::pair<unsigned char, unsigned char>> & _c3Limits,
													const unsigned char _c1Max,
													const unsigned char _c2Max,
													const unsigned char _c3Max,
													const unsigned char _subdivision) {

		if (_c1Limits.size() == 0 || _c2Limits.size() == 0 || _c3Limits.size() == 0)
			return nullptr;
		
		// Channels thresholds.
		unsigned char *AClass = new unsigned char[_subdivision];
		unsigned char *BClass = new unsigned char[_subdivision];
		unsigned char *CClass = new unsigned char[_subdivision];

		unsigned char incC1 = _c1Max / _subdivision;
		unsigned char incC2 = _c2Max / _subdivision;
		unsigned char incC3 = _c3Max / _subdivision;

		for (unsigned i = 0; i < _subdivision; i++) {
			for (unsigned j = 0; j < _c1Limits.size(); j++) {
				if ((i*incC1 >= _c1Limits[j].first) && (i*incC1 <= _c1Limits[j].second)) {
					AClass[i] = 1;
					break;
				}
				else {
					AClass[i] = 0;
				}
			}
			for (unsigned j = 0; j < _c2Limits.size(); j++) {
				if ((i*incC2 >= _c2Limits[j].first) && (i*incC2 <= _c2Limits[j].second)) {
					BClass[i] = 1;
					break;
				}
				else {
					BClass[i] = 0;
				}
			}
			for (unsigned j = 0; j < _c3Limits.size(); j++) {
				if ((i*incC3 >= _c3Limits[j].first) && (i*incC3 <= _c3Limits[j].second)) {
					CClass[i] = 1;
					break;
				}
				else {
					CClass[i] = 0;
				}
			}
		}

		// Color dictionary.
		c3u colors[8];
		colors[0] = c3u((_c1Limits[0].first + _c1Limits[0].second) / 2,
						(_c2Limits[0].first + _c2Limits[0].second) / 2,
						(_c3Limits[0].first + _c3Limits[0].second) / 2);

		return new ColorClusterSpace(_subdivision, AClass, BClass, CClass, colors);
	}

}

#endif	//	RGBDTOOLS_SEGMENTATION_COLORCLUSTERING_TYPES_CCSCREATION_H_
