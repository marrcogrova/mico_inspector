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


#ifndef RGBDTOOLS_SEGMENTATION_COLORCLUSTERING_TYPES_COLORSPACEHSV8_H_
#define RGBDTOOLS_SEGMENTATION_COLORCLUSTERING_TYPES_COLORSPACEHSV8_H_

#include <rgbd_tools/segmentation/color_clustering/types/ColorClusterSpace.h>

#include <math.h>
#include <string>

namespace rgbd {
	/**	Functions that decode binary number to integer.
	*	@param	_bin: string that contains number in binary format
	*/
	int bin2dec(std::string _bin);

	/** Create a new color cluster space clustered in 8 colors equidisted with a resolution of 10 degrees.	777 add example.
	*	@param _maskH: mask applied tu hue channel. Ignore colors with 0 in it's binary position.
	*	@param _maskS: mask applied to saturation channel. Ignore colors with 0 in it's binary position.
	*	@param _maskV: mask applied to value channel. Ignore colors with 0 in it's binary position.
	*/
	ColorClusterSpace *CreateHSVCS_8c(unsigned char _maskH, unsigned char _maskS, unsigned char _maskV);
} // 

#endif // RGBDTOOLS_SEGMENTATION_COLORCLUSTERING_TYPES_COLORSPACEHSV8_H_
