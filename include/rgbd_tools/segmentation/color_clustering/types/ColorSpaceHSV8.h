/////////////////////////////////////////////////////////////////////////////////////////
//	BOVIL: core
//
//		Author: Pablo Ramon Soria
//		Date:	2014-05-03
//
/////////////////////////////////////////////////////////////////////////////////////////


#ifndef _BOVIL_COLORSPACEHSV8_H_
#define _BOVIL_COLORSPACEHSV8_H_

#include <rgbd_tools/segmentation/color_clustering/types/ColorClusterSpace.h>

#include <math.h>
#include <string>

namespace BOViL {
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
} // namespace BOViL.

#endif // _BOVIL_COLORSPACEHSV8_H_
