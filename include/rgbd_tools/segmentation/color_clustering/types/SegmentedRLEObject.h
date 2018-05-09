/////////////////////////////////////////////////////////////////////////////////////////
//	BOVIL: segmentation
//
//		Author: Pablo Ramón Soria
//		Date:	2014-02-19
//
/////////////////////////////////////////////////////////////////////////////////////////


#ifndef _BOVIL_ALGORITHM_SEGMENTATION_H_
#define _BOVIL_ALGORITHM_SEGMENTATION_H_

#include <vector>
#include <rgbd_tools/segmentation/color_clustering/types/BasicTypes.h>

namespace BOViL{
	namespace algorithms{
		//-----------------------------------------------------------------------------
		/** Struct that holds compressed line of image using RLE based on color codes.
		*/
		struct LineRLE{		// RLE encoding object.
			LineRLE(int _i,
					int _js,
					int _je,
					int _color){
						i = _i;
						js = _js;
						je = _je;
						size = je - js;
						color = _color;
						hasParent = false;
						pi = pj = 0;
						iObj = -1;
			};

			int i;			// Row of the object.
			int js;		// Started column of the object.
			int je;		// Ended column of the object.
			int size;		// Size of the object (= je - js + 1) its computed one time in order to reduce the algorithm operations.

			int color;	// Color of the object.s

			bool hasParent;				// Flag if the RLE was parented.
			int pi;			// Row index of the parent in the vector.
			int pj;			// Column index of the parent in the vector.

			int iObj;

		};

		/** Image object compressed using RLE encoding.
		*/
		class SegmentedRLEObject {
		public:
			/** \brief Create a new instance using a line as first element.
			*/
			SegmentedRLEObject(LineRLE _ini);
			
			/** \brief add new compressed line to object.
			*/
			void addLineObjRLE(LineRLE _line);
			
			/** \brief add a set of compressed lines encoded in a RLE object.
			*/
			void addRLEFamily(SegmentedRLEObject&);

			/** \brief	get number of compressed lines. 77 rename to lines().
			*/
			int getLines() const;
			
			/** \brief	get index of object. 777 need review.
			*/
			LineRLE getRLEObj(int) const;
			
			/** \brief get upper left corner of object.
			*/
			Vec2i upperLeft() const;
			
			/** \brief get down right corner of object.
			*/
			Vec2i downRight() const;
			
			/** \brief get simplified color of object. 777 need review. 777 rename to color().
			*/
			unsigned int getColor() const;
			
			/** \brief get amount of pixels in the object. 777 rename to size().
			*/
			unsigned int getSize() const;
			
			/** \brief get size of bouncing box of object. 77 rename to bbSize().
			*/
			unsigned int getBBSize() const;
			
			/** \brief get centroid ob object. 777 rename to centroid().
			*/
			Vec2i getCentroid() const;

			/** \brief sort lines based on line's position in image.
			*/
			void sortObj();

		private:
			std::vector<LineRLE> obj;

			Vec2i mUpperLeft, mDownRight; // Border pixels
			int color;
			unsigned int size;
		};

	}	// namespace algorithms
}	// namespace BOViL

#endif	//_BOVIL_ALGORITHM_SEGMENTATION_H_
