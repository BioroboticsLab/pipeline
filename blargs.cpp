/*
 * blargs.cpp
 *
 *  Created on: Feb 20, 2015
 *      Author: tobias
 */

#include "blargs.h"
#include <cstring>

namespace heyho {

void Line(cv::Mat& img, cv::Point pt1, cv::Point pt2, const void* _color, int connectivity)
{
	if( connectivity == 0 )
		connectivity = 8;
	if( connectivity == 1 )
		connectivity = 4;

	cv::LineIterator iterator(img, pt1, pt2, connectivity, true);
	const int count = iterator.count;
	const int pix_size = static_cast<int>(img.elemSize());
	const uchar* const color = static_cast<const uchar*>(_color);

	for(int i = 0; i < count; i++, ++iterator )
	{
		uchar* const ptr = *iterator;
		if( pix_size == 1 )
			ptr[0] = color[0];
		else if( pix_size == 3 )
		{
			ptr[0] = color[0];
			ptr[1] = color[1];
			ptr[2] = color[2];
		}
		else
			std::memcpy( *iterator, color, pix_size );
	}
}

}
