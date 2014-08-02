/*
 * BoundingBox.h
 *
 *  Created on: 31.07.2014
 *      Author: mareikeziese
 */

#ifndef BOUNDINGBOX_H_
#define BOUNDINGBOX_H_


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

namespace decoder {

class BoundingBox {
public:
	BoundingBox();
	virtual ~BoundingBox();
	Rect box_;
	Mat sub_image_;
	Mat sub_image_orig_;
	bool isPossibleCenter(Point p, int tolerance);
};

} /* namespace decoder */

#endif /* BOUNDINGBOX_H_ */
