/*
 * blargs.h
 *
 *  Created on: Feb 20, 2015
 *      Author: tobias
 */

#ifndef BLARGS_H_
#define BLARGS_H_

#include <opencv2/opencv.hpp>

namespace heyho {

// static opencv functions used by fillConvexPoly
void Line  (cv::Mat& img, cv::Point pt1, cv::Point pt2, const void* _color, int connectivity = 8 );
void Line2 (cv::Mat& img, cv::Point pt1, cv::Point pt2, const void* color);
void LineAA(cv::Mat& img, cv::Point pt1, cv::Point pt2, const void* color);

void fillConvexPoly(cv::InputOutputArray _img, cv::InputArray _points,         const cv::Scalar& color, int line_type = 8, int shift = 0);
void fillConvexPoly(cv::Mat& img,              const cv::Point* pts, int npts, const cv::Scalar& color, int line_type = 8, int shift = 0);
void FillConvexPoly(cv::Mat& img,              const cv::Point* v,   int npts, const void* color,       int line_type,     int shift);

}

#endif /* BLARGS_H_ */
