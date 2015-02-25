/*
 * blargs.h
 *
 *  Created on: Feb 20, 2015
 *      Author: tobias
 */

#ifndef BLARGS_H_
#define BLARGS_H_

#include <opencv2/opencv.hpp>
#include <utility>

namespace heyho {

template<typename pixel_t>
struct pixel_setter
{
	cv::Mat &img;
	const pixel_t color;

	void operator()(cv::Point p) {
		img.at<pixel_t>(p) = color;
	}

	void operator()(int y, int x) {
		img.at<pixel_t>(y, x) = color;
	}
};

template<typename F>
F Line(cv::Mat& img, cv::Point pt1, cv::Point pt2, F f, int connectivity = 8);
template<typename pixel_t>
void DrawLine(cv::Mat& img, cv::Point pt1, cv::Point pt2, const pixel_t &color, int connectivity = 8);

template<typename F>
inline F hline(cv::Mat&, int x1, int x2, int y, F f);
template<typename pixel_t>
inline void drawhline(cv::Mat &img, int x1, int x2, int y, const pixel_t &color);


template<typename pixel_t>
void fillConvexPoly(cv::InputOutputArray _img, cv::InputArray _points,         const cv::Scalar& color, int line_type = 8);
template<typename pixel_t>
void fillConvexPoly(cv::Mat& img,              const cv::Point* pts, int npts, const cv::Scalar& color, int line_type = 8);
template<typename pixel_t>
void FillConvexPoly(cv::Mat& img,              const cv::Point* v,   int npts, const void* color,       int line_type = 8);

template<typename F>
F ConvexPoly(cv::Mat& img, const cv::Point* v, int npts, F f, int line_type = 8);

}

#include "blargs_impl.h"

#endif /* BLARGS_H_ */
