/*
 * blargs.h
 *
 *  Created on: Feb 20, 2015
 *      Author: tobias
 */

#ifndef BLARGS_H_
#define BLARGS_H_

#include "line_iterator.h"    // heyho::line_iterator
#include "helper.h"
#include <opencv2/opencv.hpp> // cv::Point, cv::LineIterator, cv::Mat, cv::InputOutputArray, cv::OutputArray, cv::Scalar
#include <utility>            // std::move
#include <stdexcept>          // std::invalid_argument
#include <algorithm>          // std::min, std::max, std::swap

namespace heyho {


template<typename F>
inline F hline(cv::Size size, int x1, int x2, int y, F f);

template<typename F>
F line(cv::Size size, cv::Point pt1, cv::Point pt2, F f, int connectivity = 8);

template<typename F>
F convex_poly(cv::Size size, const cv::Point* v, int npts, F f, int line_type = 8);



template<typename pixel_t>
inline void draw_hline(cv::Mat &img, int x1, int x2, int y, const pixel_t &color) {
	heyho::hline(img.size(), x1, x2, y, pixel_setter<pixel_t>{img, color});
}

template<typename pixel_t>
void draw_line(cv::Mat& img, cv::Point pt1, cv::Point pt2, const pixel_t &color, int connectivity = 8) {
	heyho::line(img.size(), pt1, pt2, pixel_setter<pixel_t>{img, color}, connectivity);
}


template<typename pixel_t>
void fill_convex_poly(cv::InputOutputArray _img, cv::InputArray _points,         const cv::Scalar& color, int line_type = 8);
template<typename pixel_t>
void fill_convex_poly(cv::Mat& img,              const cv::Point* pts, int npts, const cv::Scalar& color, int line_type = 8);
template<typename pixel_t>
void fill_convex_poly(cv::Mat& img,              const cv::Point* pts, int npts, const pixel_t &color,    int line_type = 8);

}

#include "blargs_impl.h"

#endif /* BLARGS_H_ */
