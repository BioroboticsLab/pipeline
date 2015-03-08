/*
 * fill_convex_poly_cv.h
 *
 *  Created on: Feb 20, 2015
 *      Author: tobias
 */

#ifndef FILL_CONVEX_POLY_CV_H_
#define FILL_CONVEX_POLY_CV_H_

#include "lines.h"            // heyho::hline, heyho::line
#include "helper.h"           // heyho::scalar2pixel
#include <opencv2/opencv.hpp> // cv::Point, cv::Mat, cv::InputOutputArray, cv::OutputArray, cv::Scalar
#include <utility>            // std::move
#include <stdexcept>          // std::invalid_argument
#include <algorithm>          // std::min, std::max, std::swap

namespace heyho {

	template<typename F, typename LINE_IT = line_iterator_cv>
	F convex_poly_cv(cv::Size size, const cv::Point* v, int npts, F f, int line_type = 8);


	template<typename pixel_t, typename LINE_IT = line_iterator_cv>
	void fill_convex_poly_cv(cv::InputOutputArray _img, cv::InputArray _points,         const cv::Scalar& color, int line_type = 8);
	template<typename pixel_t, typename LINE_IT = line_iterator_cv>
	void fill_convex_poly_cv(cv::Mat& img,              const cv::Point* pts, int npts, const cv::Scalar& color, int line_type = 8);
	template<typename pixel_t, typename LINE_IT = line_iterator_cv>
	void fill_convex_poly_cv(cv::Mat& img,              const cv::Point* pts, int npts, const pixel_t &color,    int line_type = 8);

}

#include "fill_convex_poly_cv_impl.h"

#endif /* FILL_CONVEX_POLY_CV_H_ */
