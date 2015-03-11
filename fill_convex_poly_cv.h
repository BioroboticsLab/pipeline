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

	template<typename F, typename LINE_IT>
	F convex_poly_cv(cv::Size size, cv::InputArray points, F f, int line_type = 8);


	template<typename pixel_t, typename LINE_IT>
	void fill_convex_poly_cv(cv::InputOutputArray img, cv::InputArray points, const cv::Scalar& color, int line_type = 8);
	template<typename pixel_t, typename LINE_IT>
	void fill_convex_poly_cv(cv::Mat& img,             cv::InputArray points, const cv::Scalar& color, int line_type = 8);
	template<typename pixel_t, typename LINE_IT>
	void fill_convex_poly_cv(cv::Mat& img,             cv::InputArray points, const pixel_t &color,    int line_type = 8);

}

#include "fill_convex_poly_cv.impl.h"

#endif /* FILL_CONVEX_POLY_CV_H_ */
