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

	/**
	 * Calls f with every Point of the convex polygon spanned by points a least once.
	 *
	 * !!! f is called multiple times with the same point (e.g. outline points) !!!
	 *
	 * The polygon is cropped by the rectangle cv::Rect(cv::Point(0,0), size).
	 *
	 * @about This function's code is a modified version of open cv's cv::fillConvecPoly:
	 *  * all macros have been replaced by (templatized) functions
	 *  * cv::Mat and cv::Mat::type() specific code has been replaced by calls to a functor and "drawing" boundaries (cv::Size)
	 *  * removed shift param
	 *  * removed anti-aliased lines
	 *  * added consts
	 *
	 * @param size       cropping dimensions
	 * @param points     points that span the polygon
	 * @param f          some functor or function
	 * @param line_type  line connectivity : 4 or 8
	 *
	 * @tparam LINE_IT class that iterates over the pixels connecting to points
	 *                 (e.g. heyho::line_iterator_cv, heyho::line_iterator)
	 *
	 */
	template<typename LINE_IT, typename F>
	F convex_poly_cv(F f, cv::Size size, cv::InputArray points, int line_type = 8);

	/**
	 * @see convex_poly_cv
	 *
	 * Sets every pixel inside the polygon to the specified color.
	 *
	 */
	template<typename LINE_IT, typename pixel_t>
	void fill_convex_poly_cv(cv::InputOutputArray img, const cv::Scalar &color, cv::InputArray points, int line_type = 8);
	template<typename LINE_IT, typename pixel_t>
	void fill_convex_poly_cv(cv::Mat& img,             const cv::Scalar &color, cv::InputArray points, int line_type = 8);
	template<typename LINE_IT, typename pixel_t>
	void fill_convex_poly_cv(cv::Mat& img,             const pixel_t &color,    cv::InputArray points, int line_type = 8);

}

#include "fill_convex_poly_cv.impl.h"

#endif /* FILL_CONVEX_POLY_CV_H_ */
