/*
 * fill_convex_poly.h
 *
 *  Created on: Mar 5, 2015
 *      Author: tobias
 */

#ifndef FILL_CONVEX_POLY_H_
#define FILL_CONVEX_POLY_H_

#include "ring_iterator.h"
#include "poly_line_vertical_iterator.h"
#include "helper.h"                      // heyho::pixel_setter, heyho::cv_point_less_y
#include "lines.h"                       // heyho::hline
#include <vector>                        // std::vector
#include <algorithm>                     // std::minmax_element
#include <opencv2/opencv.hpp>            // cv::Mat, cv::Point, ...

namespace heyho {

	/**
	 * The polygon is rendered linewise from left to right starting at the smallest y-coordinate.
	 *
	 * general idea:
	 * -------------
	 * 1. find points with smallest (top) and largest (bottom) y-coordiante
	 * 2. follow the poly-lines connecting top and bottom in clockwise & counterclockwise direction and draw horizontal lines
	 *
	 * how it's done:
	 * --------------
	 * - ring_iterator                is used to follow the polygon's points from top to bottom
	 *                                in both clockwise and counterclockwise directions
	 * - line_iterator                iterates over the pixels connecting to points
	 * - poly_line_iterator           uses both of them to iterate over all pixels connecting
	 *                                multiple points
	 * - poly_line_vertical_iterator  iterates over these pixels and ensures, that the y-coordinates
	 *                                are increasing i.e. skips pixels and only returns the leftmost / rightmost
	 *                                pixel in each horizontal line
	 *
	 */
	template<typename IT, typename F>
	F convex_poly(cv::Size size, IT begin, IT end, F f, int connectivity);


	template<typename pixel_t>
	void fill_convex_poly(cv::InputOutputArray _img, cv::InputArray _points, const cv::Scalar& color, int line_type = 8);

}

#include "fill_convex_poly_impl.h"

#endif /* FILL_CONVEX_POLY_H_ */
