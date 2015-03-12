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
	 * Calls f with every Point of the convex polygon spanned by points exactly one time.
	 *
	 * The polygon is cropped by the rectangle cv::Rect(cv::Point(0,0), size).
	 *
	 * @param size          cropping dimensions
	 * @param begin         iterator to the polygon's first point
	 * @param end           iterator behind the polygon's last point
	 * @param f             some functor or function
	 * @param connectivity  line connectivity : 4 or 8
	 *
	 * @tparam LINE_IT class that iterates over the pixels connecting to points
	 *                 (e.g. heyho::line_iterator_cv, heyho::line_iterator)
	 *
	 ************************************************
	 *
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
	template<typename IT, typename F, typename LINE_IT>
	F convex_poly(cv::Rect boundaries, IT begin, IT end,      F f, int connectivity = 8);
	template<typename IT, typename F, typename LINE_IT>
	F convex_poly(cv::Size size,       IT begin, IT end,      F f, int connectivity = 8);

	template<typename F, typename LINE_IT>
	F convex_poly(cv::Rect boundaries, cv::InputArray points, F f, int connectivity = 8);
	template<typename F, typename LINE_IT>
	F convex_poly(cv::Size size,       cv::InputArray points, F f, int connectivity = 8);

	/**
	 * @see convex_poly
	 *
	 * Sets every pixel inside the polygon to the specified color.
	 *
	 */
	template<typename pixel_t, typename LINE_IT>
	void fill_convex_poly(cv::InputOutputArray img, cv::InputArray points, const cv::Scalar& color, int line_type = 8);
	template<typename pixel_t, typename LINE_IT>
	void fill_convex_poly(cv::Mat &img,             cv::InputArray points, const cv::Scalar& color, int line_type = 8);
	template<typename pixel_t, typename LINE_IT>
	void fill_convex_poly(cv::Mat &img,             cv::InputArray points, const pixel_t &color,    int line_type = 8);

}

#include "fill_convex_poly.impl.h"

#endif /* FILL_CONVEX_POLY_H_ */
