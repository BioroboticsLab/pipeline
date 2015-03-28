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
	 * The polygon is cropped by the provided boundaries.
	 *
	 * @param f             some functor or function
	 * @param boundaries    cropping dimensions
	 * @param begin         iterator to the polygon's first point
	 * @param end           iterator behind the polygon's last point
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
	 * 1. find points with smallest (top) and largest (bottom) y-coordinate
	 * 2. follow the poly-lines connecting top and bottom in clockwise & counterclockwise direction and draw horizontal lines
	 *
	 * how it's done:
	 * --------------
	 * - ring_iterator                is used to follow the polygon's points from top to bottom
	 *                                in both clockwise and counterclockwise directions
	 * - LINE_IT                      iterates over the pixels connecting to points
	 * - poly_line_iterator           uses both of them to iterate over all pixels connecting
	 *                                multiple points
	 * - poly_line_vertical_iterator  iterates over these pixels and ensures, that the y-coordinates
	 *                                are increasing i.e. skips pixels and only returns the leftmost / rightmost
	 *                                pixel in each horizontal line
	 *
	 */
	template<typename LINE_IT, typename F, typename B, typename IT>
	F convex_poly(F f, B boundaries, IT begin, IT end, int connectivity = 8);

	template<typename LINE_IT, typename F, typename B>
	F convex_poly(F f, B boundaries, cv::InputArray points, int connectivity = 8);

	/**
	 * @see convex_poly
	 *
	 * Sets every pixel inside the polygon to the specified color.
	 *
	 */
	template<typename LINE_IT, typename pixel_t>
	void fill_convex_poly(cv::InputOutputArray img, const cv::Scalar& color, cv::InputArray points, int line_type = 8);
	template<typename LINE_IT, typename pixel_t>
	void fill_convex_poly(cv::Mat &img,             const cv::Scalar& color, cv::InputArray points, int line_type = 8);
	template<typename LINE_IT, typename pixel_t>
	void fill_convex_poly(cv::Mat &img,             const pixel_t &color,    cv::InputArray points, int line_type = 8);

}

#include "fill_convex_poly.impl.h"

#endif /* FILL_CONVEX_POLY_H_ */
