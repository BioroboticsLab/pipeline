/*
 * lines.h
 *
 *  Created on: Mar 2, 2015
 *      Author: tobias
 */

#ifndef LINES_H_
#define LINES_H_

#include "helper.h"           // heyho::pixel_setter
#include <opencv2/opencv.hpp> // cv::Point, cv::Mat
#include <utility>            // std::move
#include <algorithm>          // std::min, std::max, std::swap
#include "line_iterator_cv.h"    // heyho::line_iterator

namespace heyho {

	template<typename F>
	F hline(F f, no_boundaries_tag boundaries, int x_left, int x_right, int y);
	template<typename F>
	F hline(F f, cv::Size          boundaries, int x_left, int x_right, int y);
	template<typename F>
	F hline(F f, cv::Rect          boundaries, int x_left, int x_right, int y);

	template<typename F, typename LINE_IT>
	F line(cv::Size size, cv::Point pt1, cv::Point pt2, F f, int connectivity = 8, bool left_to_right = false);


	template<typename pixel_t>
	inline void draw_hline(cv::Mat &img, const pixel_t &color, int x1, int x2, int y) {
		heyho::hline(pixel_setter<pixel_t>{img, color}, img.size(), x1, x2, y);
	}

	template<typename pixel_t, typename LINE_IT>
	inline void draw_line(cv::Mat& img, cv::Point pt1, cv::Point pt2, const pixel_t &color, int connectivity = 8, bool left_to_right = false) {
		heyho::line<pixel_t, LINE_IT>(img.size(), pt1, pt2, pixel_setter<pixel_t>{img, color}, connectivity, left_to_right);
	}

}

#include "lines_impl.h"

#endif /* LINES_H_ */
