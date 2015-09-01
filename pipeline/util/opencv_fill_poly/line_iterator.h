/*
 * line_iterator.h
 *
 *  Created on: Mar 6, 2015
 *      Author: tobias
 */

#ifndef LINE_ITERATOR_H_
#define LINE_ITERATOR_H_

#include <opencv2/opencv.hpp> // cv::Point
#include "helper.h"           // heyho::abs, heyho::connectivity
#include <stdexcept>          // std::invalid_argument, std::logic_error

namespace heyho {

/**
 * TODO TODO
 *
 * size check
 *
 * templ line tests ---> test both
 *
 *
 */

	class line_iterator {
	public:
		/**
		 * Iterator for the line connecting p1 and p2.
		 * The line will be clipped on the image boundaries ( i.e. (0,0),(boundaries.width,boundaries.height) ).
		 * The line is 8-connected or 4-connected.
		 */
		explicit line_iterator(cv::Size          boundaries, cv::Point p1, cv::Point p2, connectivity line_type);

		/**
		 * Iterator for the line connecting p1 and p2.
		 * The line will be clipped on the image boundaries.
		 * The line is 8-connected or 4-connected.
		 */
		explicit line_iterator(cv::Rect          boundaries, cv::Point p1, cv::Point p2, connectivity line_type);

		/**
		 * Iterator for the line connecting p1 and p2.
		 * The line will not be clipped.
		 * The line is 8-connected or 4-connected.
		 */
		explicit line_iterator(no_boundaries_tag boundaries, cv::Point p1, cv::Point p2, connectivity line_type);

		/**
		 * empty line iterator.
		 */
		line_iterator();

		cv::Point operator*() const;

		line_iterator& operator++();

		cv::Point pos() const;

		bool end() const {
			return m_remaining_points == 0;
		}

	private:
		void init(cv::Point pt1, cv::Point pt2, connectivity line_type);
		cv::Point m_current_point;
		size_t    m_remaining_points;
		int       m_error;
		cv::Point m_fast_step;
		cv::Point m_slow_step;
		int       m_fast_step_err_inc;
		int       m_slow_step_err_inc;
	};

	namespace tests {

		void line_iterator_tests();

	}


}

#include "line_iterator.impl.h"

#endif /* LINE_ITERATOR_H_ */
