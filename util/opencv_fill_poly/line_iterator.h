/*
 * line_iterator.h
 *
 *  Created on: Mar 6, 2015
 *      Author: tobias
 */

#ifndef LINE_ITERATOR_H_
#define LINE_ITERATOR_H_

#include <opencv2/opencv.hpp> // cv::Point
#include "helper.h"           // heyho::abs
#include <stdexcept>          // std::invalid_argument, std::runtime_error

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
		 * The line will be clipped on the image boundaries ( i.e. (0,0),(size.width,size.height) ).
		 * The line is 8-connected or 4-connected.
		 */
		explicit line_iterator(cv::Point p1, cv::Point p2, int connectivity, cv::Size size);

		/**
		 * Iterator for the line connecting p1 and p2.
		 * The line will be clipped on the image boundaries.
		 * The line is 8-connected or 4-connected.
		 */
		explicit line_iterator(cv::Point p1, cv::Point p2, int connectivity, cv::Rect boundaries);

		/**
		 * Iterator for the line connecting p1 and p2.
		 * The line will not be clipped.
		 * The line is 8-connected or 4-connected.
		 */
		explicit line_iterator(cv::Point p1, cv::Point p2, int connectivity);

		/**
		 * empty line iterator.
		 */
		explicit line_iterator();

		cv::Point operator*() const;

		line_iterator& operator ++();

		cv::Point pos() const;

		bool end() const {
			return m_remaining_points == 0;
		}

	private:
		void init(cv::Point pt1, cv::Point pt2, int connectivity);
		cv::Point m_current_point;
		size_t    m_remaining_points;
		int       m_error;
		cv::Point m_fast_step;
		cv::Point m_slow_step;
		int       m_fast_step_err_inc;
		int       m_slow_step_err_inc;
	};

	inline line_iterator::line_iterator()
		: m_remaining_points(0)
		, m_error()
		, m_fast_step_err_inc()
		, m_slow_step_err_inc()
	{}

	inline line_iterator::line_iterator(cv::Point p1, cv::Point p2, int connectivity)
		: line_iterator()
	{
		this->init(p1, p2, connectivity);
	}

	inline line_iterator::line_iterator(cv::Point p1, cv::Point p2, int connectivity, cv::Size size)
		: line_iterator()
	{
		if (cv::clipLine(size, p1, p2)) {
			this->init(p1, p2, connectivity);
		}
	}

	inline line_iterator::line_iterator(cv::Point p1, cv::Point p2, int connectivity, cv::Rect boundaries)
		: line_iterator()
	{
		if (cv::clipLine(boundaries, p1, p2)) {
			this->init(p1, p2, connectivity);
		}
	}

	inline void line_iterator::init(cv::Point p1, cv::Point p2, int connectivity)
	{
		if (connectivity != 8 && connectivity != 4) {
			throw std::invalid_argument("invalid connectivity");
		}
		m_current_point    = p1;
		m_remaining_points = 1;

		if (p2 == p1) {
			return;
		}

		const cv::Point delta     = p2 - p1;
		const cv::Point abs_delta = heyho::abs(delta);

		const cv::Point step_x(delta.x < 0 ? -1 : 1, 0                   );
		const cv::Point step_y(0                   , delta.y < 0 ? -1 : 1);

		/*
		 * The general bresenham algorithm requires 0 <= slope <= 1.
		 * ( -1 <= slope <= 0 is handled by switching the increment(s)'s sign(s) @see: step_{x,y} )
		 *
		 * This can be generalized by simply switching x and y if 1 < slope < infinity.
		 *
		 * In order to avoid confusion, movements along the x- and y-axis are referred to
		 * as "fast" and "slow" depending on the the slope:
		 *    - For a small  slope line x is in/decremented in every step and y only if a certain error is reached.
		 *    - For a large  slope line y is changed in every step.
		 * This directions is referred to as "fast".
		 *
		 */
		const bool small_slope = abs_delta.x >= abs_delta.y;

		const int fast_step_abs_delta = small_slope ? abs_delta.x : abs_delta.y;
		const int slow_step_abs_delta = small_slope ? abs_delta.y : abs_delta.x;

		const cv::Point &fast_step    = small_slope ? step_x : step_y;
		const cv::Point &slow_step    = small_slope ? step_y : step_x;

		if (connectivity == 8)
		{
			m_error = fast_step_abs_delta - 2 * slow_step_abs_delta;

			m_remaining_points += static_cast<size_t>(fast_step_abs_delta);

			m_fast_step = fast_step;
			m_slow_step = slow_step;

			m_fast_step_err_inc = -2 * slow_step_abs_delta;
			m_slow_step_err_inc =  2 * fast_step_abs_delta;
		}
		else /* connectivity == 4 */
		{
			m_error = 0;

			m_remaining_points += static_cast<size_t>(fast_step_abs_delta + slow_step_abs_delta);

			m_fast_step = fast_step;
			m_slow_step = slow_step - fast_step;

			m_fast_step_err_inc = -2 *  slow_step_abs_delta;
			m_slow_step_err_inc =  2 * (fast_step_abs_delta + slow_step_abs_delta);
		}
	}

	inline line_iterator& line_iterator::operator ++()
	{
		if (!this->end())
		{
			--m_remaining_points;

			if (m_error < 0) {
				m_error         += m_fast_step_err_inc + m_slow_step_err_inc;
				m_current_point += m_fast_step         + m_slow_step;
			}
			else {
				m_error         += m_fast_step_err_inc;
				m_current_point += m_fast_step;
			}
		}
		return *this;
	}

	inline cv::Point line_iterator::pos() const
	{
		if (this->end() ) {
			throw std::runtime_error("dereferencing an invalid pointer");
		}
		return m_current_point;
	}

	inline cv::Point line_iterator::operator*() const {
		return this->pos();
	}

	namespace tests {

		void line_iterator_tests();

	}


}



#endif /* LINE_ITERATOR_H_ */
