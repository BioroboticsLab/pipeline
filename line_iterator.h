/*
 * line_iterator.h
 *
 *  Created on: Mar 2, 2015
 *      Author: tobias
 */

#ifndef LINE_ITERATOR_H_
#define LINE_ITERATOR_H_

#include <opencv2/opencv.hpp> // cv::Point, cv::LineIterator, cv::Size, cv::Rect
#include <stdexcept>          // std::runtime_error

namespace heyho {

	class line_iterator {

	private:
		explicit line_iterator(cv::Point p1, cv::Point p2, int connectivity, cv::Size size, cv::Point offset);

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
		 * Shifts iterator to the next pixel.
		 */
		line_iterator& operator++();

		/**
		 * True <=> the iterator points behind the line's last pixel.
		 */
		bool end() const;

		/**
		 * True <=> the current pixel is the line's endpoint
		 */
		bool last() const;

		/**
		 * returns coordinates of the current pixel
		 *
		 * @throws std::runtime_error <=> this->end()
		 */
		cv::Point operator*() const;

	private:
		cv::LineIterator m_it;
		size_t m_remaining_points;
		const cv::Point m_offset;
	};

	namespace tests {
		void line_iterator_tests();
	}

}

#include "line_iterator_impl.h"

#endif /* LINE_ITERATOR_H_ */
