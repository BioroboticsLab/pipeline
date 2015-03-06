/*
 * poly_line_vertical_iterator.h
 *
 *  Created on: Mar 4, 2015
 *      Author: tobias
 */

#ifndef POLY_LINE_VERTICAL_ITERATOR_H_
#define POLY_LINE_VERTICAL_ITERATOR_H_

#include "poly_line_iterator.h"
#include <stdexcept>

namespace heyho {

	template<typename POINTS_IT>
	class poly_line_vertical_iterator
	{
	public:
		class hline {
			friend class poly_line_vertical_iterator;
		public:
			int x_left;
			int x_right;
			int y;
			cv::Point left() const;
			cv::Point right() const;
		private:
			hline(cv::Point p);
			hline(const hline&);

		};

		explicit poly_line_vertical_iterator(POINTS_IT points_it, int connectivity);

		const hline& operator*() const;
		const hline* operator->() const;

		poly_line_vertical_iterator& operator++();

		bool end() const;

	private:
		void set_to_left_right_most_in_line();

		poly_lines_iterator<POINTS_IT> m_poly_line_points;
		hline m_current_point;
		bool m_end;
	};

	namespace tests {

		void poly_line_vertical_iterator_tests();

	}
}

#include "poly_line_vertical_iterator_impl.h"

#endif /* POLY_LINE_VERTICAL_ITERATOR_H_ */
