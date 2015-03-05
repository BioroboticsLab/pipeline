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
		using value_type = typename poly_lines_iterator<POINTS_IT>::value_type;

		explicit poly_line_vertical_iterator(POINTS_IT points_it, int connectivity, bool leftmost);

		value_type operator*() const;

		poly_line_vertical_iterator& operator++();

		bool end() const;

	private:
		void set_to_left_right_most_in_line();

		poly_lines_iterator<POINTS_IT> m_poly_line_points;
		value_type m_current_point;
		bool m_end;
		bool m_leftmost;

	};

	namespace tests {

		void poly_line_vertical_iterator_tests();

	}
}

#include "poly_line_vertical_iterator_impl.h"

#endif /* POLY_LINE_VERTICAL_ITERATOR_H_ */
