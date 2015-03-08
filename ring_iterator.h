/*
 * ring_iterator.h
 *
 *  Created on: Mar 2, 2015
 *      Author: tobias
 */

#ifndef RING_ITERATOR_H_
#define RING_ITERATOR_H_

#include <iterator>    // std::distance
#include <algorithm>   // std::minmax_element
#include <stdexcept>   // std::invalid_argument, std::runtime_error

namespace heyho {

	/**
	 * iterates over the elements first_element ... last_element
	 *
	 *
	 * reverse_direction == false --> [first_element, first_element + 1, ..., range_last - 1, range_first   , ..., last_element]
	 * reverse_direction == true  --> [first_element, first_element - 1, ..., range_first   , range_last - 1, ..., last_element]
	 *
	 * @tparam BDIT bidirectional iterator type
	 */
	template<typename BDIT>
	class ring_iterator_bd {
	public:
		using value_type = typename std::iterator_traits<BDIT>::value_type;

		explicit ring_iterator_bd(BDIT range_first, BDIT range_last, BDIT first_element, BDIT last_element, bool reverse_direction);

		const value_type& operator*() const;

		bool last() const;

		bool end() const;

		ring_iterator_bd& operator++();

	private:

		void move_left();

		void move_right();

		const BDIT m_range_first;
		const BDIT m_range_last;
		const BDIT m_first_element;
		const BDIT m_last_element;
		const bool m_reverse_direction;
		BDIT m_current;
	};

	namespace tests {
		void ring_iterator_tests();
	}

}

#include "ring_iterator_impl.h"

#endif /* RING_ITERATOR_H_ */
