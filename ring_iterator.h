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
	 * i.e.: [first_element, first_element + 1, ..., range_last - 1, range_first, ..., last_element]
	 *
	 * @tparam FWDIT forward iterator type
	 */
	template<typename FWDIT>
	class ring_iterator {
	public:
		using value_type = typename std::iterator_traits<FWDIT>::value_type;

		explicit ring_iterator(FWDIT range_first, FWDIT range_last, FWDIT first_element, FWDIT last_element);

		const value_type& operator*() const;

		bool last() const;

		bool end() const;

		ring_iterator& operator++();

	protected:
		const FWDIT m_range_first;
		const FWDIT m_range_last;
		const FWDIT m_first_element;
		const FWDIT m_last_element;
		FWDIT m_current;

		bool check_end_set_invalid();
		void move_forward();
	};


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
	class ring_iterator_bd : private ring_iterator<BDIT> {
	public:
		using value_type = typename ring_iterator<BDIT>::value_type;

		explicit ring_iterator_bd(BDIT range_first, BDIT range_last, BDIT first_element, BDIT last_element, bool reverse_direction);

		const value_type& operator*() const;

		using ring_iterator<BDIT>::last;
		using ring_iterator<BDIT>::end;

		ring_iterator_bd& operator++();

	private:
		void move_backward();
		const bool m_reverse_direction;
	};

	namespace tests {
		void ring_iterator_tests();
		void ring_iterator_bd_tests();
	}

}

#include "ring_iterator_impl.h"

#endif /* RING_ITERATOR_H_ */
