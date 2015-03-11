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
#include <type_traits> // std::integral_constant, std::{true,false}_type

namespace heyho {

	/**
	 * iterates over the elements first_element ... last_element
	 *
	 * i.e.: [first_element, first_element + 1, ..., range_last - 1, range_first, ..., last_element]
	 *
	 * @tparam IT forward iterator type or reverse iterator type
	 */
	template<typename IT, bool reverse = false>
	class ring_iterator {
	public:
		using value_type = typename std::iterator_traits<IT>::value_type;

		using forward_tag  = std::false_type;
		using backward_tag = std::true_type;

		using current_direction_tag  = std::integral_constant<bool,   reverse>;
		using opposite_direction_tag = std::integral_constant<bool, ! reverse>;

		explicit ring_iterator(IT range_first, IT range_last, IT first_element, IT last_element);

		const value_type& operator*() const;

		bool last() const;

		bool end() const;

		ring_iterator& operator++();

	protected:
		const IT m_range_first;
		const IT m_range_last;
		const IT m_first_element;
		const IT m_last_element;
		IT m_current;

		bool check_end_set_invalid();

		void advance(forward_tag);
		void advance(backward_tag);
		void advance();
		void advance_opposite_direction();
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
	class ring_iterator_bd : private ring_iterator<BDIT, false> {
	public:
		using value_type = typename ring_iterator<BDIT>::value_type;

		explicit ring_iterator_bd(BDIT range_first, BDIT range_last, BDIT first_element, BDIT last_element, bool reverse_direction);

		const value_type& operator*() const;

		using ring_iterator<BDIT>::last;
		using ring_iterator<BDIT>::end;

		ring_iterator_bd& operator++();

	private:
		const bool m_reverse_direction;
	};

	namespace tests {
		void ring_iterator_tests();
		void ring_iterator_bd_tests();
	}

}

#include "ring_iterator_impl.h"

#endif /* RING_ITERATOR_H_ */
