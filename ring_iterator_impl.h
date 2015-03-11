/*
 * ring_iterator_impl.h
 *
 *  Created on: Mar 2, 2015
 *      Author: tobias
 */

#ifndef RING_ITERATOR_IMPL_H_
#define RING_ITERATOR_IMPL_H_

namespace heyho {

	template<typename IT, bool reverse>
	inline ring_iterator<IT, reverse>::ring_iterator(IT range_first, IT range_last, IT first_element, IT last_element)
		: m_range_first(range_first)
		, m_range_last(range_last)
		, m_first_element(first_element)
		, m_last_element(last_element)
		, m_current(m_first_element)
	{
		if (m_range_first == m_range_last) {
			throw std::invalid_argument("empty range");
		}
	}

	template<typename IT, bool reverse>
	inline const typename ring_iterator<IT, reverse>::value_type& ring_iterator<IT, reverse>::operator*() const {
		if (this->end()) {
			throw std::runtime_error("dereferencing invalid iterator");
		}
		return *m_current;
	}

	template<typename IT, bool reverse>
	inline bool ring_iterator<IT, reverse>::last() const {
		return m_current == m_last_element;
	}

	template<typename IT, bool reverse>
	inline bool ring_iterator<IT, reverse>::end() const {
		return m_current == m_range_last;
	}

	template<typename IT, bool reverse>
	inline ring_iterator<IT, reverse>& ring_iterator<IT, reverse>::operator++()
	{
		if (! this->check_end_set_invalid() ) {
			this->advance();
		}
		return *this;
	}

	template<typename IT, bool reverse>
	inline bool ring_iterator<IT, reverse>::check_end_set_invalid() {
		if (this->end()) {
			return true;
		}
		// last element --> invalidate this
		if (this->last()) {
			m_current = m_range_last;
			return true;
		}
		return false;
	}

	template<typename IT, bool reverse>
	inline void ring_iterator<IT, reverse>::advance(forward_tag) {
		++m_current;
		if (m_current == m_range_last) {
			m_current = m_range_first;
		}
	}

	template<typename IT, bool reverse>
	inline void ring_iterator<IT, reverse>::advance(backward_tag) {
		if (m_current == m_range_first) {
			m_current = m_range_last;
		}
		--m_current;
	}

	template<typename IT, bool reverse>
	inline void ring_iterator<IT, reverse>::advance() {
		this->advance(current_direction_tag{});
	}

	template<typename IT, bool reverse>
	inline void ring_iterator<IT, reverse>::advance_opposite_direction() {
		this->advance(opposite_direction_tag{});
	}

	//////////////////////////////

	template<typename BDIT>
	inline ring_iterator_bd<BDIT>::ring_iterator_bd(BDIT range_first, BDIT range_last, BDIT first_element, BDIT last_element, bool reverse_direction)
		: ring_iterator<BDIT>(range_first, range_last, first_element, last_element)
		, m_reverse_direction(reverse_direction)
	{
	}

	template<typename BDIT>
	inline const typename ring_iterator_bd<BDIT>::value_type& ring_iterator_bd<BDIT>::operator*() const {
		return ring_iterator<BDIT>::operator*();
	}

	template<typename BDIT>
	inline ring_iterator_bd<BDIT>& ring_iterator_bd<BDIT>::operator++()
	{
		if (! this->check_end_set_invalid() ) {
			if (m_reverse_direction) {
				this->advance_opposite_direction();
			}
			else {
				this->advance();
			}
		}
		return *this;
	}


}

#endif /* RING_ITERATOR_IMPL_H_ */
