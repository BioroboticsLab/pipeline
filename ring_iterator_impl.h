/*
 * ring_iterator_impl.h
 *
 *  Created on: Mar 2, 2015
 *      Author: tobias
 */

#ifndef RING_ITERATOR_IMPL_H_
#define RING_ITERATOR_IMPL_H_

namespace heyho {

	template<typename BDIT>
	inline ring_iterator_bd<BDIT>::ring_iterator_bd(BDIT range_first, BDIT range_last, BDIT first_element, BDIT last_element, bool reverse_direction)
		: m_range_first(range_first)
		, m_range_last(range_last)
		, m_first_element(first_element)
		, m_last_element(last_element)
		, m_reverse_direction(reverse_direction)
		, m_current(m_first_element)
	{
		if (m_range_first == m_range_last) {
			throw std::invalid_argument("empty range");
		}
	}

	template<typename BDIT>
	inline const typename ring_iterator_bd<BDIT>::value_type& ring_iterator_bd<BDIT>::operator*() const {
		if (this->end()) {
			throw std::runtime_error("dereferencing invalid iterator");
		}
		return *m_current;
	}

	template<typename BDIT>
	inline bool ring_iterator_bd<BDIT>::last() const {
		return m_current == m_last_element;
	}

	template<typename BDIT>
	inline bool ring_iterator_bd<BDIT>::end() const {
		return m_current == m_range_last;
	}

	template<typename BDIT>
	inline ring_iterator_bd<BDIT>& ring_iterator_bd<BDIT>::operator++()
	{
		if (! this->end()) {
			// last element --> invalidate this
			if (this->last()) {
				m_current = m_range_last;
			}
			// move to next element
			else {
				if (m_reverse_direction) {
					this->move_left();
				}
				else {
					this->move_right();
				}
			}
		}
		return *this;
	}

	template<typename BDIT>
	inline void ring_iterator_bd<BDIT>::move_left() {
		if (m_current == m_range_first) {
			m_current = m_range_last;
		}
		--m_current;
	}

	template<typename BDIT>
	inline void  ring_iterator_bd<BDIT>::move_right() {
		++m_current;
		if (m_current == m_range_last) {
			m_current = m_range_first;
		}
	}

}

#endif /* RING_ITERATOR_IMPL_H_ */
