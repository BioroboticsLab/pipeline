/*
 * ring_iterator_impl.h
 *
 *  Created on: Mar 2, 2015
 *      Author: tobias
 */

#ifndef RING_ITERATOR_IMPL_H_
#define RING_ITERATOR_IMPL_H_

namespace heyho {

template<typename FWDIT>
inline ring_iterator<FWDIT>::ring_iterator(FWDIT range_first, FWDIT range_last, FWDIT first_element, FWDIT last_element)
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

template<typename FWDIT>
inline const typename ring_iterator<FWDIT>::value_type& ring_iterator<FWDIT>::operator*() const {
	if (this->end()) {
		throw std::runtime_error("dereferencing invalid iterator");
	}
	return *m_current;
}

template<typename FWDIT>
inline bool ring_iterator<FWDIT>::last() const {
	return m_current == m_last_element;
}

template<typename FWDIT>
inline bool ring_iterator<FWDIT>::end() const {
	return m_current == m_range_last;
}

template<typename FWDIT>
inline ring_iterator<FWDIT>& ring_iterator<FWDIT>::operator++()
{
	if (! this->check_end_set_invalid() ) {
		this->move_forward();
	}
	return *this;
}

template<typename FWDIT>
bool ring_iterator<FWDIT>::check_end_set_invalid() {
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

template<typename FWDIT>
void ring_iterator<FWDIT>::move_forward() {
	++m_current;
	if (m_current == m_range_last) {
		m_current = m_range_first;
	}
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
	inline bool ring_iterator_bd<BDIT>::last() const {
		return ring_iterator<BDIT>::last();
	}

	template<typename BDIT>
	inline bool ring_iterator_bd<BDIT>::end() const {
		return ring_iterator<BDIT>::end();
	}

	template<typename BDIT>
	inline ring_iterator_bd<BDIT>& ring_iterator_bd<BDIT>::operator++()
	{
		if (! this->check_end_set_invalid() ) {
			if (m_reverse_direction) {
				this->move_backward();
			}
			else {
				this->move_forward();
			}
		}
		return *this;
	}

	template<typename FWDIT>
	void ring_iterator_bd<FWDIT>::move_backward() {
		if (this->m_current == this->m_range_first) {
			this->m_current = this->m_range_last;
		}
		--this->m_current;
	}

}

#endif /* RING_ITERATOR_IMPL_H_ */
