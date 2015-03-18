/*
 * poly_line_iterator.impl.h
 *
 *  Created on: Mar 4, 2015
 *      Author: tobias
 */

#ifndef POLY_LINE_ITERATOR_IMPL_H_
#define POLY_LINE_ITERATOR_IMPL_H_

namespace heyho {

	template<typename POINTS_IT, typename LINE_IT>
	inline poly_line_iterator<POINTS_IT, LINE_IT>::poly_line_iterator(POINTS_IT points_it, int connectivity)
		: m_points(std::move(points_it))
		, m_connectivity(connectivity)
		, m_current_line(make_line_it(m_points, m_connectivity))
	{}

	template<typename POINTS_IT, typename LINE_IT>
	inline bool poly_line_iterator<POINTS_IT, LINE_IT>::end() const {
		return m_points.end() && m_current_line.end();
	}

	template<typename POINTS_IT, typename LINE_IT>
	inline typename poly_line_iterator<POINTS_IT, LINE_IT>::value_type poly_line_iterator<POINTS_IT, LINE_IT>::operator*() const {
		if (this->end()) {
			throw std::runtime_error("dereferencing invalid iterator");
		}
		return *m_current_line;
	}

	template<typename POINTS_IT, typename LINE_IT>
	inline poly_line_iterator<POINTS_IT, LINE_IT>& poly_line_iterator<POINTS_IT, LINE_IT>::operator++()
	{
		if (!this->end())
		{
			// advance to next pixel
			++m_current_line;

			// end of current line is reached --> advance to next line
			// (this is a loop instead of an if, in order to skip single-pixel lines)
			while (m_current_line.end() && !m_points.end())
			{
				const auto prev_line_point = *m_points;
				++m_points;
				if (!m_points.end()) {
					m_current_line = LINE_IT(no_boundaries_tag{}, prev_line_point, *m_points, m_connectivity);
					// this isn't the first line --> skipt it's first pixel == previous line's last pixel
					++m_current_line;
				}
			}
		}
		return *this;
	}

	template<typename POINTS_IT, typename LINE_IT>
	inline LINE_IT poly_line_iterator<POINTS_IT, LINE_IT>::make_line_it(POINTS_IT &it, int connectivity) {
		const auto p1 = *it;
		++it;
		const auto p2 = it.end() ? p1 : *it;
		return LINE_IT(no_boundaries_tag{}, p1, p2, connectivity);
	}

}

#endif /* POLY_LINE_ITERATOR_IMPL_H_ */
