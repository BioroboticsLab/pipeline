/*
 * poly_line_vertical_iterator_impl.h
 *
 *  Created on: Mar 5, 2015
 *      Author: tobias
 */

#ifndef POLY_LINE_VERTICAL_ITERATOR_IMPL_H_
#define POLY_LINE_VERTICAL_ITERATOR_IMPL_H_

namespace heyho {

	template<typename POINTS_IT>
	poly_line_vertical_iterator<POINTS_IT>::hline::hline(cv::Point p)
		: x_left(p.x)
		, x_right(p.x)
		, y(p.y)
	{}

	template<typename POINTS_IT>
	poly_line_vertical_iterator<POINTS_IT>::hline::hline(const hline&) = default;

	template<typename POINTS_IT>
	cv::Point poly_line_vertical_iterator<POINTS_IT>::hline::left() const {
		return {x_left, y};
	}

	template<typename POINTS_IT>
	cv::Point poly_line_vertical_iterator<POINTS_IT>::hline::right() const {
		return {x_right, y};
	}

	template<typename POINTS_IT>
	poly_line_vertical_iterator<POINTS_IT>::poly_line_vertical_iterator(POINTS_IT points_it, int connectivity)
		: m_poly_line_points(std::move(points_it), connectivity)
		, m_current_point(*m_poly_line_points)
		, m_end(false)
	{
		++m_poly_line_points;
		this->set_to_left_right_most_in_line();
	}

	template<typename POINTS_IT>
	const typename poly_line_vertical_iterator<POINTS_IT>::hline& poly_line_vertical_iterator<POINTS_IT>::operator*() const {
		if (this->end()) {
			throw std::runtime_error("dereferencing invalid iterator");
		}
		return m_current_point;
	}

	template<typename POINTS_IT>
	const typename poly_line_vertical_iterator<POINTS_IT>::hline* poly_line_vertical_iterator<POINTS_IT>::operator->() const {
		return &**this;
	}

	template<typename POINTS_IT>
	poly_line_vertical_iterator<POINTS_IT>& poly_line_vertical_iterator<POINTS_IT>::operator++() {
		if (!this->end()) {
			if (m_poly_line_points.end()) {
				m_end = true;
			}
			else {
				m_current_point = hline(*m_poly_line_points);
				this->set_to_left_right_most_in_line();
			}
		}
		return *this;
	}

	template<typename POINTS_IT>
	bool poly_line_vertical_iterator<POINTS_IT>::end() const {
		return m_end;
	}

	template<typename POINTS_IT>
	void poly_line_vertical_iterator<POINTS_IT>::set_to_left_right_most_in_line()
	{
		while (!m_poly_line_points.end())
		{
			const auto next_point = *m_poly_line_points;

			// next point is in same line --> check x-coordinate
			if (m_current_point.y == next_point.y)
			{
				if (next_point.x < m_current_point.x_left) {
					m_current_point.x_left = next_point.x;
				}
				if (next_point.x > m_current_point.x_right) {
					m_current_point.x_right = next_point.x;
				}
				++m_poly_line_points;
			}

			// the next point is in a new line
			else if (m_current_point.y + 1 == next_point.y) {
				break;
			}

			// the next point is in an invalid line
			else {
				throw int(45);
			}
		}
	}

}

#endif /* POLY_LINE_VERTICAL_ITERATOR_IMPL_H_ */
