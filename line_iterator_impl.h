/*
 * line_iterator_impl.h
 *
 *  Created on: Mar 2, 2015
 *      Author: tobias
 */

#ifndef LINE_ITERATOR_IMPL_H_
#define LINE_ITERATOR_IMPL_H_

namespace heyho {

	inline line_iterator::line_iterator(cv::Point p1, cv::Point p2, int connectivity, cv::Size size, cv::Point offset)
		: m_connectivity(connectivity)
		/*
		 * The cv::lineIterator requires a cv::Mat to get the image
		 * dimensions (i.e. valid coordinates) and does all calculations
		 * on the matrix' data pointer, step size etc.
		 * Thus a dummy matrix is used.
		 */
		, m_it(cv::Mat(size, CV_8UC1, nullptr), p1, p2, m_connectivity, false)
		, m_remaining_points(static_cast<size_t>(m_it.count))
		, m_offset(offset)
	{
		if (m_connectivity != 4 && m_connectivity != 8) {
			throw std::invalid_argument("invalid connectivity");
		}
	}


	inline line_iterator::line_iterator(cv::Point p1, cv::Point p2, int connectivity, cv::Size size)
		/*
		 * forward everything to private "main" constructor
		 */
		: line_iterator(p1, p2, connectivity, size, {0, 0})
	{}


	inline line_iterator::line_iterator(cv::Point p1, cv::Point p2, int connectivity, cv::Rect boundaries)
		/*
		 * shift points by the bounding box' upper left corner, set offset
		 */
		: line_iterator(p1 - boundaries.tl(), p2 - boundaries.tl(), connectivity, boundaries.size(), boundaries.tl())
	{}

	inline line_iterator::line_iterator(cv::Point p1, cv::Point p2, int connectivity)
		/*
		 * use bounding box that contains both points --> no clippnig
		 */
		: line_iterator(p1, p2, connectivity, cv::Rect(p1, p2) + cv::Size(1,1))
	{}

	inline line_iterator::line_iterator()
		: line_iterator({0,0}, {0,0}, 8)
	{
		++*this;
	}

	inline line_iterator& line_iterator::operator++() {
		if (! this->end()) {
			--m_remaining_points;
			++m_it;
		}
		return *this;
	}

	inline bool line_iterator::end() const {
		return m_remaining_points == 0;
	}

	inline bool line_iterator::last() const {
		return m_remaining_points == 1;
	}

	inline cv::Point line_iterator::operator*() const {
		if (this->end()) {
			throw std::runtime_error("dereferencing invalid iterator");
		}
		return m_offset + m_it.pos();
	}

	inline int line_iterator::connectivity() const {
		return m_connectivity;
	}

	inline std::size_t line_iterator::size() const {
		return static_cast<size_t>(m_it.count);
	}

	inline std::size_t line_iterator::remaining_points() const {
		return m_remaining_points;
	}

}

#endif /* LINE_ITERATOR_IMPL_H_ */
