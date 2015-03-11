/*
 * line_iterator_cv_impl.h
 *
 *  Created on: Mar 2, 2015
 *      Author: tobias
 */

#ifndef LINE_ITERATOR_CV_IMPL_H_
#define LINE_ITERATOR_CV_IMPL_H_

namespace heyho {

	inline line_iterator_cv::line_iterator_cv(cv::Point p1, cv::Point p2, int connectivity, cv::Size size, cv::Point offset)
		/*
		 * The cv::lineIterator requires a cv::Mat to get the image
		 * dimensions (i.e. valid coordinates) and does all calculations
		 * on the matrix' data pointer, step size etc.
		 * Thus a dummy matrix is used.
		 */
		: m_it(cv::Mat(size, CV_8UC1, nullptr), p1, p2, connectivity, false)
		, m_remaining_points(static_cast<size_t>(m_it.count))
		, m_offset(offset)
	{
		if (connectivity != 4 && connectivity != 8) {
			throw std::invalid_argument("invalid connectivity");
		}
	}


	inline line_iterator_cv::line_iterator_cv(cv::Point p1, cv::Point p2, int connectivity, cv::Size size)
		/*
		 * forward everything to private "main" constructor
		 */
		: line_iterator_cv(p1, p2, connectivity, size, {0, 0})
	{}


	inline line_iterator_cv::line_iterator_cv(cv::Point p1, cv::Point p2, int connectivity, cv::Rect boundaries)
		/*
		 * shift points by the bounding box' upper left corner, set offset
		 */
		: line_iterator_cv(p1 - boundaries.tl(), p2 - boundaries.tl(), connectivity, boundaries.size(), boundaries.tl())
	{}

	inline line_iterator_cv::line_iterator_cv(cv::Point p1, cv::Point p2, int connectivity)
		/*
		 * use bounding box that contains both points --> no clippnig
		 */
		: line_iterator_cv(p1, p2, connectivity, cv::Rect(p1, p2) + cv::Size(1,1))
	{}

	inline line_iterator_cv::line_iterator_cv()
		: line_iterator_cv({0,0}, {0,0}, 8)
	{
		++*this;
	}

	inline line_iterator_cv& line_iterator_cv::operator++() {
		if (! this->end()) {
			--m_remaining_points;
			++m_it;
		}
		return *this;
	}

	inline bool line_iterator_cv::end() const {
		return m_remaining_points == 0;
	}

	inline cv::Point line_iterator_cv::operator*() const {
		if (this->end()) {
			throw std::runtime_error("dereferencing invalid iterator");
		}
		return m_offset + m_it.pos();
	}

	inline std::size_t line_iterator_cv::size() const {
		return static_cast<size_t>(m_it.count);
	}

	inline std::size_t line_iterator_cv::remaining_points() const {
		return m_remaining_points;
	}

}

#endif /* LINE_ITERATOR_CV_IMPL_H_ */
