/*
 * poly_line_iterator.h
 *
 *  Created on: Mar 2, 2015
 *      Author: tobias
 */

#ifndef POLY_LINE_ITERATOR_H_
#define POLY_LINE_ITERATOR_H_

#include "ring_iterator.h"    // heyho::ring_iterator
#include <opencv2/opencv.hpp> // cv::Point
#include <type_traits>        // std::is_same
#include "line_iterator_cv.h" // heyho::line_iterator

namespace heyho {

	/**
	 * iterates over the points of the lines connecting the iterator's points
	 *
	 * @tparam POINTS_IT required operations:
	 *                     POINTS_IT& POINTS_IT::operator++()
	 *                     bool POINTS_IT::end()
	 *                     typename POINTS_IT::value_type
	 *                     POINTS_IT::value_type POINTS_IT::operator*();
	 */
	template<typename POINTS_IT>
	class poly_line_iterator {
	public:
		using value_type = cv::Point;

		static_assert(std::is_same<typename POINTS_IT::value_type, value_type>::value, "invalid point type");

		explicit poly_line_iterator(POINTS_IT points_it, int connectivity);

		bool end() const;

		value_type operator*() const;

		poly_line_iterator& operator++();

	private:
		static heyho::line_iterator_cv make_line_it(POINTS_IT &it, int connectivity);

		POINTS_IT m_points;
		heyho::line_iterator_cv m_current_line;
	};

	namespace tests {
		void poly_line_iterator_tests();
	}

}

#include "poly_line_iterator_impl.h"

#endif /* POLY_LINE_ITERATOR_H_ */
