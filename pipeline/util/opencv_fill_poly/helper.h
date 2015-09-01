/*
 * helper.h
 *
 *  Created on: Feb 26, 2015
 *      Author: tobias
 */

#ifndef HELPER_H_
#define HELPER_H_

#include <opencv2/opencv.hpp> // cv::Mat, cv::Scalar, cv::Point, cv::saturate_cast, CV_MAT_DEPTH, CV_8U, ...
#include <string>             // std::string, std::to_string
#include <functional>         // std::reverence_wrapper
#include <stdexcept>          // std::invalid_argument
#include <cmath>              // std::abs
#include <algorithm>          // std::copy_n
#include <type_traits>        // std::true_type

namespace heyho {

	/**
	 * converts an opencv image type to the name of the corresponding macro i.e. "CV_\d[UFS]C\d"
	 */
	std::string img_type_to_str(int type);

	/**
	 * compares cv::Point_s based on their y coordinate.
	 */
	struct cv_point_less_y
	{
		template<typename T>
		bool operator()(const cv::Point_<T> &lhs, const cv::Point_<T> &rhs) const {
			return lhs.y < rhs.y;
		}
	};

	/**
	 * wraps a bool "value" that is true, iff T is one of either
	 *   - unsigned char
	 *   - signed char
	 *   - unsigned short
	 *   - short
	 *   - int
	 *   - float
	 *   - double
	 *
	 *   although "bool" is a primitive open cv data type
	 *   (see: http://docs.opencv.org/modules/core/doc/basic_structures.html)
	 *   it has been removed from this traits object, since
	 *   cv::DataType<bool>::type == cv::DataType<uin8_t>::type
	 */
	template<typename T>
	struct is_primitive_open_cv_data_type;

	/**
	 * converts cv::Scalar into pixel.
	 */
	template<typename pixel_t>
	inline pixel_t scalar2pixel(const cv::Scalar &color);

	/**
	 * used as param for functions that accept cv::Size, cv::Rect as boundaries
	 */
	struct no_boundaries_tag {};


	enum class connectivity : int {four_connected = 4, eight_connected = 8};

	/**
	 * Stores a reference to a cv::Mat and a pixel value.
	 *
	 * operator()(cv::Point) sets the matrix' corresponding pixel to the stored value.
	 *
	 */
	template<typename pixel_t>
	class pixel_setter
	{
	public:
		explicit pixel_setter(cv::Mat &img, const pixel_t &color);
		explicit pixel_setter(cv::Mat &img, const cv::Scalar &color);
		void operator()(cv::Point p);
	private:
		std::reference_wrapper<cv::Mat> m_img;
		pixel_t m_color;
	};

	/**
	 * Stores a reference to a cv::Mat and the value of a zero-pixel in order to count zero and non-zero pixels.
	 *
	 * operator()(cv::Point) compares the matrix' corresponding pixel with the stored value and increment either the zero or non-zero counter.
	 *
	 */
	template<typename pixel_t>
	class pixel_counter
	{
	public:
		struct pixel_count {

			size_t m_zero;
			size_t m_non_zero;

			size_t zero() const;
			size_t non_zero() const;
			size_t all() const;
		};

		explicit pixel_counter(const cv::Mat &img, const pixel_t &zero);
		explicit pixel_counter(const cv::Mat &img, const cv::Scalar &zero);
		void operator()(cv::Point p);
		const pixel_count& count() const;
	private:
		std::reference_wrapper<const cv::Mat> m_img;
		pixel_t                               m_zero;
		pixel_count                           m_count;
	};

	template<typename T>
	cv::Point_<T> abs(const cv::Point_<T> &p) {
		return {std::abs(p.x), std::abs(p.y)};
	}

	/**
	 * extracts cv::Point* and number of Points from a cv::InputArray
	 *
	 * @throws std::invalid_argument iff pts doesn't contain cv::Points or it's data isn't continuous
	 * @return (ptr to first point, number of points)
	 */
	std::pair<const cv::Point*, int> cv_point_input_array_to_pointer(cv::InputArray pts);

	namespace tests {

		void helper_tests();

	}

}

#include "helper.impl.h"

#endif /* HELPER_H_ */
