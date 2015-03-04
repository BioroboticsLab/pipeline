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

namespace heyho {

	/**
	 * converts an opencv image type to the name of the corresponding macro i.e. "CV_\d[UFS]C\d"
	 */
	std::string img_type_to_str(int type);

	/**
	 * compares cv::Points based on their y coordinate
	 */
	struct cv_point_less_y {
		bool operator()(const cv::Point &lhs, const cv::Point &rhs) const {
			return lhs.y < rhs.y;
		}
	};


	/**
	 * converts cv::Scalar into pixel.
	 */
	template<typename pixel_t>
	inline pixel_t scalar2pixel(const cv::Scalar &color);


	/**
	 * Stores a reference to a cv::Mat and a pixel.
	 *
	 * operator()(cv::Points) sets the matrix' pixel to the stored value.
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

}

#include "helper_impl.h"

#endif /* HELPER_H_ */
