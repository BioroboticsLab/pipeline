/*
 * helper.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: tobias
 */

#include "helper.h"
#include <stdexcept> // std::runtime_error
#include <iostream>  // std::cout
#include <limits>    // std::numeric_limits

namespace heyho {

	std::string img_type_to_str(int type) {
		const int depth = CV_MAT_DEPTH(type);
		const int cn = CV_MAT_CN(type);
		std::string result = "CV_";

		switch (depth) {
			case CV_8U:
				result += "8U";
				break;
			case CV_8S:
				result += "8S";
				break;
			case CV_16U:
				result += "16U";
				break;
			case CV_16S:
				result += "16S";
				break;
			case CV_32S:
				result += "32S";
				break;
			case CV_32F:
				result += "32F";
				break;
			case CV_64F:
				result += "64F";
				break;
			default:
				result += "UNKOWN";
		}
		result += "C" + std::to_string(cn);
		return result;
	}

	namespace tests {

		/**
		 * converts a cv::Scalar to a pixel using the cv::Mat's constructor
		 * @param color
		 * @return
		 */
		template<typename T>
		T cv_scalar_2_pixel(const cv::Scalar &color) {
			cv::Mat m(1, 1, cv::DataType<T>::type, color);
			return m.template at<T>(0, 0);
		}

		template<typename T>
		void compare_scalar_to_pixel(const cv::Scalar &color) {
			const T pixel_cv    = cv_scalar_2_pixel<T>(color);
			const T pixel_heyho = scalar2pixel<T>(color);
			if (pixel_cv != pixel_heyho) {
				throw std::runtime_error("");
			}
		}

		template<typename chan_t, typename pixel_t = chan_t>
		void compare_scalar_to_pixel() {
			const double min = std::numeric_limits<chan_t>::min();
			const double max = std::numeric_limits<chan_t>::max();
			const double mid = (std::numeric_limits<chan_t>::min() + std::numeric_limits<chan_t>::max()) / 2;

			// in range
			compare_scalar_to_pixel<pixel_t>(cv::Scalar(min, min + 1.0, min + 2.0, min + 3.0));
			compare_scalar_to_pixel<pixel_t>(cv::Scalar(max, max + 1.0, max - 2.0, max - 3.0));
			compare_scalar_to_pixel<pixel_t>(cv::Scalar(mid, mid + 1.0, mid + 2.0, mid + 3.0));
			// out of range
			compare_scalar_to_pixel<pixel_t>(cv::Scalar(min - 16.0, min - 32.0, min - 64.0, min - 128.0));
			compare_scalar_to_pixel<pixel_t>(cv::Scalar(max + 16.0, max + 32.0, max + 64.0, max + 128.0));
		}

		template<typename chan_t>
		void compare_scalar_to_pixel_vec() {
			compare_scalar_to_pixel<chan_t, cv::Vec<chan_t, 3>>();
		}

		/**
		 * @see : http://docs.opencv.org/ref/master/d0/d3a/classcv_1_1DataType.html
		 */
		void helper_tests()
		{
			std::cout << "helper tests ... ";
			std::cout.flush();

			// single channel
			// ===============
			compare_scalar_to_pixel<bool>();

			compare_scalar_to_pixel<unsigned char>();
			compare_scalar_to_pixel<signed char>();

			compare_scalar_to_pixel<unsigned short>();
			compare_scalar_to_pixel<signed short>();

			compare_scalar_to_pixel<int>();

			compare_scalar_to_pixel<float>();
			compare_scalar_to_pixel<double>();

			// tripple channel
			// ===============
			compare_scalar_to_pixel_vec<bool>();

			compare_scalar_to_pixel_vec<unsigned char>();
			compare_scalar_to_pixel_vec<signed char>();

			compare_scalar_to_pixel_vec<unsigned short>();
			compare_scalar_to_pixel_vec<signed short>();

			compare_scalar_to_pixel_vec<int>();

			compare_scalar_to_pixel_vec<float>();
			compare_scalar_to_pixel_vec<double>();

			std::cout << "passed :)\n";


			std::cout << "types:\n";
			std::cout << "  bool:   " << img_type_to_str(cv::DataType<bool>::type) << "\n";
			std::cout << "  uchar:  " << img_type_to_str(cv::DataType<unsigned char>::type) << "\n";
			std::cout << "  char:   " << img_type_to_str(cv::DataType<signed char>::type) << "\n";
			std::cout << "  ushort: " << img_type_to_str(cv::DataType<unsigned short>::type) << "\n";
			std::cout << "  short:  " << img_type_to_str(cv::DataType<short>::type) << "\n";
			std::cout << "  int:    " << img_type_to_str(cv::DataType<int>::type) << "\n";
			std::cout << "  float:  " << img_type_to_str(cv::DataType<float>::type) << "\n";
			std::cout << "  double: " << img_type_to_str(cv::DataType<double>::type) << "\n";
		}

	}

}
