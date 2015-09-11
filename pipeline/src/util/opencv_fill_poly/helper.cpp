/*
 * helper.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: tobias
 */

#include "../../../util/opencv_fill_poly/helper.h"
#include "../../../util/opencv_fill_poly/test_helper.h" // assertion_error
#include <iostream>      // std::cout
#include <limits>        // std::numeric_limits

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
		 */
		template<typename T>
		T cv_scalar_2_pixel(const cv::Scalar &color) {
			cv::Mat m(1, 1, cv::DataType<T>::type, color);
			return m.template at<T>(0, 0);
		}

		template<typename T>
		void compare_scalar_to_pixel(const cv::Scalar &color)
		{
			if (cv_scalar_2_pixel<T>(color) != scalar2pixel<T>(color)) {
				throw assertion_error("scalar2pixel failed :( - " + img_type_to_str(cv::DataType<T>::type));
			}
		}

		template<typename pixel_t>
		void compare_scalar_to_pixel() {
			using chan_t = typename cv::DataType<pixel_t>::channel_type;
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

		template<template<typename> class C>
		void compare_scalar_to_pixel_c() {
			compare_scalar_to_pixel<C<unsigned char>>();
			compare_scalar_to_pixel<C<signed char>>();
			compare_scalar_to_pixel<C<unsigned short>>();
			compare_scalar_to_pixel<C<signed short>>();
			compare_scalar_to_pixel<C<int>>();
			compare_scalar_to_pixel<C<float>>();
			compare_scalar_to_pixel<C<double>>();

		}

		template<typename T>
		using vec1 = cv::Vec<T, 1>;

		template<typename T>
		using vec2 = cv::Vec<T, 2>;

		template<typename T>
		using vec3 = cv::Vec<T, 3>;

		template<typename T>
		using vec4 = cv::Vec<T, 4>;

		template<typename T>
		using mat11 = cv::Matx<T, 1, 1>;

		template<typename T>
		using mat12 = cv::Matx<T, 1, 2>;

		template<typename T>
		using mat13 = cv::Matx<T, 1, 3>;

		template<typename T>
		using mat14 = cv::Matx<T, 1, 4>;

		template<typename T>
		using mat21 = cv::Matx<T, 2, 1>;

		template<typename T>
		using mat22 = cv::Matx<T, 2, 2>;

		template<typename T>
		using mat31 = cv::Matx<T, 3, 1>;

		template<typename T>
		using mat41 = cv::Matx<T, 4, 1>;








		/**
		 * @see : http://docs.opencv.org/ref/master/d0/d3a/classcv_1_1DataType.html
		 */
		void helper_tests()
		{
			{
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

			std::cout << "helper tests ... ";
			std::cout.flush();

			{
				static_assert(! is_primitive_open_cv_data_type<bool>{},           "");
				static_assert(is_primitive_open_cv_data_type<unsigned char>{},  "");
				static_assert(is_primitive_open_cv_data_type<char>{},           "");
				static_assert(is_primitive_open_cv_data_type<signed char>{},    "");
				static_assert(is_primitive_open_cv_data_type<unsigned short>{}, "");
				static_assert(is_primitive_open_cv_data_type<short>{},          "");
				static_assert(is_primitive_open_cv_data_type<int>{},            "");
				static_assert(is_primitive_open_cv_data_type<float>{},          "");
				static_assert(is_primitive_open_cv_data_type<double>{},         "");

				static_assert(! is_primitive_open_cv_data_type<unsigned int>{},       "");
				static_assert(! is_primitive_open_cv_data_type<int*>{},               "");
				struct foo {};
				static_assert(! is_primitive_open_cv_data_type<foo>{}, "");
			}

			{
				// single channel
				// ===============
				compare_scalar_to_pixel<unsigned char>();
				compare_scalar_to_pixel<signed char>();

				compare_scalar_to_pixel<unsigned short>();
				compare_scalar_to_pixel<signed short>();

				compare_scalar_to_pixel<int>();

				compare_scalar_to_pixel<float>();
				compare_scalar_to_pixel<double>();



				// Point
				compare_scalar_to_pixel_c<cv::Point_>();
				compare_scalar_to_pixel_c<cv::Point3_>();

				// Vec
				compare_scalar_to_pixel_c<vec1>();
				compare_scalar_to_pixel_c<vec2>();
				compare_scalar_to_pixel_c<vec3>();
				compare_scalar_to_pixel_c<vec4>();

				// mat
				/*
				compare_scalar_to_pixel_c<mat11>();
				compare_scalar_to_pixel_c<mat12>();
				compare_scalar_to_pixel_c<mat13>();
				compare_scalar_to_pixel_c<mat14>();
				compare_scalar_to_pixel_c<mat21>();
				compare_scalar_to_pixel_c<mat22>();
				compare_scalar_to_pixel_c<mat31>();
				compare_scalar_to_pixel_c<mat41>();
				*/

				std::cout << "passed :)\n";
			}


		}

	}

}
