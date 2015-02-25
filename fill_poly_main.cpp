/*
 * fill_poly_main.cpp
 *
 *  Created on: Feb 20, 2015
 *      Author: tobias
 */



#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <chrono>
#include <exception>
#include <cstring>
#include "blargs.h"



template<int>
cv::Scalar white();
template<>
inline cv::Scalar white<CV_8UC1>() { return {255}; }
template<>
inline cv::Scalar white<CV_8UC3>() { return {255, 255, 255}; }
template<>
inline cv::Scalar white<CV_32FC1>() { return {1.0f}; }
template<>
inline cv::Scalar white<CV_32FC3>() { return {1.0f, 1.0f, 1.0f}; }


template<int>
cv::Scalar default_color();
template<>
inline cv::Scalar default_color<CV_8UC1>() { return {127}; }
template<>
inline cv::Scalar default_color<CV_8UC3>() { return {127, 255, 5}; }
template<>
inline cv::Scalar default_color<CV_32FC1>() { return {0.4f}; }
template<>
inline cv::Scalar default_color<CV_32FC3>() { return {1.0f, 0.0f, 0.2f}; }


/**
 * compares cv::fillConvexPoly & heyho::fillConvexPoly
 */
struct compare {

	const cv::Size axes;
	const int angle;

	template<typename pixel_t, int line_type>
	bool operator()() const {
		constexpr int img_type = cv::DataType<pixel_t>::type;
		constexpr int shift = 0;
		const int dim = 2 * std::max(axes.width, axes.height) + 10;
		const cv::Point center(dim / 2, dim / 2);
		std::vector<cv::Point> points;
		cv::ellipse2Poly(center, axes, angle, 0, 360, 1, points);


		cv::Mat img1(dim, dim, img_type, white<img_type>());
		cv::fillConvexPoly(img1, points, default_color<img_type>(), line_type, shift);

		cv::Mat img2(dim, dim, img_type, white<img_type>());
		heyho::fillConvexPoly<pixel_t>(img2, points, default_color<img_type>(), line_type);


		const bool equal =  0 == std::memcmp(img1.datastart, img2.datastart, img1.dataend - img1.datastart);

		if (! equal) {

			cv::namedWindow( "opencv", cv::WINDOW_AUTOSIZE);
			cv::imshow( "opencv", img1);

			cv::namedWindow( "heyho", cv::WINDOW_AUTOSIZE);
			cv::imshow( "heyho", img2);

			const cv::Mat diff = img1 != img2;

			cv::namedWindow( "diff", cv::WINDOW_AUTOSIZE);
			cv::imshow( "diff", diff);

			cv::waitKey(0);


		}

		return equal;
	}
};


/**
 * calls f with each combination of CV image type & CV line type
 */
struct foreach {

	template<typename F>
	void operator()(F f) const {

		// img_types:  {CV_8UC1, CV_8UC3, CV_32FC1, CV_32FC3}
		// line_types: {8, 4}

		using t_8UC1 = uint8_t;
		using t_8UC3 = cv::Vec<uint8_t, 3>;
		using t_32FC1 = float;
		using t_32FC3 = cv::Vec<float, 3>;

		if (! f.operator()<t_8UC1,  8    >() ) {throw std::runtime_error("");}
		if (! f.operator()<t_8UC1,  4    >() ) {throw std::runtime_error("");}

		if (! f.operator()<t_8UC3,  8    >() ) {throw std::runtime_error("");}
		if (! f.operator()<t_8UC3,  4    >() ) {throw std::runtime_error("");}

		if (! f.operator()<t_32FC1, 8    >() ) {throw std::runtime_error("");}
		if (! f.operator()<t_32FC1, 4    >() ) {throw std::runtime_error("");}

		if (! f.operator()<t_32FC3, 8    >() ) {throw std::runtime_error("");}
		if (! f.operator()<t_32FC3, 4    >() ) {throw std::runtime_error("");}
	}

};



class timer {
private:
	const std::chrono::system_clock::time_point start;
public:
	explicit timer()
	: start(std::chrono::system_clock::now())
	{}
	~timer() {
		const std::chrono::system_clock::time_point stop = std::chrono::system_clock::now();
		const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
		std::cout << ms.count() << " ms\n";
	}
};


const cv::Scalar BLUE(255,    0,   0);
const cv::Scalar WHITE(255, 255, 255);



using fill_convex_poly_f = void (*) (cv::InputOutputArray img, cv::InputArray points, const cv::Scalar& color, int line_type);

inline void cv_fill_confex_poly(cv::InputOutputArray img, cv::InputArray points, const cv::Scalar& color, int line_type) {
	cv::fillConvexPoly(img, points, color, line_type);
}

void bench_fill_convex_poly(const std::vector<fill_convex_poly_f> &e, int major, size_t times) {
	const int axis_major = major;
	const int axis_minor = axis_major / 2;
	const int dim_y = 2 * std::max(axis_major, axis_minor) + 10;
	const int dim_x = dim_y;
	const cv::Point center(dim_x/2, dim_y/2);
	std::vector<cv::Point> poly;
	cv::ellipse2Poly(center, cv::Size(axis_major, axis_minor), 45, 0, 360, 1, poly);
	cv::Mat img(dim_y, dim_x, CV_8UC3, WHITE);

	for (auto f : e) {
		timer t;
		for (size_t i = 0; i < times; ++i) {
			f(img, poly, BLUE, 4);
		}
	}
}

int main() {

	for (int a = 0; a < 45; a += 5) {
		for (int a1 = 25; a1 < 50; ++a1) {
			for (int a2 = a1; a2 < 50; ++a2) {
				foreach()(compare{{a1, a2}, a});
			}
		}
	}
	std::cout << "FOREACH: passed :)\n";





	std::vector<fill_convex_poly_f> e{
		&cv_fill_confex_poly,
		static_cast<fill_convex_poly_f>(&heyho::fillConvexPoly<cv::Vec<uint8_t, 3>>)
	};

	if (false or true) {

		bench_fill_convex_poly(e, 400, 5000);
	}

	return 0;
}
