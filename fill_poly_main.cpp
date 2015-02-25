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
		// line_types: {8, 4, CV_AA}

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



using ell_f = void (*) (cv::Mat &img, cv::Point center, cv::Size axes, double angle);
const cv::Scalar BLUE(255,    0,   0);
const cv::Scalar WHITE(255, 255, 255);


/**
 * draw outer filled ellipse; draw filled inner ellipse
 */
void opencv_ellipse(cv::Mat &img, cv::Point center, cv::Size axes, double angle) {
	cv::ellipse(img, center, axes, angle, 0, 360, BLUE, -1);
	cv::ellipse(img, center, {axes.width / 2, axes.height / 2}, angle, 0, 360, WHITE, -1);
}

/**
 * connect outer & inner ellipse points into one polygon & draw
 *
 * (closing ellipses & reversing inner ellipse seems unnecessary ...)
 *
 */
void opencv_fill_poly(cv::Mat &img, cv::Point center, cv::Size axes, double angle) {

	std::vector<std::vector<cv::Point>> points(1);
	cv::ellipse2Poly(center, axes, static_cast<int>(std::round(angle)), 0, 360, 1, points[0]);
	//points[0].push_back(points[0].front());

	std::vector<cv::Point> tmp;
	cv::ellipse2Poly(center, {axes.width / 2, axes.height / 2}, static_cast<int>(std::round(angle)), 0, 360, 1, tmp);
	//tmp.push_back(tmp.front());

	points[0].insert(points[0].end(), tmp.begin(), tmp.end());
	//points[0].insert(points[0].end(), tmp.rbegin(), tmp.rend());
	cv::fillPoly(img, points, BLUE);
}

/**
 * pass inner and outer ellipse in separate vecs to cv::fillPoly
 */
void opencv_fill_poly2(cv::Mat &img, cv::Point center, cv::Size axes, double angle) {

	std::vector<std::vector<cv::Point>> points(2);
	cv::ellipse2Poly(center, axes, static_cast<int>(std::round(angle)), 0, 360, 1, points[0]);
	cv::ellipse2Poly(center, {axes.width / 2, axes.height / 2}, static_cast<int>(std::round(angle)), 0, 360, 1, points[1]);

	cv::fillPoly(img, points, BLUE);
}

void bench(const std::vector<ell_f> &e, int major, size_t times) {
	const int axis_major = major;
	const int axis_minor = axis_major / 2;
	const int dim_y = 2 * std::max(axis_major, axis_minor) + 10;
	const int dim_x = dim_y;
	const cv::Point center(dim_x/2, dim_y/2);

	cv::Mat img(dim_y, dim_x, CV_8UC3, WHITE);

	for (auto f : e) {
		timer t;
		for (size_t i = 0; i < times; ++i) {
			f(img, center, cv::Size(axis_major, axis_minor), 45);
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





	std::vector<ell_f> e{
		&opencv_ellipse,
		&opencv_fill_poly,
		&opencv_fill_poly2
	};

	if (false) {

		bench(e, 400, 5000);
	}

	if (false) {

		constexpr int axis_major = 100;
		constexpr int axis_minor = 50;
		const int dim_y = 2 * std::max(axis_major, axis_minor) + 10;
		const int dim_x = dim_y * static_cast<int>(e.size());

		cv::Mat img(dim_y, dim_x, CV_8UC3, WHITE);

		for (size_t i = 0; i < e.size(); ++i) {
			const cv::Point center(dim_y/2 + static_cast<int>(i) * dim_y, dim_y/2);
			e[i](img, center, cv::Size(axis_major, axis_minor), 45);
		}

		cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE);
		cv::imshow( "Display window", img);
		cv::waitKey(0);
	}

	return 0;
}
