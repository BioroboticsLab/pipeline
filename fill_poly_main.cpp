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

	std::vector<ell_f> e{
		&opencv_ellipse,
		&opencv_fill_poly
		, &opencv_fill_poly2
	};

	bench(e, 400, 5000);

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
