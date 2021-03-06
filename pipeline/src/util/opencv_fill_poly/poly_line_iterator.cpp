/*
 * poly_line_iterator.cpp
 *
 *  Created on: Mar 4, 2015
 *      Author: tobias
 */

#include "../../../util/opencv_fill_poly/poly_line_iterator.h"
#include <vector>         // std::vector
#include <iostream>       // std::cout
#include <algorithm>      // std::equal
#include "../../../util/opencv_fill_poly/test_helper.h"  // container_compare

namespace heyho {

	namespace tests {
		void poly_line_iterator_tests() {

			/**
			 * poly line points --> ring_iterator --> poly_line_iterator --> points
			 */
			const auto line_2_points = [](std::vector<cv::Point> poly_points) {
				using BDIT = std::vector<cv::Point>::const_iterator;
				using ring = ring_iterator_bd<BDIT>;
				using poly_line = poly_line_iterator<ring, line_iterator_cv>;

				poly_line pl(ring(poly_points.cbegin(), poly_points.cend(), poly_points.cbegin(), poly_points.cend() - 1, false), connectivity::eight_connected);
				std::vector<cv::Point> pl_points;
				for (; !pl.end(); ++pl) {
					pl_points.emplace_back(*pl);
				}
				return pl_points;
			};

			const auto compare = [&line_2_points](std::vector<cv::Point> poly_points, std::vector<cv::Point> expected) {
				container_compare(
					expected,
					line_2_points(poly_points)
				);
			};

			std::cout << "poly line iterator tests ... ";
			std::cout.flush();

			// 1 point
			//=========
			compare({{4,5}}, {{4,5}});

			// 2 points
			// =========
			compare({{1,1},{1,1}}, {{1,1}});
			compare({{1,1},{1,3}}, {{1,1},{1,2},{1,3}});

			// duplicate points at begin
			compare({{1,1},{1,1},{1,1},{1,3}}, {{1,1},{1,2},{1,3}});

			// duplicate points in middle
			compare({{1,1},{1,2},{1,2},{1,2},{1,3}}, {{1,1},{1,2},{1,3}});

			// duplicate points at end
			compare({{1,1},{1,3},{1,3},{1,3}}, {{1,1},{1,2},{1,3}});

			// random points
			compare({{-2,1},{-1,1},{0,1},{2,3},{2,3},{0,3},{-3,3}}, {{-2,1},{-1,1},{0,1},{1,2},{2,3},{1,3},{0,3},{-1,3},{-2,3},{-3,3}});

			std::cout << "passed :)\n";
		}
	}

}
