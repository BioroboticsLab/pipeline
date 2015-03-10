/*
 * line_iterator.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: tobias
 */

#include "line_iterator_cv.h"
#include "line_iterator.h"
#include "line_iterator_test_helper.h"

namespace heyho {

	namespace tests {

		void line_iterator_tests()
		{
			const auto compare = [](line_iterator it, line_iterator_cv expected) {
				const auto pts_got      = line_it_2_vec(it);
				const auto pts_expected = line_it_2_vec(expected);
				if (pts_got.size() != pts_expected.size() || !std::equal(pts_got.cbegin(), pts_got.cend(), pts_expected.cbegin())) {
					std::cout << "expected:";
					for (const auto p : pts_expected) {
						std::cout << " " << p;
					}
					std::cout << "\n";
					std::cout << "got:";
					for (const auto p : pts_got) {
						std::cout << " " << p;
					}
					std::cout << "\n";
					throw std::runtime_error(":(");
				}
			};

			std::cout << "line iterator tests ... ";
			std::cout.flush();

			// compare line_iterator with line_iterator_cv : test every unclipped line
			for(int x = -10; x <= 10; ++x) {
				for (int y = -10; y <= 10; ++y) {
					const cv::Size size(50,50);
					const cv::Point p1(25, 25);
					const cv::Point vec(x,y);
					compare(
						line_iterator   (p1, p1 + vec, 8      ),
						line_iterator_cv(p1, p1 + vec, 8, size)
					);

					compare(
						line_iterator   (p1, p1 + vec, 4      ),
						line_iterator_cv(p1, p1 + vec, 4, size)
					);
				}
			}

			std::cout << "passed :)\n";
		}

	}


}


