/*
 * test_helper.h
 *
 *  Created on: Apr 14, 2015
 *      Author: tobias
 */

#ifndef TEST_HELPER_H_
#define TEST_HELPER_H_

#include <stdexcept>   // std::logic_error
#include <algorithm>   // std::equal
#include <iostream>    // std::cout
#include <iterator>    // std::distance

namespace heyho {

	namespace tests {

		struct assertion_error : std::logic_error
		{
			using std::logic_error::logic_error;
		};

		template<typename InputIt1, typename InputIt2>
		void range_compare(InputIt1 expected_first, InputIt1 expected_last, InputIt2 first, InputIt2 last) {
			const auto size1 = std::distance(expected_first, expected_last);
			const auto size2 = std::distance(first, last);

			if (size1 != size2 || !std::equal(expected_first, expected_last, first)) {
				std::cout << "expected:";
				for (; expected_first != expected_last; ++expected_first) {
					std::cout << " " << *expected_first;
				}
				std::cout << "\n";
				std::cout << "got:";
				for (; first != last; ++first) {
					std::cout << " " << *first;
				}
				std::cout << "\n";
				throw assertion_error("ranges differ :(");
			}
		}

		template<typename C1, typename C2>
		inline void container_compare(const C1 &expected, const C2 &c) {
			range_compare(expected.cbegin(), expected.cend(), c.cbegin(), c.cend());
		}

	}

}

#endif /* TEST_HELPER_H_ */
