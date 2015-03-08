/*
 * ring_iterator.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: tobias
 */

#include "ring_iterator.h"
#include <vector>          // std::vector
#include <stdexcept>       // std::runtime_error
#include <iostream>        // std::cout

namespace heyho {

	namespace tests {

		void ring_iterator_tests() {

			using ring_it_t = heyho::ring_iterator_bd<std::vector<int>::const_iterator>;

			const auto my_assert = [](bool b) {
				if (!b) {
					throw std::runtime_error(":(");
				}
			};
			const std::vector<int> v{1, 2, 3};

			// 0 elements
			{
				try {
					ring_it_t it(v.cbegin(), v.cbegin(), v.cbegin(), v.cbegin(), false);
					my_assert(false);
				}
				catch (const std::invalid_argument&) {}
			}

			std::cout << "ring iterator tests ... ";
			std::cout.flush();

			// forward
			{
				// 1 element
				{
					ring_it_t it(v.cbegin(), v.cend(), v.cbegin(), v.cbegin(), false);
					my_assert( !it.end() && it.last() && *it == 1 );
					++it;
					my_assert( it.end() && !it.last() );
				}

				// 2 elements
				{
					ring_it_t it(v.cbegin(), v.cend(), v.cbegin(), v.cbegin() + 1, false);
					my_assert( !it.end() && !it.last() && *it == 1 );
					++it;
					my_assert( !it.end() && it.last() && *it == 2 );
					++it;
					my_assert( it.end() && !it.last() );
				}

				// 2 elements crossing end
				{
					ring_it_t it(v.cbegin(), v.cend(), v.cbegin() + 2, v.cbegin(), false);
					my_assert( !it.end() && !it.last() && *it == 3 );
					++it;
					my_assert( !it.end() && it.last() && *it == 1 );
					++it;
					my_assert( it.end() && !it.last() );
				}

				// 3 elements crossing end
				{
					ring_it_t it(v.cbegin(), v.cend(), v.cbegin() + 2, v.cbegin() + 1, false);
					my_assert( !it.end() && !it.last() && *it == 3 );
					++it;
					my_assert( !it.end() && !it.last() && *it == 1 );
					++it;
					my_assert( !it.end() && it.last() && *it == 2 );
					++it;
					my_assert( it.end() && !it.last() );
				}
			}

			// reversed
			{
				// 1 element
				{
					ring_it_t it(v.cbegin(), v.cend(), v.cbegin(), v.cbegin(), true);
					my_assert( !it.end() && it.last() && *it == 1 );
					++it;
					my_assert( it.end() && !it.last() );
				}

				// 2 elements
				{
					ring_it_t it(v.cbegin(), v.cend(), v.cbegin() + 1, v.cbegin(), true);
					my_assert( !it.end() && !it.last() && *it == 2 );
					++it;
					my_assert( !it.end() && it.last() && *it == 1 );
					++it;
					my_assert( it.end() && !it.last() );
				}

				// 2 elements crossing start
				{
					ring_it_t it(v.cbegin(), v.cend(), v.cbegin(), v.cbegin() + 2, true);
					my_assert( !it.end() && !it.last() && *it == 1 );
					++it;
					my_assert( !it.end() && it.last() && *it == 3 );
					++it;
					my_assert( it.end() && !it.last() );
				}

				// 3 elements crossing start
				{
					ring_it_t it(v.cbegin(), v.cend(), v.cbegin() + 1, v.cbegin() + 2, true);
					my_assert( !it.end() && !it.last() && *it == 2 );
					++it;
					my_assert( !it.end() && !it.last() && *it == 1 );
					++it;
					my_assert( !it.end() && it.last() && *it == 3 );
					++it;
					my_assert( it.end() && !it.last() );
				}
			} // END reversed
			std::cout << "passed :)\n";
		} // END ring_iterator_tests();

	} // END namespace tests

}
