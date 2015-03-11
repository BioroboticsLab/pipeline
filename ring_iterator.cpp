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
#include <iterator>        // std::revere_iterator

namespace heyho {

	namespace tests {

		namespace {
			inline void my_assert(bool b) {
				if (!b) {
					throw std::runtime_error(":(");
				}
			}
		}

		void ring_iterator_tests() {

			using ring_it_t          = heyho::ring_iterator<std::vector<int>::const_iterator, false>;
			using ring_it_reversed_t = heyho::ring_iterator<std::vector<int>::const_iterator, true>;

			using rev_it_t           = std::reverse_iterator<std::vector<int>::const_iterator>;
			using ring_it_rev_it_t   = heyho::ring_iterator<rev_it_t, false>;

			std::cout << "ring iterator tests ... ";
			std::cout.flush();

			const std::vector<int> v{1, 2, 3};

			// forward
			{
				// 0 elements
				{
					try {
						ring_it_t it_fwd(v.cbegin(), v.cbegin(), v.cbegin(), v.cbegin());
						my_assert(false);
					}
					catch (const std::invalid_argument&) {}
				}

				// 1 element
				{
					ring_it_t it_fwd(v.cbegin(), v.cend(), v.cbegin(), v.cbegin());
					my_assert( !it_fwd.end() && it_fwd.last() && *it_fwd == 1 );
					++it_fwd;
					my_assert( it_fwd.end() && !it_fwd.last() );
				}

				// 2 elements
				{
					ring_it_t it_fwd(v.cbegin(), v.cend(), v.cbegin(), v.cbegin() + 1);
					my_assert( !it_fwd.end() && !it_fwd.last() && *it_fwd == 1 );
					++it_fwd;
					my_assert( !it_fwd.end() && it_fwd.last() && *it_fwd == 2 );
					++it_fwd;
					my_assert( it_fwd.end() && !it_fwd.last() );
				}

				// 2 elements crossing end
				{
					ring_it_t it_fwd(v.cbegin(), v.cend(), v.cbegin() + 2, v.cbegin());
					my_assert( !it_fwd.end() && !it_fwd.last() && *it_fwd == 3 );
					++it_fwd;
					my_assert( !it_fwd.end() && it_fwd.last() && *it_fwd == 1 );
					++it_fwd;
					my_assert( it_fwd.end() && !it_fwd.last() );
				}

				// 3 elements crossing end
				{
					ring_it_t it_fwd(v.cbegin(), v.cend(), v.cbegin() + 2, v.cbegin() + 1);
					my_assert( !it_fwd.end() && !it_fwd.last() && *it_fwd == 3 );
					++it_fwd;
					my_assert( !it_fwd.end() && !it_fwd.last() && *it_fwd == 1 );
					++it_fwd;
					my_assert( !it_fwd.end() && it_fwd.last() && *it_fwd == 2 );
					++it_fwd;
					my_assert( it_fwd.end() && !it_fwd.last() );
				}
			}

			// reversed using std::reverse iterator
			{
				// 0 elements
				{
					try {
						ring_it_rev_it_t it_rev(rev_it_t{v.cbegin()}, rev_it_t{v.cbegin()}, rev_it_t{(v.cbegin()) + 1}, rev_it_t{(v.cbegin()) + 1});
						my_assert(false);
					}
					catch (const std::invalid_argument&) {}
				}

				// 1 element
				{
					ring_it_rev_it_t it_rev(rev_it_t{v.cend()}, rev_it_t{v.cbegin()}, rev_it_t{(v.cbegin()) + 1}, rev_it_t{(v.cbegin()) + 1});
					my_assert( !it_rev.end() && it_rev.last() && *it_rev == 1 );
					++it_rev;
					my_assert( it_rev.end() && !it_rev.last() );
				}

				// 2 elements
				{
					ring_it_rev_it_t it_rev(rev_it_t{v.cend()}, rev_it_t{v.cbegin()}, rev_it_t{(v.cbegin() + 1) + 1}, rev_it_t{(v.cbegin()) + 1});
					my_assert( !it_rev.end() && !it_rev.last() && *it_rev == 2 );
					++it_rev;
					my_assert( !it_rev.end() && it_rev.last() && *it_rev == 1 );
					++it_rev;
					my_assert( it_rev.end() && !it_rev.last() );
				}

				// 2 elements crossing start
				{
					ring_it_rev_it_t it_rev(rev_it_t{v.cend()}, rev_it_t{v.cbegin()}, rev_it_t{(v.cbegin()) + 1}, rev_it_t{(v.cbegin() + 2) + 1});
					my_assert( !it_rev.end() && !it_rev.last() && *it_rev == 1 );
					++it_rev;
					my_assert( !it_rev.end() && it_rev.last() && *it_rev == 3 );
					++it_rev;
					my_assert( it_rev.end() && !it_rev.last() );
				}

				// 3 elements crossing start
				{
					ring_it_rev_it_t it_rev(rev_it_t{v.cend()}, rev_it_t{v.cbegin()}, rev_it_t{(v.cbegin() + 1) + 1}, rev_it_t{(v.cbegin() + 2) + 1});
					my_assert( !it_rev.end() && !it_rev.last() && *it_rev == 2 );
					++it_rev;
					my_assert( !it_rev.end() && !it_rev.last() && *it_rev == 1 );
					++it_rev;
					my_assert( !it_rev.end() && it_rev.last() && *it_rev == 3 );
					++it_rev;
					my_assert( it_rev.end() && !it_rev.last() );
				}
			} // END reversed (std::reverse iterator)

			// reversed using reverse template param
			{
				// 0 elements
				{
					try {
						ring_it_reversed_t it_rev(v.cbegin(), v.cbegin(), v.cbegin(), v.cbegin());
						my_assert(false);
					}
					catch (const std::invalid_argument&) {}
				}

				// 1 element
				{
					ring_it_reversed_t it_rev(v.cbegin(), v.cend(), v.cbegin(), v.cbegin());
					my_assert( !it_rev.end() && it_rev.last() && *it_rev == 1 );
					++it_rev;
					my_assert( it_rev.end() && !it_rev.last() );
				}

				// 2 elements
				{
					ring_it_reversed_t it_rev(v.cbegin(), v.cend(), v.cbegin() + 1, v.cbegin());
					my_assert( !it_rev.end() && !it_rev.last() && *it_rev == 2 );
					++it_rev;
					my_assert( !it_rev.end() && it_rev.last() && *it_rev == 1 );
					++it_rev;
					my_assert( it_rev.end() && !it_rev.last() );
				}

				// 2 elements crossing start
				{
					ring_it_reversed_t it_rev(v.cbegin(), v.cend(), v.cbegin(), v.cbegin() + 2);
					my_assert( !it_rev.end() && !it_rev.last() && *it_rev == 1 );
					++it_rev;
					my_assert( !it_rev.end() && it_rev.last() && *it_rev == 3 );
					++it_rev;
					my_assert( it_rev.end() && !it_rev.last() );
				}

				// 3 elements crossing start
				{
					ring_it_reversed_t it_rev(v.cbegin(), v.cend(), v.cbegin() + 1, v.cbegin() + 2);
					my_assert( !it_rev.end() && !it_rev.last() && *it_rev == 2 );
					++it_rev;
					my_assert( !it_rev.end() && !it_rev.last() && *it_rev == 1 );
					++it_rev;
					my_assert( !it_rev.end() && it_rev.last() && *it_rev == 3 );
					++it_rev;
					my_assert( it_rev.end() && !it_rev.last() );
				}
			} // END reversed (template param)

			std::cout << "passed :)\n";
		} // END ring_iterator_tests();


		void ring_iterator_bd_tests() {

			using ring_it_t = heyho::ring_iterator_bd<std::vector<int>::const_iterator>;

			std::cout << "ring iterator bd tests ... ";
			std::cout.flush();

			const std::vector<int> v{1, 2, 3};

			// 0 elements
			{
				try {
					ring_it_t it(v.cbegin(), v.cbegin(), v.cbegin(), v.cbegin(), false);
					my_assert(false);
				}
				catch (const std::invalid_argument&) {}
			}
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
		} // END ring_iterator_bd_tests();

	} // END namespace tests

}
