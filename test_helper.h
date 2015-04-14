/*
 * test_helper.h
 *
 *  Created on: Apr 14, 2015
 *      Author: tobias
 */

#ifndef TEST_HELPER_H_
#define TEST_HELPER_H_

#include <stdexcept>

namespace heyho {

	namespace tests {

		struct assertion_error : std::logic_error
		{
			using std::logic_error::logic_error;
		};

	}

}

#endif /* TEST_HELPER_H_ */
