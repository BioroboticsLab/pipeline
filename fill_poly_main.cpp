/*
 * fill_poly_main.cpp
 *
 *  Created on: Feb 20, 2015
 *      Author: tobias
 */



#include <iostream>

#include "fill_convex_poly_cv.h"    // fill_convex_poly
#include "ring_iterator.h"          // ring_iterator_tests
#include "line_iterator.h"          // line_iterator_tests
#include "poly_line_iterator.h"     // poly_line_iterator_tests
#include "fill_convex_poly_tests.h" //






int main() {

	{
		heyho::tests::line_iterator_tests();
		heyho::tests::ring_iterator_tests();
		heyho::tests::poly_line_iterator_tests();
	}

	if (false or true)
	{
		const std::vector<heyho::tests::fill_convex_poly_f> fill_functions {
			&heyho::tests::cv_fill_confex_poly,
			static_cast<heyho::tests::fill_convex_poly_f>(&heyho::fill_convex_poly<uint8_t>)
		};
		heyho::tests::benchmark_fill_convex_poly_functions(fill_functions, 400, 5000);
		std::cout << '\n';
	}

	if (false or true)
	{
		std::cout << "BENCHMARK\n==================\n";
		heyho::tests::foreach()(heyho::tests::benchmark{{400, 200}, 45, 500});
		std::cout << '\n';
	}

	return 0;
}
