/*
 * fill_convex_poly_test_functors.h
 *
 *  Created on: Mar 12, 2015
 *      Author: tobias
 */

#ifndef FILL_CONVEX_POLY_TEST_FUNCTORS_H_
#define FILL_CONVEX_POLY_TEST_FUNCTORS_H_

#include "line_iterator_cv.h"
#include "line_iterator.h"
#include "fill_convex_poly_cv.h"
#include "fill_convex_poly.h"
#include "fill convex_poly_test_colors.h"
#include <utility>                       // std::pair

namespace heyho {

	namespace tests {



		using fill_convex_poly_f  = void                      (*) (      cv::Mat &img, cv::InputArray points, const cv::Scalar& color, int line_type);
		using count_convex_poly_f = std::pair<size_t, size_t> (*) (const cv::Mat &img, cv::InputArray points,                          int line_type);


		/**
		 * @return {name of function wrapped by F, pointer to wrapped paint function}
		 */
		template<typename F, typename pixel_t>
		std::pair<std::string, fill_convex_poly_f> paint_function() {
			return {F::name(), &F::template paint<pixel_t>};
		}

		/**
		 * @return {name of function wrapped by F, pointer to wrapped count function}
		 */
		template<typename F, typename pixel_t>
		std::pair<std::string, count_convex_poly_f> count_function() {
			return {F::name(), &F::template count<pixel_t>};
		}

		/***************************************************************************
		 *
		 * FUNCTORS:
		 *
		 * - wrap open cv & heyho fill / count functions
		 *
		 * - provide 3 static methods:
		 *   - paint
		 *   - count
		 *
		 */

		namespace detail {
			template<typename LINE_IT>
			std::string line_it_name();
			template<>
			inline std::string line_it_name<heyho::line_iterator_cv>() { return "line_iterator_cv"; }
			template<>
			inline std::string line_it_name<heyho::line_iterator   >() { return "line_iterator";    }
		}

		struct open_cv_fill_poly_f
		{
			static std::string name() {
				return "cv::fillConvexPoly";
			}
			template<typename pixel_t>
			static void paint(cv::Mat &img, cv::InputArray points, const cv::Scalar& color, int line_type) {
				cv::fillConvexPoly(img, points, color, line_type, 0);
			}
			template<typename pixel_t>
			static std::pair<size_t, size_t> count(const cv::Mat &img, cv::InputArray points, int line_type) {
				cv::Mat poly_img(img.size(), img.type(), pixel_2_white<pixel_t>());
				cv::fillConvexPoly(poly_img, points, pixel_2_black<pixel_t>(), line_type);
				const size_t all = static_cast<size_t>(cv::countNonZero(poly_img));
				poly_img &= img;
				const size_t non_zero = static_cast<size_t>(cv::countNonZero(poly_img));
				const size_t zero = all - non_zero;
				return {zero, non_zero};
			}
		};

		template<typename LINE_IT>
		struct heyho_fill_poly_cv_f
		{
			static std::string name() {
				return "heyho::fill_convex_poly_cv<" + detail::line_it_name<LINE_IT>() + ">";
			}
			template<typename pixel_t>
			static void paint(cv::Mat &img, cv::InputArray points, const cv::Scalar& color, int line_type) {
				heyho::fill_convex_poly_cv<pixel_t, LINE_IT>(img, points, color, line_type);
			}
			template<typename pixel_t>
			static std::pair<size_t, size_t> count(const cv::Mat &img, cv::InputArray points, int line_type) {
				const auto counts = heyho::convex_poly_cv<pixel_counter<pixel_t>, LINE_IT>(img.size(), points, pixel_counter<pixel_t>{img, pixel_2_white<pixel_t>()}, line_type).count();
				return {counts.zero(), counts.non_zero()};
			}
		};

		template<typename LINE_IT>
		struct heyho_fill_poly_f
		{
			static std::string name() {
				return "heyho::fill_convex_poly<" + detail::line_it_name<LINE_IT>() + ">";
			}
			template<typename pixel_t>
			static void paint(cv::Mat &img, cv::InputArray points, const cv::Scalar& color, int line_type) {
				heyho::fill_convex_poly<pixel_t, LINE_IT>(img, points, color, line_type);
			}
			template<typename pixel_t>
			static std::pair<size_t, size_t> count(const cv::Mat &img, cv::InputArray points, int line_type) {
				const auto counts = heyho::convex_poly<pixel_counter<pixel_t>, LINE_IT>(img.size(), points, pixel_counter<pixel_t>{img, pixel_2_white<pixel_t>()}, line_type).count();
				return {counts.zero(), counts.non_zero()};
			}
		};

	} // END namespace tests
} // END namespace heyho

#endif /* FILL_CONVEX_POLY_TEST_FUNCTORS_H_ */
