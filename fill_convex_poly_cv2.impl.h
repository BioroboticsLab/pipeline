/*
 * fill_convex_poly_cv2.impl.h
 *
 *  Created on: Apr 21, 2015
 *      Author: tobias
 */

#ifndef FILL_CONVEX_POLY_CV2_IMPL_H_
#define FILL_CONVEX_POLY_CV2_IMPL_H_

namespace heyho {

template<typename LINE_IT, typename F, typename IT>
	F convex_poly_cv2(F f, cv::Size size, IT begin, IT end, connectivity line_type)
	{
		constexpr int FIXED_POINT_SHIFT    = 16;
		constexpr int FIXED_POINT_ONE      = 1 << FIXED_POINT_SHIFT;
		constexpr int FIXED_POINT_ONE_HALF = FIXED_POINT_ONE >> 1;

		class Edge
		{
		public:

			// CONSTRUCTOR
			Edge(IT ptr, bool reverse)
			: ptr(ptr), reverse(reverse), ye(ptr->y), x_shifted(), dx_shifted()
			{}

			// GETTERS
			const cv::Point& get_point() const { return *ptr; }
			int get_x_shifted() const { return x_shifted; }
			int get_x() const { return (x_shifted + FIXED_POINT_ONE_HALF) >> FIXED_POINT_SHIFT; }
			int get_ye() const { return ye; }

			// MODIFIERS
			void inc_x() { x_shifted += dx_shifted; }

			void update_dx(int y, int ty, int xs) {
				ye = ty;
				x_shifted = xs << FIXED_POINT_SHIFT;
				dx_shifted = (2*((get_point().x - xs) << FIXED_POINT_SHIFT) + (ye - y)) / (2 * (ye - y));
			}

			void advance(const IT begin, const IT end) {
				if (reverse) {
					if (ptr == begin) {
						ptr = end;
					}
					--ptr;
				}
				else {
					++ ptr;
					if (ptr == end) {
						ptr = begin;
					}
				}
			}

		private:
			IT ptr;
			const bool reverse;

			int ye;
			int x_shifted;
			int dx_shifted;
		};

		const auto advance = [begin, end] (Edge &e) {
			e.advance(begin, end);
		};

		if (begin == end) {
			return std::move(f);
		}

		// draw outline, calc min/max x/y coordinates
		size_t npts = 0;
		IT ptr_ymin = begin;
		int ymax = begin->y;
		{
			int xmin = begin->x;
			int xmax = begin->x;
			cv::Point p0 = *std::prev(end);
			for (auto ptr = begin; ptr != end; ++ptr, ++npts)
			{
				if( ptr->y < ptr_ymin->y )
				{
					ptr_ymin = ptr;
				}

				ymax = std::max( ymax, ptr->y );
				xmax = std::max( xmax, ptr->x );
				xmin = std::min( xmin, ptr->x );

				f = heyho::line<LINE_IT>(std::move(f), size, p0, *ptr, line_type, true);

				p0 = *ptr;
			}

			if ( npts < 3 || xmax < 0 || ymax < 0 || xmin >= size.width || ptr_ymin->y >= size.height ) {
				return std::move(f);
			}
		}

		ymax = std::min( ymax, size.height - 1 );

		Edge edge[2] {
			{ptr_ymin, false},
			{ptr_ymin, true}
		};

		{
			size_t remaining_edges = npts;
			int y = ptr_ymin->y;
			do
			{
				for (auto &e : edge)
				{
					if( y >= e.get_ye() )
					{
						int xs = 0;
						int ty = 0;

						for (;;)
						{
							ty = e.get_point().y;
							if( ty > y || remaining_edges == 0 ) {
								break;
							}
							xs = e.get_point().x;

							advance(e);

							--remaining_edges;
						}
						if ( y >= ty ) {
							return std::move(f);
						}
						e.update_dx(y, ty, xs);
					}
				}

				const int x1 = edge[0].get_x();
				const int x2 = edge[1].get_x();
				f = heyho::hline(
						std::move(f),
						size,
						std::min(x1, x2),
						std::max(x1, x2),
						y
				);

				for (auto &e : edge) {
					e.inc_x();
				}
			}
			while ( ++y <= ymax );
		}
		return std::move(f);
	}

	template<typename LINE_IT, typename F>
	inline F convex_poly_cv2(F f, cv::Size size, cv::InputArray points, connectivity line_type)
	{
		// cv::InputArray --> begin & end iterator; forward everything else
		const auto ptr_size = cv_point_input_array_to_pointer(points);
		return convex_poly_cv2<LINE_IT>(
			std::move(f),
			size,
			ptr_size.first,
			ptr_size.first + ptr_size.second,
			line_type
		);
	}

}

#endif /* FILL_CONVEX_POLY_CV2_IMPL_H_ */
