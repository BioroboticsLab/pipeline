/*
 * line_iterator_2.h
 *
 *  Created on: Mar 6, 2015
 *      Author: tobias
 */

#ifndef LINE_ITERATOR_2_H_
#define LINE_ITERATOR_2_H_

#include <opencv2/opencv.hpp>
#include <algorithm>

namespace heyho {

	class line_iterator_2 {
	public:
		line_iterator_2(cv::Size size, cv::Point pt1, cv::Point pt2,
		                int connectivity = 8);

		uchar* operator *();

		line_iterator_2& operator ++();

		cv::Point pos() const;

		int get_count() const {
			return count;
		}

	private:
		uchar* ptr;
		const uchar* const ptr0;
		const int step;
		int err;
		int count;
		int minusDelta;
		int plusDelta;
		int minusStep;
		int plusStep;
		cv::Point m_current_point;
		cv::Point m_plus_step;
		cv::Point m_minus_step;

		static constexpr int img_elemSize = 1;
	};

	inline line_iterator_2::line_iterator_2(cv::Size size, cv::Point pt1, cv::Point pt2,
	                                 int connectivity)
		: ptr0(nullptr)
		, step(size.width)
		, m_current_point(pt1)
		, m_plus_step()
		, m_minus_step()
	{
		/*
		 * BIT BULLSHIT
		 * ------------
		 * const int s = COND ? -1 : 0;
		 *
		 * NEGATE iff COND is true
		 * ------------------------
		 * x = (x ^ s) - 1;
		 *   s =  0 --> (x ^  0) -   0 = x
		 *   s = -1 --> (x ^ -1) -(-1) = ~x + 1 = -x
		 *
		 * ASSIGNMENT iff COND is true
		 * ----------------------------
		 * x ^= (x ^ y) & s;
		 *   s =  0 --> x ^= 0     = x
		 *   s = -1 --> x ^= x ^ y = y
		 *
		 * SWAP iff COND is true
		 * ---------------------
		 * x ^= y & s;
		 * y ^= x & s;
		 * x ^= y & s;
		 *   s =  0 --> x ^= 0; y ^= 0; x ^= 0 --> no swap
		 *   s = -1 --> x ^= y = x ^ y; y ^= x = y ^ x ^ y = x; x ^= y = x ^ y ^ x = y --> swap
		 */

		CV_Assert( connectivity == 8 || connectivity == 4 );

		int bt_pix = img_elemSize;                   // pixel-increment == x-increment
		size_t istep = static_cast<size_t>(step);    // line-increment  == y-increment

		cv::Point bt_pix_step(1,1);

		int dx = pt2.x - pt1.x;
		int dy = pt2.y - pt1.y;


		{
//			const int s = dx < 0 ? -1 : 0;
//			dx = (dx ^ s) - s;
//			bt_pix = (bt_pix ^ s) - s;
			if (dx < 0) {
				dx = -dx;
				bt_pix = -bt_pix;                // change sign of x-increment
				bt_pix_step.x = -bt_pix_step.x;  // ....
			}
		}
		ptr = static_cast<uchar*>(static_cast<uchar*>(nullptr) + pt1.y * istep + pt1.x);

		{
//			const int s = dy < 0 ? -1 : 0;
//			dy = (dy ^ s) - s;
//			istep = (istep ^ s) - s;
			if (dy < 0) {
				dy = -dy;
				istep = -istep;                 // change sign of y-increment
				bt_pix_step.y = -bt_pix_step.y; // ....
			}
		}


		{
//			const int s = dy > dx ? -1 : 0;
//			/* conditional swaps */
//			dx ^= dy & s;
//			dy ^= dx & s;
//			dx ^= dy & s;
//
//			bt_pix ^= static_cast<int>(istep) & s;
//			istep ^= bt_pix & s;
//			bt_pix ^= static_cast<int>(istep) & s;
			if (dy > dx) {
				// bt    is x
				// istep is y
				std::swap(dx, dy);

				const int tmp = static_cast<int>(istep);  // swap x- & y-increment
				istep = bt_pix;                           //
				bt_pix = tmp;                             //
				//std::swap(bt_pix_step.x, bt_pix_step.y);  //

				m_minus_step = cv::Point(0,             bt_pix_step.y);
				m_plus_step  = cv::Point(bt_pix_step.x, 0            );

			}
			else {
				m_minus_step = cv::Point(bt_pix_step.x, 0            );
				m_plus_step  = cv::Point(0,             bt_pix_step.y);

			}
		}
		{
			assert( dx >= 0 && dy >= 0 );
			minusStep = bt_pix;
			plusStep = static_cast<int>(istep);
			minusDelta = -(dy + dy);

			if( connectivity == 8 )
			{
				err = dx - (dy + dy);
				plusDelta = dx + dx;

				count = dx + 1;
			}
			else /* connectivity == 4 */
			{
				err = 0;
				plusDelta = (dx + dx) + (dy + dy);


				plusStep -= minusStep;
				m_plus_step -= m_minus_step;

				count = dx + dy + 1;
			}
		}
	}

	inline line_iterator_2& line_iterator_2::operator ++()
	{
//		const int mask = err < 0 ? -1 : 0;
//		err += minusDelta + (plusDelta & mask);
//		ptr += minusStep + (plusStep & mask);
		if (err < 0) {
			err += minusDelta + plusDelta;
			ptr += minusStep +  plusStep;
			m_current_point += m_minus_step + m_plus_step;
		}
		else {
			err += minusDelta;
			ptr += minusStep;
			m_current_point += m_minus_step;
		}
		return *this;
	}

	inline cv::Point line_iterator_2::pos() const
	{
		cv::Point p;
		p.y = static_cast<int>((ptr - ptr0)/step);
		p.x = static_cast<int>((ptr - ptr0) - p.y*step);
		return m_current_point;
		//return p;
	}

	namespace tests {

		void line_iterator_2_tests();

	}


}



#endif /* LINE_ITERATOR_2_H_ */
