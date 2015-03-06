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
		                int connectivity=8, bool leftToRight=false);

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

		static constexpr int img_elemSize = 1;
	};

	inline line_iterator_2::line_iterator_2(cv::Size size, cv::Point pt1, cv::Point pt2,
	                                 int connectivity, bool left_to_right)
		: ptr0(nullptr)
		, step(size.width)
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
		count = -1;

		CV_Assert( connectivity == 8 || connectivity == 4 );

		int bt_pix = img_elemSize;
		size_t istep = static_cast<size_t>(step);

		int dx = pt2.x - pt1.x;
		int dy = pt2.y - pt1.y;

		{
//			const int s = dx < 0 ? -1 : 0;
			if( left_to_right )
			{
//				dx = (dx ^ s) - s;
//				dy = (dy ^ s) - s;
//				pt1.x ^= (pt1.x ^ pt2.x) & s;
//				pt1.y ^= (pt1.y ^ pt2.y) & s;
				if (dx < 0) {
					dx = -dx;
					dy = -dy;
					pt1 = pt2;
				}
			}
			else
			{
//				dx = (dx ^ s) - s;
//				bt_pix = (bt_pix ^ s) - s;
				if (dx < 0) {
					dx = -dx;
					bt_pix = -bt_pix;
				}
			}
		}
		ptr = static_cast<uchar*>(static_cast<uchar*>(nullptr) + pt1.y * istep + pt1.x);

		{
//			const int s = dy < 0 ? -1 : 0;
//			dy = (dy ^ s) - s;
//			istep = (istep ^ s) - s;
			if (dy < 0) {
				dy = -dy;
				istep = -istep;
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
				std::swap(dx, dy);
				const int tmp = static_cast<int>(istep);
				istep = bt_pix;
				bt_pix = tmp;

			}
		}

		if( connectivity == 8 )
		{
			assert( dx >= 0 && dy >= 0 );

			err = dx - (dy + dy);
			plusDelta = dx + dx;
			minusDelta = -(dy + dy);
			plusStep = static_cast<int>(istep);
			minusStep = bt_pix;
			count = dx + 1;
		}
		else /* connectivity == 4 */
		{
			assert( dx >= 0 && dy >= 0 );

			err = 0;
			plusDelta = (dx + dx) + (dy + dy);
			minusDelta = -(dy + dy);
			plusStep = static_cast<int>(istep) - bt_pix;
			minusStep = bt_pix;
			count = dx + dy + 1;
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
		}
		else {
			err += minusDelta;
			ptr += minusStep;
		}
		return *this;
	}

	inline cv::Point line_iterator_2::pos() const
	{
		cv::Point p;
		p.y = static_cast<int>((ptr - ptr0)/step);
		p.x = static_cast<int>((ptr - ptr0) - p.y*step);
		return p;
	}

	namespace tests {

		void line_iterator_2_tests();

	}


}



#endif /* LINE_ITERATOR_2_H_ */
