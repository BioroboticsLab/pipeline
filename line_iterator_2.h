/*
 * line_iterator_2.h
 *
 *  Created on: Mar 6, 2015
 *      Author: tobias
 */

#ifndef LINE_ITERATOR_2_H_
#define LINE_ITERATOR_2_H_

#include <opencv2/opencv.hpp>

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
		int elemSize;
		int err, count;
		int minusDelta, plusDelta;
		int minusStep, plusStep;

		static constexpr int img_elemSize = 1;
	};

	inline line_iterator_2::line_iterator_2(cv::Size size, cv::Point pt1, cv::Point pt2,
	                                 int connectivity, bool left_to_right)
		: ptr0(nullptr)
		, step(size.width)
	{
		count = -1;

		CV_Assert( connectivity == 8 || connectivity == 4 );

		int bt_pix0 = img_elemSize;
		int bt_pix = bt_pix0;
		size_t istep = static_cast<size_t>(step);

		int dx = pt2.x - pt1.x;
		int dy = pt2.y - pt1.y;
		int s = dx < 0 ? -1 : 0;

		if( left_to_right )
		{
			dx = (dx ^ s) - s;
			dy = (dy ^ s) - s;
			pt1.x ^= (pt1.x ^ pt2.x) & s;
			pt1.y ^= (pt1.y ^ pt2.y) & s;
		}
		else
		{
			dx = (dx ^ s) - s;
			bt_pix = (bt_pix ^ s) - s;
		}

		ptr = static_cast<uchar*>(static_cast<uchar*>(nullptr) + pt1.y * istep + pt1.x * bt_pix0);

		s = dy < 0 ? -1 : 0;
		dy = (dy ^ s) - s;
		istep = (istep ^ s) - s;

		s = dy > dx ? -1 : 0;

		/* conditional swaps */
		dx ^= dy & s;
		dy ^= dx & s;
		dx ^= dy & s;

		bt_pix ^= static_cast<int>(istep) & s;
		istep ^= bt_pix & s;
		bt_pix ^= static_cast<int>(istep) & s;

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

		this->elemSize = bt_pix0;
	}

	inline line_iterator_2& line_iterator_2::operator ++()
	{
		int mask = err < 0 ? -1 : 0;
		err += minusDelta + (plusDelta & mask);
		ptr += minusStep + (plusStep & mask);
		return *this;
	}

	inline cv::Point line_iterator_2::pos() const
	{
		cv::Point p;
		p.y = static_cast<int>((ptr - ptr0)/step);
		p.x = static_cast<int>(((ptr - ptr0) - p.y*step)/elemSize);
		return p;
	}

	namespace tests {

		void line_iterator_2_tests();

	}


}



#endif /* LINE_ITERATOR_2_H_ */
