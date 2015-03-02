/*
 * helper_impl.h
 *
 *  Created on: Mar 2, 2015
 *      Author: tobias
 */

#ifndef HELPER_IMPL_H_
#define HELPER_IMPL_H_

namespace heyho {

	template<typename pixel_t>
	inline pixel_t scalar2pixel(const cv::Scalar &color) {
		constexpr int img_type = cv::DataType<pixel_t>::type;
		double buf[4];
		cv::scalarToRawData(color, buf, img_type, 0);
		pixel_t result;
		uchar *pixel_p = reinterpret_cast<uchar*>(&result);
		for (size_t i = 0; i < sizeof result; ++i) {
			pixel_p[i] = reinterpret_cast<const uchar*>(buf)[i];
		}
		return result;
	}


	template<typename pixel_t>
	inline pixel_setter<pixel_t>::pixel_setter(cv::Mat &img, const pixel_t &color)
		: m_img(img)
		, m_color(color)
	{
		constexpr int img_type = cv::DataType<pixel_t>::type;
		if (img.type() != img_type) {
			throw std::invalid_argument("invalid image pixel type");
		}
	}

	template<typename pixel_t>
	inline pixel_setter<pixel_t>::pixel_setter(cv::Mat &img, const cv::Scalar &color)
		: pixel_setter(img, scalar2pixel<pixel_t>(color))
	{}

	template<typename pixel_t>
	inline void pixel_setter<pixel_t>::operator()(cv::Point p) {
		m_img.get().at<pixel_t>(p) = m_color;
	}


}

#endif /* HELPER_IMPL_H_ */
