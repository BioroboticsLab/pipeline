/*
 * helper.impl.h
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

	template<typename pixel_t>
	size_t pixel_counter<pixel_t>::pixel_count::zero() const {
		return m_zero;
	}

	template<typename pixel_t>
	size_t pixel_counter<pixel_t>::pixel_count::non_zero() const {
		return m_non_zero;
	}

	template<typename pixel_t>
	size_t pixel_counter<pixel_t>::pixel_count::all() const {
		return this->zero() + this->non_zero();
	}

	template<typename pixel_t>
	pixel_counter<pixel_t>::pixel_counter(const cv::Mat &img, const pixel_t &zero)
		: m_img(img)
		, m_zero(zero)
		, m_count{0,0}
	{}

	template<typename pixel_t>
	pixel_counter<pixel_t>::pixel_counter(const cv::Mat &img, const cv::Scalar &zero)
		: pixel_counter(img, scalar2pixel<pixel_t>(zero))
	{}

	template<typename pixel_t>
	void pixel_counter<pixel_t>::operator()(cv::Point p) {
		if (m_img.get().at<pixel_t>(p) == m_zero) {
			++m_count.m_zero;
		}
		else {
			++m_count.m_non_zero;
		}
	}

	template<typename pixel_t>
	const typename pixel_counter<pixel_t>::pixel_count& pixel_counter<pixel_t>::count() const {
		return m_count;
	}

}

#endif /* HELPER_IMPL_H_ */
