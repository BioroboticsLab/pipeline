/*
 * helper.impl.h
 *
 *  Created on: Mar 2, 2015
 *      Author: tobias
 */

#ifndef HELPER_IMPL_H_
#define HELPER_IMPL_H_

namespace cv {
    void scalarToRawData(const Scalar& s, void* _buf, int type, int unroll_to);
}

namespace heyho {

	namespace detail {

		template<typename T, typename = typename std::enable_if<is_primitive_open_cv_data_type<T>::value>::type>
		inline T saturate(double d) {
			return cv::saturate_cast<T>(d);
		}

		/**
		 * default open cv implementation using cv::scalarToRawData
		 */
		template<typename pixel_t, bool = is_primitive_open_cv_data_type<pixel_t>::value>
		struct scalar2pixel
		{
			pixel_t operator()(const cv::Scalar &color) const {
				constexpr int img_type = cv::DataType<pixel_t>::type;

				// values from cv::scalarToRawData implementation
				constexpr size_t max_num_chans = 4;              // CV_Assert(cn <= 4);
				constexpr size_t max_chan_size = sizeof(double); // CV_64F

				uchar buf[max_num_chans * max_chan_size];
				cv::scalarToRawData(color, buf, img_type, 0);

				pixel_t result;
				static_assert(sizeof result <= sizeof buf, "invalid pixel_t");
				std::copy_n(buf, sizeof result, reinterpret_cast<uchar*>(&result));
				return result;
			}
		};

		/**
		 * primitive open cv data type implementation (e.g. char, int, float, ...)
		 */
		template<typename pixel_t>
		struct scalar2pixel<pixel_t, true>
		{
			pixel_t operator()(const cv::Scalar &color) const {
				return saturate<pixel_t>(color[0]);
			}
		};

		/**
		 * cv::Vec implementation
		 */
		template<typename pixel_t, int num_chans>
		struct scalar2pixel<cv::Vec<pixel_t, num_chans>, false>
		{
			static_assert(0 < num_chans && num_chans <= 4, "");

			cv::Vec<pixel_t, num_chans> operator()(const cv::Scalar &color) const {
				cv::Vec<pixel_t, num_chans> result;
				for (int i = 0; i < num_chans; ++i) {
					result[i] = saturate<pixel_t>(color[i]);
				}
				return result;
			}
		};

		/**
		 * cv::Matx implementation
		 */
		template<typename pixel_t, int dy, int dx>
		struct scalar2pixel<cv::Matx<pixel_t, dy, dx>, false>
		{
			static_assert(dx > 0 && dy > 0 && dx * dy <= 4, "");

			cv::Matx<pixel_t, dy, dx> operator()(const cv::Scalar &color) const {
				cv::Matx<pixel_t, dy, dx> result;
				for (int i = 0; i < dx * dy; ++i) {
					result(i) = saturate<pixel_t>(color[i]);
				}
				return result;
			}
		};

		/**
		 * cv::Point_ implementation
		 */
		template<typename pixel_t>
		struct scalar2pixel<cv::Point_<pixel_t>, false>
		{
			cv::Point_<pixel_t> operator()(const cv::Scalar &color) const {
				return {
					saturate<pixel_t>(color[0]),
					saturate<pixel_t>(color[1])
				};
			}
		};

		/**
		 * cv::Point3_ implementation
		 */
		template<typename pixel_t>
		struct scalar2pixel<cv::Point3_<pixel_t>, false>
		{
			cv::Point3_<pixel_t> operator()(const cv::Scalar &color) const {
				return {
					saturate<pixel_t>(color[0]),
					saturate<pixel_t>(color[1]),
					saturate<pixel_t>(color[2])
				};
			}
		};

	}

	template<typename pixel_t>
	inline pixel_t scalar2pixel(const cv::Scalar &color) {
		return detail::scalar2pixel<pixel_t>{}(color);
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

		constexpr int pix_size = sizeof(typename cv::DataType<pixel_t>::value_type);
		if (pix_size != static_cast<int>(img.elemSize())) {
			throw std::invalid_argument("invalid image pixel size");
		}
	}

	template<typename pixel_t>
	inline pixel_setter<pixel_t>::pixel_setter(cv::Mat &img, const cv::Scalar &color)
		: pixel_setter(img, scalar2pixel<pixel_t>(color))
	{}

	template<typename pixel_t>
	inline void pixel_setter<pixel_t>::operator()(cv::Point p) {
		m_img.get().template at<pixel_t>(p) = m_color;
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
		if (m_img.get().template at<pixel_t>(p) == m_zero) {
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

	inline std::pair<const cv::Point*, int> cv_point_input_array_to_pointer(cv::InputArray pts) {
		const cv::Mat points = pts.getMat();
		if (! (points.checkVector(2, CV_32S, true) >= 0)) {
			throw std::invalid_argument("InputArray doesn't point to continuous cv::Point data");
		}
		const cv::Point *begin = reinterpret_cast<const cv::Point*>(points.data);
		const int size = points.rows * points.cols * points.channels() / 2;
		return {begin, size};
	}

	namespace detail {

		template<typename, typename ...>
		struct contains : std::false_type {};

		template<typename T, typename ...TS>
		struct contains<T, T, TS...> : std::true_type {};

		template<typename T, typename U, typename ... TS>
		struct contains<T, U, TS...> : contains<T, TS...> {};

	}

	template<typename T>
	struct is_primitive_open_cv_data_type : detail::contains<T,
		char,
		unsigned char,
		signed char,
		unsigned short,
		short,
		int,
		float,
		double>
	{};

}

#endif /* HELPER_IMPL_H_ */
