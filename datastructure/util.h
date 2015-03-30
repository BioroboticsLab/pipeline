#pragma once

#include <cstdint>
#include <vector>

namespace util {
// branchless, type-safe signum
// see: http://stackoverflow.com/a/4609795
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

//template <typename T>
//constexpr auto linspace(T start, T stop, T step) {
//    const size_t num = static_cast<size_t>((stop - start) / step) + 1;
//    std::vector<T> values(num);
//    size_t idx = 0;
//    for (T val = start; val <= stop; val += step) {
//        values[idx] = val;
//        ++idx;
//    }
//    return values;
//}

template <typename T>
std::vector<T> linspace(T first, T last, size_t len) {
    std::vector<T> result(len);
    T step = (last-first) / (len - 1);
    for (size_t i=0; i<len; i++) { result[i] = first + i*step; }
    return result;
}

//template<typename T, size_t N>
//std::ostream& operator<<(std::ostream &os, const std::array<T, N> &a) {
//	os << '[';
//	if (a.size()) {
//		os << a[0];
//		for (size_t i = 1; i < a.size(); ++i) {
//			os << ", " << a[i];
//		}
//	}
//	return os << ']';
//}


//namespace detail {

//	template<size_t ... SEQ>
//	struct sequence {};


//	/**
//	 * generates "sequence<0, ..., LENGTH - 1>{};"
//	 */
//	template<size_t LENGTH, size_t ... TMP_SEQ>
//	struct generate_sequence : generate_sequence<LENGTH - 1, LENGTH - 1, TMP_SEQ ...> {};
//	/**
//	 * base case
//	 */
//	template<size_t ... TMP_SEQ>
//	struct generate_sequence<0, TMP_SEQ ...> : sequence<TMP_SEQ ...> {};



//	template<typename ftype, size_t ... SEQ>
//	constexpr std::array<ftype, sizeof ...(SEQ)> linspace_helper(sequence<SEQ...>, ftype offset, ftype step) {
//		return {(offset + SEQ * step)...};
//	}
//}

///**
// * Return evenly spaced numbers over a specified interval.
// *
// * Returns NUM evenly spaced samples, calculated over the interval [start, stop ].
// * The endpoint of the interval can optionally be excluded.
// *
// * @tparam NUM     Number of samples to generate.
// * @param start    The starting value of the sequence.
// * @param stop     The end value of the sequence, unless endpoint is set to False.
// *                 In that case, the sequence consists of all but the last of NUM + 1 evenly spaced samples,
// *                 so that stop is excluded. Note that the step size changes when endpoint is False.
// * @param endpoint If True, stop is the last sample. Otherwise, it is not included. Default is True.
// */
//template<size_t NUM, typename ftype>
//constexpr std::array<ftype, NUM> linspace(ftype start, ftype stop, bool endpoint = true) {
//    return detail::linspace_helper(detail::generate_sequence<NUM>{}, start, (stop - start) / static_cast<ftype>(NUM - endpoint));
//}
}
