/**
 * serialization methods for necessary structures
 */
#pragma once

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/string.hpp>
#include <iostream>
#include <fstream>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/bitset.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/optional.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/nvp.hpp>

namespace boost {
namespace serialization {

// Rect (opencv)
template<class Archive>
void serialize(Archive &ar, cv::Rect_<int> &rec, const unsigned int) {
	ar & BOOST_SERIALIZATION_NVP(rec.height);
	ar & BOOST_SERIALIZATION_NVP(rec.width);
	ar & BOOST_SERIALIZATION_NVP(rec.x);
	ar & BOOST_SERIALIZATION_NVP(rec.y);
}

// Point2i (opencv)
template<class Archive>
void serialize(Archive &ar, cv::Point2i &point, const unsigned int) {
	ar & BOOST_SERIALIZATION_NVP(point.x);
	ar & BOOST_SERIALIZATION_NVP(point.y);
}


// Size (opencv)
template<class Archive>
void serialize(Archive &ar, cv::Size_<int>& size, const unsigned int) {
	ar & BOOST_SERIALIZATION_NVP(size.width);
	ar & BOOST_SERIALIZATION_NVP(size.height);
}

}
}
