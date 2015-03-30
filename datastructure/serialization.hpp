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
#include <boost/serialization/map.hpp>
#include <boost/serialization/optional.hpp>
#include <boost/serialization/split_free.hpp>





namespace boost {
namespace serialization {



// Rect (opencv)
template<class Archive>
void serialize(Archive &ar, cv::Rect_<int> &rec, const unsigned int version) {
	ar & rec.height;
	ar & rec.width;
	ar & rec.x;
	ar & rec.y;
}

// Point2i (opencv)
template<class Archive>
void serialize(Archive &ar, cv::Point2i &point, const unsigned int version) {
	ar & point.x;
	ar & point.y;
}

// Size (opencv)
template<class Archive>
void serialize(Archive &ar, cv::Size_<int>& size, const unsigned int version) {
	ar & size.width;
	ar & size.height;
}


}
}
