/*
 * BBList.cpp
 *
 *  Created on: 31.07.2014
 *      Author: mareikeziese
 */

#include "BBList.h"

namespace boost {
namespace serialization {

//needed to serialize Points
template<class Archive>
void serialize(Archive & ar, cv::Point & p, const unsigned int version) {
	ar & p.x;
	ar & p.y;

}

} // namespace serialization
} // namespace boost

namespace decoder {
BBList::BBList() {
	// TODO Auto-generated constructor stub

}

BBList::~BBList() {
	// TODO Auto-generated destructor stub
}

template<class Archive>
void BBList::serialize(Archive & ar, const unsigned int version) {
	ar & points;
}

void BBList::serialize(boost::archive::text_oarchive& ar,
		const unsigned int version) {
	ar & points;
}

void BBList::serialize(boost::archive::text_iarchive& ar,
		const unsigned int version) {
	ar & points;
}
void BBList::AddPoint(Point p) {
	points.push_back(p);
}

int BBList::size() {
	return points.size();
}

Point BBList::getPoint(int position) {
	return points[position];
}
}
