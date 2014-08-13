/*
 * Decoder.h
 *
 *  Created on: 28.07.2014
 *      Author: mareikeziese
 */

#ifndef BBLIST_H
#define BBLIST_H

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <fstream>
#include <boost/serialization/vector.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;
using namespace boost;

namespace decoder {
class BBList {
private:
	std::vector<Point> points;
	friend class boost::serialization::access;
	// Serialize the std::vector member of Info
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);
	void serialize(boost::archive::text_oarchive& ar, const unsigned int version);
	void serialize(boost::archive::text_iarchive& ar, unsigned int version);

public:
	BBList();
	virtual ~BBList();
	void AddPoint(Point p);
	//void Print() const;
	int size();
	Point getPoint(int position);

};
}

#endif /* BBLIST_H */
