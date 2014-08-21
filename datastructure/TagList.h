/*
 * TagList.h
 *
 *  Created on: 18.08.2014
 *      Author: mareikeziese
 */

#ifndef TAGLIST_H_
#define TAGLIST_H_

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <fstream>
#include <boost/serialization/vector.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include "Tag.h"

using namespace cv;
using namespace std;
using namespace boost;

namespace decoder {
class TagList {
private:
	std::vector<Tag> _tags;
	friend class boost::serialization::access;

	template<class Archive>
	void serialize(Archive & ar, const unsigned int version);

public:
	TagList();
	virtual ~TagList();
	void AddTag(Tag t);
	int size();
	Tag getTag(int position);
	void removeTag(int position);


};
}
// needed to be included for the function template
//#include "TagList.cpp"

#endif /* TAGLIST_H_ */
