/*
 * TagList.cpp
 *
 *  Created on: 18.08.2014
 *      Author: mareikeziese
 */

#include "TagList.h"

namespace decoder {


/**************************************
 *
 * 			constructor
 *
 **************************************/

TagList::TagList() {
	this->_tags = vector <Tag>();

}

TagList::~TagList() {
	// TODO Auto-generated destructor stub
}

/**************************************
 *
 * 			stuff
 *
 **************************************/


template<class Archive>
void TagList::serialize(Archive & ar, const unsigned int version) {
	ar & this->_tags;
}

void TagList::addTag(Tag t) {
	this->_tags.push_back(t);
}

int TagList::size() {
	return this->_tags.size();
}

Tag TagList::getTag(int position) {
	return this->_tags[position];
}

void TagList::removeTag(int position){
	this->_tags.erase(this->_tags.begin() + position);
}

} /* namespace decoder */
