/*
 * Tag.cpp
 *
 *  Created on: 17.08.2014
 *      Author: mareikeziese
 */

#include "Tag.h"

namespace decoder {

/**************************************
 *
 * 			constructor
 *
 **************************************/

Tag::Tag() {
	this->_valid = true;
	this->_candidates = vector<TagCandidate>();

}


Tag::Tag(Rect rec) {
	this->_valid = true;
	this->_candidates = vector<TagCandidate>();
	BoundingBox bb = BoundingBox(rec);
	this->_boundingBox = bb;

}


Tag::~Tag() {
	// TODO Auto-generated destructor stub
}

/**************************************
 *
 * 			getter/setter
 *
 **************************************/

const BoundingBox& Tag::getBoundingBox() const {
	return _boundingBox;
}

void Tag::setBoundingBox(const BoundingBox& boundingBox) {
	_boundingBox = boundingBox;
}

const vector<TagCandidate>& Tag::getCandidates() const {
	return _candidates;
}

void Tag::setCandidates(const vector<TagCandidate>& candidates) {
	_candidates = candidates;
}

const Mat& Tag::getCannySubImage() const {
	return _cannySubImage;
}

void Tag::setCannySubImage(const Mat& cannySubImage) {
	_cannySubImage = cannySubImage;
}

const Mat& Tag::getOrigSubImage() const {
	return _origSubImage;
}

void Tag::setOrigSubImage(const Mat& origSubImage) {
	_origSubImage = origSubImage;
}

bool Tag::isValid() const {
	return _valid;
}

void Tag::setValid(bool valid) {
	_valid = valid;
}

int Tag::getId() const {
	return id;
}

void Tag::setId(int id) {
	this->id = id;
}

/**************************************
 *
 * 			stuff
 *
 **************************************/

template<class Archive>
void Tag::serialize(Archive & ar, const unsigned int version) {
	ar & this->id & this->_valid;
}

void Tag::addCandidate(TagCandidate c){
	this->_candidates.push_back(c);
}

} /* namespace decoder */
