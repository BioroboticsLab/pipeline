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
*           constructor
*
**************************************/

Tag::Tag() {
    this->_valid      = true;
    this->_candidates = vector<TagCandidate>();
}

Tag::Tag(Rect rec) {
    this->_valid      = true;
    this->_candidates = vector<TagCandidate>();

    this->_box = rec;
}

Tag::~Tag() {
    // TODO Auto-generated destructor stub
}

/**************************************
*
*           getter/setter
*
**************************************/

vector<TagCandidate>& Tag::getCandidates() {
    return _candidates;
}

void Tag::setCandidates(vector<TagCandidate>&& candidates) {
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

const Rect& Tag::getBox() const {
    return _box;
}

void Tag::setBox(const Rect& box) {
    _box = box;
}

/**************************************
*
*           stuff
*
**************************************/

#ifdef PipelineStandalone
template<class Archive>
void Tag::serialize(Archive & ar, const unsigned int version) {
    ar & this->id & this->_valid;
}
#endif

void Tag::addCandidate(TagCandidate c){
    this->_candidates.push_back(c);
}
} /* namespace decoder */
