/*
 * Tag.cpp
 *
 *  Created on: 17.08.2014
 *      Author: mareikeziese
 */

#include "Tag.h"
#include <utility> // std::move

namespace pipeline {
/**************************************
*
*           constructor
*
**************************************/

Tag::Tag(cv::Rect rec, int id)
    : _box(rec)
    , _valid(true)
    , _id(id)
{
}

/**************************************
*
*           getter/setter
*
**************************************/

std::vector<TagCandidate>& Tag::getCandidates() {
	return _candidates;
}

const std::vector<TagCandidate>& Tag::getCandidatesConst() const {
	return _candidates;
}

void Tag::setCandidates(std::vector<TagCandidate>&& candidates) {
	_candidates = std::move(candidates);
}

const cv::Mat& Tag::getCannySubImage() const {
	return _cannySubImage;
}

void Tag::setCannySubImage(const cv::Mat& cannySubImage) {
	_cannySubImage = cannySubImage;
}

const cv::Mat& Tag::getOrigSubImage() const {
	return _origSubImage;
}

void Tag::setOrigSubImage(const cv::Mat& origSubImage) {
	_origSubImage = origSubImage;
}

bool Tag::isValid() const {
	return _valid;
}

void Tag::setValid(bool valid) {
	_valid = valid;
}

int Tag::getId() const {
	return _id;
}

void Tag::setId(int id) {
	this->_id = id;
}

const cv::Rect& Tag::getBox() const {
	return _box;
}

void Tag::setBox(const cv::Rect& box) {
	_box = box;
}

/**************************************
*
*           stuff
*
**************************************/

#ifdef PipelineStandalone
<<<<<<< HEAD
template<class Archive>
void Tag::serialize(Archive & ar, const unsigned int version) {
	ar & this->id & this->_valid;
}
=======
// TODO: FIXME!
/*template<class Archive>
void Tag::serialize(Archive & ar, const unsigned int version) {
    ar & this->_id;
    ar & this->_valid;
    ar & this->_;
    ar & this->_valid;
}*/
>>>>>>> 59045fe... things..
#endif

void Tag::addCandidate(TagCandidate c){
	this->_candidates.push_back(c);
}

bool operator<(const Tag &lhs, const Tag &rhs)
{
	return lhs.getId() < rhs.getId();
}

} /* namespace decoder */
