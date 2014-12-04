/*
 * Tag.cpp
 *
 *  Created on: 17.08.2014
 *      Author: mareikeziese
 */

#include "Tag.h"
#include <utility> // std::move

namespace decoder {
/**************************************
*
*           constructor
*
**************************************/

Tag::Tag(cv::Rect rec, int id)
	: _box(rec)
	, _valid(true)
	, id(id)
{
}

Tag::~Tag() = default;

/**************************************
*
*           getter/setter
*
**************************************/

std::vector<TagCandidate>& Tag::getCandidates() {
    return _candidates;
}

const std::vector<TagCandidate>& Tag::getCandidates() const {
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
    return id;
}

void Tag::setId(int id) {
    this->id = id;
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
template<class Archive>
void Tag::serialize(Archive & ar, const unsigned int version) {
    ar & this->id & this->_valid;
}
#endif

void Tag::addCandidate(TagCandidate c){
    this->_candidates.push_back(c);
}
} /* namespace decoder */
