#include "../../datastructure//Tag.h"

#include <utility> // std::move

namespace pipeline {

Tag::Tag(cv::Rect roi, int id)
    : _valid(true)
    , _id(id)
{
    _representations.roi = roi;
}

Tag::Tag(cv::Rect roi, int id, const PreprocessorResult &preprocessorResult)
    : _valid(true)
    , _id(id)
{
    cv::Mat origSubImage(preprocessorResult.originalImage, roi);
    _representations.orig = origSubImage.clone();
    cv::Mat edgeSubImage(preprocessorResult.preprocessedImage, roi);
    _representations.edges = edgeSubImage.clone();
    cv::Mat claheSubImage(preprocessorResult.claheImage, roi);
    _representations.clahe = claheSubImage.clone();
}

std::vector<TagCandidate>& Tag::getCandidates() {
	return _candidates;
}

const std::vector<TagCandidate>& Tag::getCandidatesConst() const {
	return _candidates;
}

void Tag::setCandidates(std::vector<TagCandidate>&& candidates) {
	_candidates = std::move(candidates);
}

void Tag::setEdgeSubImage(const cv::Mat& edgeSubImage) {
    _representations.edges = edgeSubImage;
}

void Tag::setClaheSubImage(const cv::Mat &claheSubImage)
{
    _representations.clahe = claheSubImage;
}

void Tag::setOrigSubImage(const cv::Mat& origSubImage) {
    _representations.orig = origSubImage;
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
    _id = id;
}

void Tag::setRoi(const cv::Rect& box) {
    _representations.roi = box;
}

void Tag::addCandidate(TagCandidate c){
    _candidates.push_back(c);
}

const Tag::Representations &Tag::getRepresentations() const
{
    return _representations;
}

bool operator<(const Tag &lhs, const Tag &rhs)
{
    return lhs.getId() < rhs.getId();
}

double Tag::getLocalizerScore() const
{
    return _localizerScore;
}

void Tag::setLocalizerScore(const double score)
{
    _localizerScore = score;

}
} /* namespace decoder */
