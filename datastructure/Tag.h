#pragma once

#include <vector>                // std::vector

#include <opencv2/core/core.hpp> // cv::Mat, cv::Rect

#include "TagCandidate.h"

#ifdef PipelineStandalone
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>

using namespace boost;
#endif

namespace pipeline {
class Tag {
private:
	cv::Rect _box;
	cv::Mat _origSubImage;
	cv::Mat _cannySubImage;

	//marks if the tag is really a tag;
	bool _valid;

	//virtual id, just necessary for the decoding process;
	int _id;

	//there may be multiple ellipses and grids for this location, so there is a list of candidates
	std::vector<TagCandidate> _candidates;

#ifdef PipelineStandalone
	//needed to serialize all the private members
	friend class boost::serialization::access;

	//needed to serialize class implicit
	template<class Archive> void serialize(Archive & ar, const unsigned int version);
#endif

public:
	explicit Tag(cv::Rect rec, int _id);

	std::vector<TagCandidate> &getCandidates();
	const std::vector<TagCandidate> &getCandidatesConst() const;
	void setCandidates(std::vector<TagCandidate>&& candidates);
	void addCandidate(TagCandidate c);

	const cv::Mat& getCannySubImage() const;
	void setCannySubImage(const cv::Mat& cannySubImage);

	const cv::Mat& getOrigSubImage() const;
	void setOrigSubImage(const cv::Mat& origSubImage);

	bool isValid() const;
	void setValid(bool valid);

	int getId() const;
	void setId(int _id);

	const cv::Rect& getBox() const;
	void setBox(const cv::Rect& box);
};

bool operator<(const Tag& lhs, const Tag& rhs);
}
