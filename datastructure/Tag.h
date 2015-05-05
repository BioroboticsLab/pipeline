#pragma once

#include <vector>                // std::vector

#include <opencv2/core/core.hpp> // cv::Mat, cv::Rect

#include "TagCandidate.h"
#include "serialization.hpp"

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

	//needed to serialize all the private members
	friend class boost::serialization::access;

	//needed to serialize class implicit
	template<class Archive>
	void serialize(Archive & ar, const unsigned int) {
	    ar & BOOST_SERIALIZATION_NVP(_box);
	    ar & BOOST_SERIALIZATION_NVP(_id);
	    ar & BOOST_SERIALIZATION_NVP(_valid);
	    ar & BOOST_SERIALIZATION_NVP(_candidates);
	}

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

BOOST_CLASS_EXPORT_KEY(pipeline::Tag)


namespace boost { namespace serialization {

template<class Archive>
inline void load_construct_data(Archive &, pipeline::Tag * t,  unsigned int) {
    // retrieve data from archive required to construct new instance
/*	cv::Rect box;
	int id;
    ar >> id;
    ar >> box;*/
    // invoke inplace constructor to initialize instance of PipelineGrid

	/**
	 * @TODO fix ME!!
	 */
    ::new(t)pipeline::Tag(cv::Rect(0,0,0,0),0);
}
}}
