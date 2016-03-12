#pragma once

#include <vector>                // std::vector

#include <opencv2/core/core.hpp> // cv::Mat, cv::Rect

#include "PreprocessorResults.h"
#include "TagCandidate.h"
#include "serialization.hpp"

namespace pipeline {
class Tag {
public:
    explicit Tag(cv::Rect roi, int id);
    explicit Tag(cv::Rect roi, int id, PreprocessorResult const& preprocessorResult);

    typedef struct {
        cv::Rect roi;
        cv::Mat orig;
        cv::Mat edges;
        cv::Mat clahe;
    } Representations;

    std::vector<TagCandidate> &getCandidates();
    const std::vector<TagCandidate> &getCandidatesConst() const;
    void setCandidates(std::vector<TagCandidate>&& candidates);
    void addCandidate(TagCandidate c);

    Representations const& getRepresentations() const;
    void setRoi(const cv::Rect& box);
    void setOrigSubImage(const cv::Mat& origSubImage);
    void setEdgeSubImage(const cv::Mat& cannySubImage);
    void setClaheSubImage(const cv::Mat& claheSubImage);

    bool isValid() const;
    void setValid(bool valid);

    int getId() const;
    void setId(int _id);

    double getLocalizerScore() const;
    void setLocalizerScore(const double score);

private:
    Representations _representations;

    // marks if the tag is really a tag;
    bool _valid;

    // virtual id, just necessary for the decoding process;
    int _id;

    double _localizerScore;

    //there may be multiple ellipses and grids for this location, so there is a list of candidates
    std::vector<TagCandidate> _candidates;

    //needed to serialize all the private members
    friend class boost::serialization::access;

    // needed to serialize class implicit
    template<class Archive>
    void serialize(Archive & ar, const unsigned int) {
        ar & BOOST_SERIALIZATION_NVP(_representations);
        ar & BOOST_SERIALIZATION_NVP(_id);
        ar & BOOST_SERIALIZATION_NVP(_valid);
        ar & BOOST_SERIALIZATION_NVP(_candidates);
    }
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
