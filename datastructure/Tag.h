/*
 * Tag.h
 *
 *  Created on: 17.08.2014
 *      Author: mareikeziese
 */

#ifndef TAG_H_
#define TAG_H_

#include "TagCandidate.h"        // TagCandidate
#include "opencv2/core/core.hpp" // cv::Mat, cv::Rect
#include <vector>                // std::vector

#ifdef PipelineStandalone
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>

using namespace boost;
#endif


namespace decoder {
class Tag {
private:

    /**************************************
    *
    *           members
    *
    **************************************/

    cv::Rect _box;

    cv::Mat _origSubImage;

    cv::Mat _cannySubImage;

    //marks if the tag is really a tag;
    bool _valid;

    //virtual id, just necessary for the decoding process;
    int id;

    //there may be multiple ellipses and grids for this location, so there is a list of candidates
    std::vector<TagCandidate> _candidates;

#ifdef PipelineStandalone
    //needed to serialize all the private members
    friend class boost::serialization::access;

    //needed to serialize class implicit
    template<class Archive> void serialize(Archive & ar,
      const unsigned int version);
#endif

public:

    explicit Tag(cv::Rect rec, int id);
    ~Tag();

    /**************************************
    *
    *           getter/setter
    *
    **************************************/

    std::vector<TagCandidate> &getCandidates();
	const std::vector<TagCandidate> &getCandidates() const;
    void setCandidates(std::vector<TagCandidate>&& candidates);
    const cv::Mat& getCannySubImage() const;
    void setCannySubImage(const cv::Mat& cannySubImage);
    const cv::Mat& getOrigSubImage() const;
    void setOrigSubImage(const cv::Mat& origSubImage);
    bool isValid() const;
    void setValid(bool valid);
    int getId() const;
    void setId(int id);
    void addCandidate(TagCandidate c);
    const cv::Rect& getBox() const;
    void setBox(const cv::Rect& box);
};
} /* namespace decoder */

// needed to be included for the function template
//#include "Tag.cpp"
#endif /* TAG_H_ */
