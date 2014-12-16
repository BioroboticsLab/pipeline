/*
 * BoundingBox.h
 *
 *  Created on: 31.07.2014
 *      Author: mareikeziese
 */

#ifndef BOUNDINGBOX_H_
#define BOUNDINGBOX_H_

#include <opencv2/core/core.hpp> // cv::Rect, cv::Point

#ifdef PipelineStandalone
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#endif


namespace decoder {
class BoundingBox {
private:

    /**************************************
    *
    *           members
    *
    **************************************/

    cv::Rect _box;

#ifdef PipelineStandalone
    //needed to serialize all the private members
    friend class boost::serialization::access;
#endif

    /**************************************
    *
    *           stuff
    *
    **************************************/

public:

    /**************************************
    *
    *           constructor
    *
    **************************************/

    BoundingBox() {
    }

    BoundingBox(cv::Rect bb) {
        this->_box = bb;
    }

    /**************************************
    *
    *           getter/setter
    *
    **************************************/

    const cv::Rect& getBox() const {
        return _box;
    }

    void setBox(const cv::Rect& box) {
        _box = box;
    }

    /**************************************
    *
    *           stuff
    *
    **************************************/

    bool isPossibleCenter(cv::Point p, int tolerance) {
        int centerx = this->_box.x + this->_box.width / 2;
        int centery = this->_box.y + this->_box.height / 2;
        return (p.x >= (centerx - tolerance) && p.x <= (centerx + tolerance)
               && p.y >= centery - tolerance && p.y <= (centery + tolerance));
    }

#ifdef PipelineStandalone
    /**
     * serialization via boost
     */
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version){
        ar & this->_box.x & this->_box.y & this->_box.width & this->_box.height;
    }
#endif
};
} /* namespace decoder */

#endif /* BOUNDINGBOX_H_ */
