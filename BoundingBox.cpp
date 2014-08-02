/*
 * BoundingBox.cpp
 *
 *  Created on: 31.07.2014
 *      Author: mareikeziese
 */

#include "BoundingBox.h"

namespace decoder {

BoundingBox::BoundingBox() {
	// TODO Auto-generated constructor stub

}

BoundingBox::~BoundingBox() {
	// TODO Auto-generated destructor stub
}
bool BoundingBox::isPossibleCenter(Point p, int tolerance){

	int centerx = this->box_.x + this->box_.width/2;
	int centery = this->box_.y + this->box_.height/2;
	return(p.x >= (centerx- tolerance)   &&
			p.x <= (centerx +tolerance) &&
			p.y >= centery - tolerance &&
			p.y <= (centery + tolerance));
}

} /* namespace decoder */
