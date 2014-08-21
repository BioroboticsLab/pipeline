/*
 * Candidate.h
 *
 *  Created on: 18.08.2014
 *      Author: mareikeziese
 */

#ifndef CANDIDATE_H_
#define CANDIDATE_H_



#include "Grid.h"
#include "Ellipse.h"


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>



using namespace cv;

namespace decoder {

class TagCandidate {
private:
	/**************************************
	 *
	 * 			members
	 *
	 **************************************/
	//score of the candidate
	double _score;
	Ellipse _ellipse;
	Grid _grid;
	//decoded Id
	int _decodedId;
	Mat _transformedImage;
public:
	/**************************************
	 *
	 * 			constructor
	 *
	 **************************************/
	TagCandidate(){

	}
	TagCandidate(Ellipse e){
		this->_ellipse = e;
	}
	 ~TagCandidate(){

	 }


	/**************************************
	 *
	 * 			getter/setter
	 *
	 **************************************/

	int getDecodedId() const {
		return _decodedId;
	}

	void setDecodedId(int decodedId) {
		_decodedId = decodedId;
	}

	const Ellipse& getEllipse() const {
		return _ellipse;
	}

	void setEllipse(const Ellipse& ellipse) {
		_ellipse = ellipse;
	}

	const Grid& getGrid() const {
		return _grid;
	}

	void setGrid(const Grid& grid) {
		_grid = grid;
	}

	double getScore() const {
		return _score;
	}

	void setScore(double score) {
		_score = score;
	}

	const Mat& getTransformedImage() const {
		return _transformedImage;
	}

	void setTransformedImage(const Mat& transformedImage) {
		_transformedImage = transformedImage;
	}
};

} /* namespace decoder */

#endif /* CANDIDATE_H_ */
