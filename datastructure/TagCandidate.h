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
#include "Decoding.h"


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
	vector<Grid> _grids;
	vector <Decoding> _decodings;
	//decoded Id
	int _decodedId;
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



	double getScore() const {
		return _score;
	}

	void setScore(double score) {
		_score = score;
	}

	const Mat& getTransformedImage() const {
        return _ellipse.getTransformedImage();
	}

	void setTransformedImage(const Mat& transformedImage) {
        _ellipse.setTransformedImage(transformedImage);
	}

	const vector<Grid>& getGrids() const {
		return _grids;
	}

	void setGrids(const vector<Grid>& grids) {
		_grids = grids;
	}

	const vector<Decoding>& getDecodings() const {
		return _decodings;
	}

	void setDecodings(const vector<Decoding>& decodings) {
		_decodings = decodings;
	}
};

} /* namespace decoder */

#endif /* CANDIDATE_H_ */
