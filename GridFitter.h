/*
 * gridFitter.h
 *
 *  Created on: 05.05.2014
 *	  Author: mareikeziese
 */

#ifndef GRIDFITTER_H_
#define GRIDFITTER_H_

#include "Ellipse.h"
#include "Grid.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Constants for optimized detection of the tag center
// initial step size
#define INITIAL_STEP_SIZE 20
// step size the fitting-algorithm should terminate
#define FINAL_STEP_SIZE 0
// growth speed for the step size (on position change)
#define UP_SPEED 3
// speed decrease for step size (on position stay)
#define DOWN_SPEED 0.5

using namespace std;
using namespace cv;
namespace decoder {
class GridFitter {
public:

	/**
	 * constructor
	 *
	 * @param id id of the roi
	 * @param ellipses a vector with possible ellipses for the roi
	 * @param scoringMethod method that should be used for scoring (FOR LATER USE MAYBE)
	 */
	GridFitter(int id, vector<Ellipse> ellipses, Grid::ScoringMethod scoringMethod = Grid::BINARYCOUNT);
	virtual ~GridFitter();

	/**
	 * processes all ellipses and returns the grids which fit to the given ellipses
	 *
	 * @return possible grids
	 */
	virtual vector < Grid > process();

private:
	/**
	 * id of the bounding box
	 */
	int id;

	/**
	 * vector of all possible ellipses for this bounding boxes
	 */
	vector < Ellipse > ellipses;

	/**
	 * method used for gridscoring
	 */
	Grid::ScoringMethod scoringMethod;

	/**
	 * performs the grit fitting step for a given ellipse
	 *
	 * @param ellipse the ellipse the grid should belong to
	 * @return the hopefully best grid
	 */
	virtual Grid fitGrid(Ellipse ellipse);

	/**
	 * guesses orientation of the tag
	 *
	 * @param roi part of the image where the tag is
	 * @param circle Point3f object with position and radius of the tag circle
	 */
	vector<Point2f> getOrientationVector(Ellipse &ellipse);

	/** Otsu binarization to reduce grey scale
	 *
	 * @param srcMat grayscale source image
	 * @return Otsu-threshold of the masked input image
	 */
	double getOtsuThreshold(Mat &srcMat);

	/**
	 * Tries to get the best grid according to the given ellipse with a gradient approach
	 *
	 * @param ellipse the ellipse
	 * @param angle initial orientation angle
	 * @param startX x part of the start coordinate
	 * @param startY y part of the start coordinate
	 * @return the best grid
	 */
	Grid fitGridGradient(Ellipse &ellipse, double angle, int startX, int startY);

	/**
	 * Returns the grid with the highest score
	 *
	 * @param grids a vector of grids
	 * @return the grid with the highest score
	 */
	Grid getBestGrid(vector<Grid> grids);

	/**
	 * Tries to get the best angle of a grid at the given position
	 *
	 * @param ellipse the ellipse the grid belongs to
	 * @param gsize size of the grid
	 * @param angle initial angle for fitting
	 * @param x horizontal part of grid position
	 * @param y vertical part of grid position
	 * @return best grid angle
	 */
	Grid fitGridAngle(Ellipse &ellipse, float gsize, double angle, int x, int y);
};
}
#endif /* GRIDFITTER_H_ */
