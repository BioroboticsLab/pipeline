#ifndef GRID_H_
#define GRID_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>
#include "Ellipse.h"

// current tag design -- without inner border
/** outer radius to grid size ratio (tag design constant) */
#define ORR 2.31
/** tag radius to grid size ratio */
#define TRR 2.85
/** inner outer radius to grid size ratio */
#define IORR 1.289
/** inner radius to grid size ratio (tag design constant)*/
#define IRR 1.289

// the confidence interval between the standard deviations of separate cells (for the angle correction)
#define STD_EPS 10

using namespace cv;
using namespace std;

/** definition of different grid cell types (or cell shapes), which influence scoring */
enum GridType{
	SQUARE,
	CIRCLE,
	ARC
};

/**
 * Vec4f type alias for readability
 *
 * grid[0] = grid's x_center \n
 * grid[1] = grid's y_center \n
 * grid[2] = grid's x-y angle \n
 * grid[3] = grid's score (given roi)
 */

namespace decoder {
class Grid {
public:
	/**
	 * Enum for several scoring method
	 */
	enum ScoringMethod {
		BINARYCOUNT,
		FISHER,
		FASTFISHER
	};

	float size;
	float x;
	float y;
	float angle;
	float tilt;
	Ellipse ell;
	bool permutation; // Determines whether the grid is just a rotated grid or a fitted one

	/**
	 * Initialization method for multiple contructor
	 *
	 * @see init
	 */
	Grid(float size, float angle, float tilt,  int x,  int y, Ellipse ell, ScoringMethod scoringMethod = BINARYCOUNT);
	Grid(float size, float angle, float tilt,  int x,  int y, Ellipse ell, bool permutation, ScoringMethod scoringMethod = BINARYCOUNT);
	Grid(ScoringMethod scoringMethod = BINARYCOUNT);
	Grid(float s, ScoringMethod scoringMethod = BINARYCOUNT);
	virtual ~Grid();

	/**
	 * computes score of the grid and returns it
	 *
	 * @return the score
	 */
	double score();

	/**
	 * returns the currently used scoring method
	 *
	 * @return the used scoring method
	 */
	ScoringMethod scoringMethod();

	/**
	 * Render a grid cell of the given type and ID
	 *
	 * @param cell ID between [0,14]; which cell of the grid is to be rendered?
	 * @param offset angle offset to draw inner half circles with different angles 1 offset = 30°
	 * @return a vector with the contours of the cell
	 */
	vector<Point> renderGridCell(unsigned short cell, int offset = 0);

	/**
	 * Render a grid cell of the given type and ID
	 *
	 * @param cell ID between [0,14]; which cell of the grid is to be rendered?
	 * @param scale the scale of the cell within the interval [0, 1]
	 * @param offset angle offset to draw inner half circles with different angles 1 offset = 30°
	 * @return a vector with the contours of the cell
	 */
	vector<Point> renderScaledGridCell(unsigned short cell, double scale, int offset = 0);

	/**
	 * Determines whether the given grid is worser than itself (depending on the score).
	 * g1 > g2 doesn't mean g1.score > g2.score, it depends on the scoring method! You should
	 * read it that way: g1 fits better than g2.
	 *
	 * @param g the other grid
	 * @param true if the grid itself is better than the given grid
	 */
	bool operator>(Grid &g);

	/**
	 * Determines whether the given grid is better than itself (depending on the score).
	 * g1 < g2 doesn't mean g1.score < g2.score, it depends on the scoring method! It's
	 * quite similar to the > operator.
	 *
	 * @param g the other grid
	 * @param true if the grid itself is worser than the given grid
	 */
	bool operator<(Grid &g);

	/**
	 * Generates an edge of a circle and returns a vector with the intensities along the edge, with respect to the angle.
	 *
	 * @param radius the radius of the circle
	 * @param width the width of the edge
	 * @param useBinaryImage determines whether the binarized image should be used or the grayscale one
	 * @return a vector with the intensities along the edge
	 */
	vector<float> generateEdge(int radius, int width = 1, bool useBinaryImage = false);

	/**
	 * @see generateEdge
	 *
	 * @return a CV_32FC1 Mat with the edge data
	 */
	Mat generateEdgeAsMat(int radius, int width = 1, bool useBinaryImage = false);

	// ===== DEBUG METHODS =====

	/**
	 * Draw a known matrix grid on top of the corresponding region of interest
	 * the region of interest needs to be transformed \see ellipseTransform
	 *
	 * @param the scale of the cells
	 * @param useBinaryImage whether the binary image should be shown or ne normal one
	 * @return the Mat object the grid should be drawn into
	 */
	Mat drawGrid(double scale, bool useBinaryImage);
	Mat drawGrid(double scale);
	Mat drawGrid();
	Mat drawGrid(bool useBinaryImage);
private:

	/**
	 * internal score struct (primarily for caching)
	 */
	struct {
		double value;
		ScoringMethod metric;
	} _score;

	/**
	 * Initialization method for multiple contructor
	 *
	 * @param size size of the grid
	 * @param angle angle of the grid
	 * @param tilt tilt of the grid?
	 * @param x horizontal part of the position
	 * @param y vertical part of the position
	 * @param ell ellipse the grid belongs to
	 * @param score initial score of the grid
	 * @param scoringMethod used scoring method
	 */
	void init(float size, float angle, float tilt,  int x,  int y, Ellipse ell, bool permutation, ScoringMethod scoringMethod);

	/**
	 * Returns a orientation correction for a grid as offset.
	 *
	 * @param g the grid
	 * @return offset in cells
	 */
	int bestGridAngleCorrection();

	/**
	 * returns the mean of the intensities along a line
	 *
	 * @param xStart x ordinate of the start
	 * @param yStart y ordinate of the start
	 * @param xEnd x ordinate of the end
	 * @param yEnd y ordinate of the end
	 * @param size the length of the line
	 * @param useBinaryImage determines whether the binarized image should be used (default: false)
	 * @return the mean along the line
	 */
	float getMeanAlongLine(int xStart, int yStart, int xEnd, int yEnd, int size, bool useBinaryImage = false);


	// === Scoring methods ===

	/**
	 * Computes the score by binary count method (should converge to zero)
	 *
	 * @return the score
	 */
	double binaryCountScore();

	/**
	 * Computes the score by fisher method (higher Scores are better).
	 * Just in case we need it again (e.g. to improve accuracy)
	 *
	 * @return the score
	 */
	double fisherScore();

	// ======

};
}
#endif /* GRID_H_ */
