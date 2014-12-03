/*
 * gridFitter.h
 *
 *  Created on: 05.05.2014
 *	  Author: mareikeziese
 */

#ifndef GRIDFITTER_H_
#define GRIDFITTER_H_

#include "datastructure/Ellipse.h"
#include "datastructure/Grid.h"
#include "datastructure/Tag.h"
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <array> // std::array

// Constants for optimized detection of the tag center
// initial step size
#define INITIAL_STEP_SIZE 20
// step size the fitting-algorithm should terminate
#define FINAL_STEP_SIZE 0
// growth speed for the step size (on position change)
#define UP_SPEED 3
// speed decrease for step size (on position stay)
#define DOWN_SPEED 0.5


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
    GridFitter(Grid::ScoringMethod scoringMethod = Grid::BINARYCOUNT);
    ~GridFitter();

    /**
     * processes all ellipses and returns the grids which fit to the given ellipses
     *
     * @return possible grids
     */
    std::vector<Tag> process(std::vector<Tag> &&taglist) const;

private:

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
    Grid fitGrid(Ellipse &ellipse) const;

    /**
     * guesses orientation of the tag
     *
     * @param roi part of the image where the tag is
     * @param circle cv::Point3f object with position and radius of the tag circle
     */
    std::array<cv::Point2f, 2> getOrientationVector(const Ellipse &ellipse) const;

    /** Otsu binarization to reduce grey scale
     *
     * @param srcMat grayscale source image
     * @return Otsu-threshold of the masked input image
     */
    double getOtsuThreshold(const cv::Mat &srcMat) const;

    /**
     * Tries to get the best grid according to the given ellipse with a gradient approach
     *
     * @param ellipse the ellipse
     * @param angle initial orientation angle
     * @param startX x part of the start coordinate
     * @param startY y part of the start coordinate
     * @return the best grid
     */
    Grid fitGridGradient(const Ellipse &ellipse, double angle, int startX, int startY) const;

    /**
     * Returns the grid with the highest score
     *
     * @param grids a vector of grids
     * @return the grid with the highest score
     */
    Grid getBestGrid(const std::vector<Grid> &grids) const;

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
    Grid fitGridAngle(const Ellipse &ellipse, float gsize, double angle, int x, int y) const;

    /**
     * Returns a orientation correction for a grid as offset.
     *
     * @param g the grid
     * @return offset in cells
     */
    int bestGridAngleCorrection(Grid g) const;
};
}
#endif /* GRIDFITTER_H_ */
