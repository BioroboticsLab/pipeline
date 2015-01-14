/*
 * gridFitter.h
 *
 *  Created on: 05.05.2014
 *	  Author: mareikeziese
 */

#ifndef GRIDFITTER_H_
#define GRIDFITTER_H_

#include "datastructure/Grid.h" // Grid
#include "datastructure/Tag.h"  // Tag
#include <vector>               // std::vector
#include <array>                // std::array
#include <opencv2/core/core.hpp> // cv::Point cv::Mat

#ifdef PipelineStandalone
#include "../config.h"
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#endif

#define _DOWN_SPEED 0.5

namespace decoder {
typedef struct {
    // initial step size
    int initial_step_size = 20;

    // step size at which the fitting-algorithm should terminate 
    int final_step_size = 0;

    // growth speed for the step size (on position change)
    float up_speed = 3;

    // speed decrease for step size (on position stay)
    float down_speed =  0.5;
} gridfitter_settings_t;


/**
 * forward declarations to reduce includes
 */
class Ellipse;


class GridFitter {
public:

    /**
     * constructor
     *
     * @param scoringMethod method that should be used for scoring (FOR LATER USE MAYBE)
     */
    GridFitter(Grid::ScoringMethod scoringMethod = Grid::BINARYCOUNT);
    ~GridFitter();

#ifdef PipelineStandalone
    GridFitter(const std::string &configFile);
#endif

    void loadSettings(gridfitter_settings_t&& settings);

    /**
     * processes all ellipses and returns the grids which fit to the given ellipses
     *
     * @return possible grids
     */
    std::vector<Tag> process(std::vector<Tag> &&taglist) const;

private:

    gridfitter_settings_t _settings;

#ifdef PipelineStandalone
    void loadConfigVars(const std::string &filename);
#endif

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
    Grid fitGrid(const Ellipse &ellipse) const;

    /**
     * guesses orientation of the tag
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
    int bestGridAngleCorrection(const Grid &g) const;
};
}
#endif /* GRIDFITTER_H_ */
