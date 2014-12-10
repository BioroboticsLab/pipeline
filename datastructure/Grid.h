#ifndef GRID_H_
#define GRID_H_

#include "Ellipse.h"
#include <algorithm>
#include <iostream>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#ifdef PipelineStandalone
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#endif

#define BINARYCOUNT_INIT (100000) // initial score for binary count method
#define FISHER_INIT (-1) // initial score for fisher method

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


namespace decoder {
/**
 * Vec4f type alias for readability
 *
 * grid[0] = grid's x_center \n
 * grid[1] = grid's y_center \n
 * grid[2] = grid's x-y angle \n
 * grid[3] = grid's score (given roi)
 */
class Grid {
public:
    /**
     * Enum for several scoring method
     */
    enum ScoringMethod {
        BINARYCOUNT,
        FISHER
    };

    /**
     * internal score struct (primarily for caching)
     * need to put it into the public area because of boost serialization issues
     */

private:
    struct Score {
        mutable double value;
        ScoringMethod metric;
        Score(ScoringMethod scoringMethod)
        	: value(scoringMethod == BINARYCOUNT ? BINARYCOUNT_INIT : FISHER_INIT)
        	, metric(scoringMethod)
        {}
    } m_score;

    float m_size;
    float m_x;
    float m_y;
    float m_angle;
    Ellipse m_ell;

public:

    float size() const { return m_size; }
    float x() const { return m_x; }
    float y() const { return m_y; }
    float angle() const { return m_angle; }
    const Ellipse& ell() const { return m_ell; }
    /**
     * Initialization method for multiple contructor
     *
     * @see Grid(float size, float angle, float tilt,  int x,  int y, Ellipse ell, bool permutation, ScoringMethod scoringMethod);
     */
    explicit Grid(ScoringMethod scoringMethod = BINARYCOUNT);
    explicit Grid(float s, ScoringMethod scoringMethod = BINARYCOUNT);

    /**
     * @param size size of the grid
     * @param angle angle of the grid
     * @param x horizontal part of the position
     * @param y vertical part of the position
     * @param ell ellipse the grid belongs to
     * @param scoringMethod used scoring method
     */
    explicit Grid(float size, float angle,  int x,  int y, Ellipse ell, ScoringMethod scoringMethod = BINARYCOUNT);
    ~Grid();

    /**
     * computes score of the grid and returns it
     *
     * @return the score
     */
    double score() const;

    /**
     * returns the currently used scoring method
     *
     * @return the used scoring method
     */
    ScoringMethod scoringMethod() const;

    /**
     * Render a grid cell of the given type and ID
     *
     * @param cell ID between [0,14]; which cell of the grid is to be rendered?
     * @param offset angle offset to draw inner half circles with different angles 1 offset = 30°
     * @return a reference to a vector containing a vector with the contours of the cell (thread local internal buffer)
     */
    const std::vector<std::vector<cv::Point>>& renderGridCell(unsigned short cell, int offset = 0) const {
    	return renderScaledGridCell(cell, 1, offset);
    }

    /**
     * Render a grid cell of the given type and ID
     *
     * @param cell ID between [0,14]; which cell of the grid is to be rendered?
     * @param scale the scale of the cell within the interval [0, 1]
     * @param offset angle offset to draw inner half circles with different angles 1 offset = 30°
     * @return a reference to a vector containing a vector with the contours of the cell (thread local internal buffer)
     */
    const std::vector<std::vector<cv::Point>>& renderScaledGridCell(unsigned short cell, double scale, int offset = 0) const;

    /**
     * Determines whether the given grid is worser than itself (depending on the score).
     * g1 > g2 doesn't mean g1.score > g2.score, it depends on the scoring method! You should
     * read it that way: g1 fits better than g2.
     *
     * @param g the other grid
     */
    bool operator>(const Grid &g) const;

    /**
     * Determines whether the given grid is better than itself (depending on the score).
     * g1 < g2 doesn't mean g1.score < g2.score, it depends on the scoring method! It's
     * quite similar to the > operator.
     *
     * @param g the other grid
     */
    bool operator<(const Grid &g) const;

    /**
     * Generates an edge of a circle and returns a vector with the intensities along the edge, with respect to the angle.
     *
     * @param radius the radius of the circle
     * @param width the width of the edge
     * @param useBinaryImage determines whether the binarized image should be used or the grayscale one
     * @return a vector with the intensities along the edge
     */
    std::vector<float> generateEdge(int radius, int width = 1, bool useBinaryImage = false) const;

    /**
     * @see generateEdge
     *
     * @return a CV_32FC1 cv::Mat with the edge data
     */
    cv::Mat generateEdgeAsMat(int radius, int width = 1, bool useBinaryImage = false) const;

    // ===== DEBUG METHODS =====

    /**
     * Draw a known matrix grid on top of the corresponding region of interest
     * the region of interest needs to be transformed \see ellipseTransform
     *
     * @param scale the scale of the cells
     * @param useBinaryImage whether the binary image should be shown or ne normal one
     * @return the cv::Mat object the grid should be drawn into
     */
    cv::Mat drawGrid(double scale, bool useBinaryImage) const;
    cv::Mat drawGrid(double scale) const;
    cv::Mat drawGrid() const;
    cv::Mat drawGrid(bool useBinaryImage) const;
private:

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
    float getMeanAlongLine(int xStart, int yStart, int xEnd, int yEnd, int size, bool useBinaryImage = false) const;

    // === Scoring methods ===

    /**
     * Computes the score by binary count method (should converge to zero)
     *
     * @return the score
     */
    double binaryCountScore() const;

    /**
     * Computes the score by fisher method (higher Scores are better).
     * Just in case we need it again (e.g. to improve accuracy)
     *
     * @return the score
     */
    double fisherScore() const;

    // ======
};
}
#endif /* GRID_H_ */
