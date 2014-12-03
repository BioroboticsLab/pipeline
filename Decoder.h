/*
 * Decoder.h
 *
 *  Created on: 05.05.2014
 *	  Author: mareikeziese
 */

#ifndef DECODER_H_
#define DECODER_H_

#include "./datastructure/Decoding.h"
#include "./datastructure/Tag.h"
#include "datastructure/Ellipse.h"
#include "datastructure/Grid.h"
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iostream>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#define CELL_SCALE 0.3

namespace decoder {
class Decoder {
public:
    // Struct for turning points
    struct EdgePoint {
        enum Type {
            PEAK,
            VALLEY,
            TRANSITION
        };

        // Direction if it's a transition
        enum Direction {
            UP,
            DOWN,
            PLAIN
        };

        Type type;
        double position = -1;
        double value;
        Direction dir;
    };

    /**
     * constructor
     *
     * @param id the id of the roi
     * @param grids a vector of possible grids of the SAME tag/roi
     */
    explicit Decoder();

    ~Decoder();

    /**
     * decodes the tags of the grids
     *
     * @return new struct named Decoding(consists of id, tagId, score)
     */
    std::vector<Tag> process(std::vector<Tag> &&taglist) const;

    /**
     * generates an circle edge
     *
     * @param g the grid
     * @param radius the radius of the circle
     * @return the vector with the brightness levels of the edge
     */
    //std::vector<unsigned char> generateEdge(const Grid &g, int radius);
private:

    /**
     * decodes a grid
     *
     * @param g the grid
     * @return decoding of the grid
     */
    Decoding decode(const Grid &g) const;

    /**
     * decodes the grid by using an include exclude approach
     *
     * @param g the grid
     * @return the decoding
     */
    Decoding includeExcludeDecode(const Grid &g) const;

    /**
     * helper method to calculate the fisher score depending on the given labels and the grid
     *
     * @param g the grid
     * @param labels for the cells
     * @param useBinaryImage determines whether the binary image should be used or the grayscale one
     * @return the fisher score
     */
    double fisherScore(const Grid &g, Mat &labels, bool useBinaryImage = false) const;

    /**
     * decodes the grid by walking along an edge of a circle
     *
     * @param g the grid
     * @return the decoding
     */
    Decoding edgeWalkerDecode(const Grid &g) const;

    /**
     * Returns the means for white and black color with standard deviation. Uses the black and white inner half circle as set.
     *
     * @param g the grid
     * @return a vector with the means and standard deviation of the white and black color (format:
     * [0]: white mean, [1]: white stddev, [2]: black mean, [3]: black stddev)
     */
    //std::vector<Scalar> colorMeans(const Grid &g);
};
}
#endif /* DECODER_H_ */
