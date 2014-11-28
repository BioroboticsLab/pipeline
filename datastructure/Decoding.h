#ifndef DECODING_H_
#define DECODING_H_

#include "Ellipse.h"
#include "Grid.h"

namespace decoder {
/**
 * Struct for a tag decoding
 */
struct Decoding {
    /**
     * ID of the roi
     */
    int id;

    /**
     * Decoded ID of the tag
     */
    unsigned int tagId;

    /**
     * A score of the decoding
     */
    double score = -1;

    /**
     * The grid that has been decoded
     */
    Grid grid;
};
}

#endif /* DECODING_H_ */
