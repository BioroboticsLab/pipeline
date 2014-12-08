/*
 * Decoder.cpp
 *
 *  Created on: 05.05.2014
 *	Author: mareikeziese
 */

#include "Decoder.h"
#include <utility> // std::move

#include <bitset>


namespace decoder {
Decoder::Decoder() {
}

Decoder::~Decoder() = default;

std::vector<Tag> Decoder::process(std::vector<Tag>&& taglist) const {
    // remove invalid tags
    taglist.erase(std::remove_if(taglist.begin(), taglist.end(), [](Tag& tag) { return !tag.isValid(); }), taglist.end());
    for (Tag& tag : taglist) {
        for (TagCandidate& candidate : tag.getCandidates()) {
            std::vector<Decoding> decodings;
            decodings.reserve(candidate.getGrids().size() * 2);
            for (Grid& grid : candidate.getGrids()) {
                decodings.push_back(includeExcludeDecode(grid));
                decodings.push_back(edgeWalkerDecode(grid));
            }
            // Just keep the best three decodings, determined by fisher score
            const size_t num = std::min<size_t>(decodings.size(), 3);
            std::partial_sort(decodings.begin(), decodings.begin() + num, decodings.end(),
                              [](Decoding const& d1, Decoding const& d2) {
                                return d1.score > d2.score;
                              });
            // remove remaining decodings
            decodings.erase(decodings.begin() + num, decodings.end());

#ifdef DEBUG_SHOW_DECODED_GRID
            // Show the grids for debug propose
            size_t cnt = 0;
            for (Decoding& decoding : decodings) {
                uint tagId = decoding.tagId;
                std::bitset<12> binDigits(tagId);
                cv::Mat draw = decoding.grid.drawGrid();
                stringstream ss;
                ss << "Grid " << cnt << " | Decoding: " << decoding.tagId << " | Binary Digits: " << binDigits;
                string windowName = ss.str();
                namedWindow(windowName, WINDOW_NORMAL);
                imshow(windowName, draw);
                ++cnt;
            }
            waitKey();
            destroyAllWindows();
#endif

            candidate.setDecodings(std::move(decodings));
        }
    }
    return std::move(taglist);
}

Decoding Decoder::decode(const Grid &g) const {
    return edgeWalkerDecode(g);
}

Decoding Decoder::includeExcludeDecode(const Grid &g) const {
    const cv::Mat &image = g.ell().getBinarizedImage();

    cv::Mat whiteMask(image.rows, image.cols, image.type(), cv::Scalar(0));
    cv::Mat blackMask(image.rows, image.cols, image.type(), cv::Scalar(0));
    std::vector<std::vector<cv::Point> > conts(1);
    conts[0] = g.renderScaledGridCell(13, CELL_SCALE);
    drawContours(whiteMask, conts, 0, cv::Scalar(1), CV_FILLED);
    conts[0] = g.renderScaledGridCell(14, CELL_SCALE);
    drawContours(blackMask, conts, 0, cv::Scalar(1), CV_FILLED);

    cv::Scalar mean;
    cv::Scalar std;
    meanStdDev(image, mean, std, whiteMask);
    const double whiteCenter = mean[0];
    meanStdDev(image, mean, std, blackMask);
    const double blackCenter = mean[0];

    // Initial labeling
    cv::Mat labels(12, 1, CV_8U);
    cv::Mat means(12, 1, CV_32F);
    cv::Mat stds(12, 1, CV_32F);
    for (int i = 0; i < 12; i++) {
        cv::Mat cellMask(image.rows, image.cols, image.type(), cv::Scalar(0));

        conts[0] = g.renderScaledGridCell(i, CELL_SCALE);
        drawContours(cellMask, conts, 0, cv::Scalar(1), CV_FILLED);
        cv::Scalar mean;
        cv::Scalar std;
        meanStdDev(image, mean, std, cellMask);
        labels.at<unsigned char>(i) =
          std::abs(whiteCenter - mean[0]) < std::abs(blackCenter - mean[0]) ? 1 : 0;

        means.at<float>(i) = mean[0];
        stds.at<float>(i)  = std[0];
    }

    double score = fisherScore(g, labels);
    double blackMovedScore = -1;
    double whiteMovedScore = -1;
    int blackMaxIdx[2];
    int whiteMinIdx[2];
    while (true) {
        blackMovedScore = -1;
        whiteMovedScore = -1;

        // Get the brightest black cell and the darkest white cell
        minMaxIdx(means, nullptr, nullptr, nullptr, blackMaxIdx,
          cv::Mat::ones(12, 1, CV_8U) - labels);
        minMaxIdx(means, nullptr, nullptr, whiteMinIdx, nullptr, labels);

        // Change class of the cell, but just if there is one
        if (blackMaxIdx[1] != -1) {
            labels.at<unsigned char>(blackMaxIdx[1]) = 1;
            blackMovedScore = fisherScore(g, labels);
            labels.at<unsigned char>(blackMaxIdx[1]) = 0;
        }

        // Well moving a white cell doesn't do anything, but maybe it will with other data
        if (whiteMinIdx[1] != -1) {
            labels.at<unsigned char>(whiteMinIdx[1]) = 0;
            whiteMovedScore = fisherScore(g, labels);
            labels.at<unsigned char>(whiteMinIdx[1]) = 1;
        }

        if (blackMovedScore > score && blackMovedScore > whiteMovedScore
          && blackMaxIdx[1] != -1) {
            score = blackMovedScore;
            labels.at<unsigned char>(blackMaxIdx[1]) = 1;
        } else if (whiteMovedScore > score && whiteMovedScore > blackMovedScore
          && whiteMinIdx[1] != -1) {
            score = whiteMovedScore;
            labels.at<unsigned char>(whiteMinIdx[1]) = 0;
        } else {
            break;
        }
    }

    unsigned int res = 0;
    for (int i = 0; i < 12; i++) {
        res += (static_cast<int>(labels.at<unsigned char>(i))) << (11 - i);
    }

    // Pack it into the decoding
    return Decoding(res, fisherScore(g, labels, true), g);
}

double Decoder::fisherScore(const Grid &g, cv::Mat &labels, bool useBinaryImage) const {
    const cv::Mat &image = useBinaryImage ? g.ell().getBinarizedImage() : g.ell().getTransformedImage();
    std::vector<std::vector<cv::Point> > conts(1);
    cv::Mat whiteMask(image.rows, image.cols, image.type(), cv::Scalar(0));
    cv::Mat blackMask(image.rows, image.cols, image.type(), cv::Scalar(0));
    conts[0] = g.renderScaledGridCell(13, CELL_SCALE);
    drawContours(whiteMask, conts, 0, cv::Scalar(255), CV_FILLED);
    conts[0] = g.renderScaledGridCell(14, CELL_SCALE);
    drawContours(blackMask, conts, 0, cv::Scalar(255), CV_FILLED);

    for (int i = 0; i < 12; i++) {
        // Add the cell to the mask of the designated group
        cv::Mat *cellMask;
        if (labels.at<unsigned char>(i) == 1) {
            cellMask = &whiteMask;
        } else {
            cellMask = &blackMask;
        }
        conts[0] = g.renderScaledGridCell(i, CELL_SCALE);
        drawContours(*cellMask, conts, 0, cv::Scalar(255), CV_FILLED);
    }
    cv::Scalar whiteMean;
    cv::Scalar whiteStd;
    meanStdDev(image, whiteMean, whiteStd, whiteMask);

    cv::Scalar blackMean;
    cv::Scalar blackStd;
    meanStdDev(image, blackMean, blackStd, blackMask);

    return ((whiteMean[0] - blackMean[0]) * (whiteMean[0] - blackMean[0]))
           / (whiteStd[0] * whiteStd[0] + blackStd[0] * blackStd[0]);
}

Decoding Decoder::edgeWalkerDecode(const Grid &g) const {
    cv::Mat edge = g.generateEdgeAsMat(
        static_cast<int>(IORR * g.size() + (ORR * g.size() - IORR * g.size()) * 0.5), 1);
    const int edgeSize    = edge.size().height;
    const double cellSize = edgeSize / 12.0;

    // Generate cut at the half way between the min and the max value of the edge
    double min;
    double max;
    minMaxIdx(edge, &min, &max);
    const double cut = min + (max - min) / 2;

    // Get peaks and valleys of the edge, by walking from left to right
    // Indicate the direction of the curve
    EdgePoint::Direction leftDir  = EdgePoint::PLAIN;
    EdgePoint::Direction rightDir = EdgePoint::PLAIN;
    std::vector<EdgePoint> turningPoints;
    for (int i = 0; i < edgeSize; i++) {
        EdgePoint turningPoint;
        turningPoint.value    = edge.at<float>(i);
        turningPoint.position = i;

        // Follow the left side till something changes
        for (int prevIdx = (i - 1 + edgeSize) % edgeSize;;
          prevIdx = (prevIdx - 1 + edgeSize) % edgeSize) {
            const float prevVal = edge.at<float>(prevIdx);
            if (prevVal > turningPoint.value) {
                leftDir = EdgePoint::UP;
                break;
            } else if (prevVal < turningPoint.value) {
                leftDir = EdgePoint::DOWN;
                break;
            }
        }

        // Follow the right side till something changes
        for (int nextIdx = (i + 1) % edgeSize;;
          nextIdx = (nextIdx + 1) % edgeSize) {
            const float nextVal = edge.at<float>(nextIdx);

            //i = nextIdx - 1; // Skip points on the same level

            if (nextVal > turningPoint.value) {
                rightDir = EdgePoint::UP;
                break;
            } else if (nextVal < turningPoint.value) {
                rightDir = EdgePoint::DOWN;
                break;
            }
        }

        // Turning point if both sides changes in the same direction
        if (leftDir == rightDir) {
            if (leftDir == EdgePoint::UP) {
                turningPoint.type = EdgePoint::VALLEY;
            } else if (leftDir == EdgePoint::DOWN) {
                turningPoint.type = EdgePoint::PEAK;
            }

            turningPoints.push_back(turningPoint);
        }
    }

    std::ofstream turningPointsFile(
        "../../../arbeit/data/decoding/edgeWalker/turning_points_003_001.txt");
    for (unsigned int k = 0; k < turningPoints.size(); k++) {
        turningPointsFile << turningPoints[k].position << " "
                          << turningPoints[k].value << std::endl;
    }
    turningPointsFile.close();

    // Find transition points
#define TP_EPS 0 // interval a turning point can move
    std::vector<EdgePoint> transitionPoints;
    for (unsigned int i = 0; i < turningPoints.size(); i++) {
        EdgePoint leftPoint  = turningPoints[i];
        EdgePoint rightPoint = turningPoints[(i + 1) % turningPoints.size()];

        EdgePoint transitionPoint;
        transitionPoint.type = EdgePoint::TRANSITION;
        // Just watch direction changes which are big enough
        if ((leftPoint.value - cut > 0 ? 1 : -1)
          != (rightPoint.value - cut > 0 ? 1 : -1)) {
            //if (std::abs(rightPoint.position - leftPoint.position) < cellSize + 2 * TP_EPS) {
            //// turning points are near enough to keep us in watch
            //transitionPoint.value = (leftPoint.value + rightPoint.value) / 2;
            //transitionPoint.position = (leftPoint.position + rightPoint.position) / 2;
            //} else {
            // Walk the edge to the right from the leftPoint and to the left from the rightPoint till a point goes over the cut. This point is the transition point!
            for (int j = 1;; j++) {
                const int leftPos  = (static_cast<int>(leftPoint.position) + j) % edgeSize;
                const float leftVal  = edge.at<float>(leftPos);
                const int rightPos = (static_cast<int>(rightPoint.position) - j + edgeSize)
                  % edgeSize;
                const float rightVal = edge.at<float>(rightPos);

                // TODO maybe use better heuristics for usable differences
                if ((leftPoint.value - cut >= 0 ? 1 : -1)
                  != (leftVal - cut > 0 ? 1 : -1)) {
                    transitionPoint.position = leftPos - 0.5;
                    transitionPoint.value    = (leftPoint.value + leftVal) / 2;
                    break;
                } else if ((rightPoint.value - cut >= 0 ? 1 : -1)
                  != (rightVal - cut > 0 ? 1 : -1)) {
                    transitionPoint.position = rightPos + 0.5;
                    transitionPoint.value    = (rightPoint.value + rightVal) / 2;
                    break;
                }
            }
            //}
            transitionPoint.dir =
              leftPoint.type == EdgePoint::PEAK ?
              EdgePoint::DOWN : EdgePoint::UP;
            transitionPoints.push_back(transitionPoint);
        }
    }

    std::ofstream transitionPointsFile(
        "../../../arbeit/data/decoding/edgeWalker/transition_points_003_001.txt");
    for (unsigned int k = 0; k < transitionPoints.size(); k++) {
        transitionPointsFile << transitionPoints[k].position << " "
                             << transitionPoints[k].value << std::endl;
    }
    transitionPointsFile.close();

    // correction at the first transition point
    const double cellBeginning = round((transitionPoints[0].position) / cellSize)
      * cellSize;
    const double correction = cellBeginning - transitionPoints[0].position;

    // Apply correction
    for (unsigned int i = 0; i < transitionPoints.size(); i++) {
        // fmod usage to keep the position positive
        transitionPoints[i].position = fmod(
            transitionPoints[i].position + correction + edgeSize, edgeSize);
        transitionPoints[i].value = edge.at<float>(
            transitionPoints[i].position);
    }

    // At least the decoding
    cv::Mat labels(12, 1, CV_8UC1, 2);
    for (unsigned int i = 0; i < transitionPoints.size(); i++) {
        EdgePoint leftPoint  = transitionPoints[i];
        EdgePoint rightPoint = transitionPoints[(i + 1)
          % transitionPoints.size()];

        const int curBit = leftPoint.dir == EdgePoint::UP;         // Compute possible current bit

        const int leftCellId  = round(leftPoint.position / cellSize);
        const int rightCellId = round(rightPoint.position / cellSize);

        for (int j = 0; j < (rightCellId - leftCellId + 12) % 12; j++) {
            labels.at<unsigned char>((leftCellId + j) % 12) =
              static_cast<unsigned char>(curBit);
        }
    }

    //std::cout << labels << std::endl;
    unsigned int res = 0;
    for (int i = 0; i < 12; i++) {
        res += labels.at<unsigned char>(i) << (11 - i);
    }

    // Pack it into the decoding
    return Decoding(res, fisherScore(g, labels, true), g);
}
}
