/*
 * gridFitter.cpp
 *
 *  Created on: 05.05.2014
 *	  Author: mareikeziese
 */

#include "GridFitter.h"
#include "util/ThreadPool.h"
#include <algorithm> // std::remove_if
#include <iterator> // std::distance


namespace decoder {
GridFitter::GridFitter(Grid::ScoringMethod scoringMethod) {
    this->scoringMethod = scoringMethod;
}

GridFitter::~GridFitter() = default;

std::vector<Tag> GridFitter::process(std::vector<Tag>&& taglist) const {
    // remove invalid tags
    taglist.erase(std::remove_if(taglist.begin(), taglist.end(), [](const Tag& tag) { return !tag.isValid(); }), taglist.end());
    static const size_t numThreads = 8;
    ThreadPool pool(numThreads);
    std::vector<std::future<void>> results;
    for (Tag& tag : taglist) {
        results.emplace_back(
           pool.enqueue([&] {
              for (TagCandidate& candidate : tag.getCandidates()) {
                  const Grid grid = fitGrid(candidate.getEllipse());
                  std::vector<Grid> grids;
                  grids.push_back(grid);
                  // Rotation by half cell (in both directions), because in some cases it's all you need to get a correct decoding
                  grids.emplace_back(grid.size(), grid.angle() + 15, grid.x(), grid.y(), grid.ell(), scoringMethod);
                  grids.emplace_back(grid.size(), grid.angle() - 15, grid.x(), grid.y(), grid.ell(), scoringMethod);
                  candidate.setGrids(std::move(grids));
        }
        }));
    }
    for(auto && result: results) result.get();
    return std::move(taglist);
}

Grid GridFitter::fitGrid(const Ellipse& ellipse) const {

    // Get ellipse orientation
    const auto &orient = getOrientationVector(ellipse);
    const cv::Vec2f v = orient[1] - orient[0];
    double alph = atan2(v[1], v[0]) * 180 / CV_PI + 90; // [-90, 270]

    //check for NaN values
    if (alph != alph) {
        alph = 0;
    }

    // Run multiple Grid Fittings with several start positions
    Grid bestGrid = fitGridGradient(ellipse, alph, ellipse.getCen().x,
        ellipse.getCen().y);
    srand(time(nullptr));     // Seed the random generator
    // TODO 16 ist besser als 4
    for (int i = 0; i < 16; i++) {
        // Calculate offset to the center of the ellipse
        const int offsetX = rand() % ellipse.getAxis().width - (ellipse.getAxis().width / 2);
        const int offsetY = rand() % ellipse.getAxis().width - (ellipse.getAxis().width / 2);
        Grid grid = fitGridGradient(ellipse, alph,
            ellipse.getCen().x + offsetX, ellipse.getCen().y + offsetY);

        if (grid > bestGrid) {
            bestGrid = grid;
        }
    }

    return bestGrid;
}

std::array<cv::Point2f, 2> GridFitter::getOrientationVector(const Ellipse &ellipse) const {

    const cv::Point3f circle(ellipse.getCen().x, ellipse.getCen().y,
        (ellipse.getAxis().width / 3.0));
    const cv::Mat &roi = ellipse.getBinarizedImage();

    // create circular cutout
    cv::Mat circMask(roi.rows, roi.cols, CV_8UC1, cv::Scalar(0));
    cv::circle(circMask, cv::Point(circle.x, circle.y), circle.z, cv::Scalar(1),
      CV_FILLED);

    // apply otsu binarization to this part
    //double otsut = getOtsuThreshold(roi);

    cv::Mat hcWhite = roi.mul(circMask);

    cv::Mat hcBlack = circMask.mul(255 - hcWhite);

    // Calculate moment => orientation of the tag
    const cv::Moments momw = moments(hcWhite, true);
    const cv::Moments momb = moments(hcBlack, true);

    const auto p0_x = momb.m10 / momb.m00;
    const auto p0_y = momb.m01 / momb.m00;

    const auto p1_x = momw.m10 / momw.m00;
    const auto p1_y = momw.m01 / momw.m00;

	return {cv::Point2f(p0_x, p0_y), cv::Point2f(p1_x, p1_y)};
}

double GridFitter::getOtsuThreshold(const cv::Mat &srcMat) const {
    //Code Snippet from
    //http://stackoverflow.com/questions/12953993/otsu-thresholding-for-depth-image
	// error removed & simplified

    cv::Mat copyImg = srcMat.clone();

    // remove all zeros
    const auto new_dataend = std::remove_if(copyImg.datastart, copyImg.dataend, [](const uchar p) {return p == 0; });

    // make a new matrix with only valid data (i.e. non-zero)
    cv::Mat nz(cv::Size(1, std::distance(copyImg.datastart, new_dataend)), cv::DataType<uchar>::type, copyImg.datastart);

    // compute  Otsu threshold
    const double thresh = threshold(nz, nz, 0, 255,
        CV_THRESH_BINARY | CV_THRESH_OTSU);

    return thresh;
}

Grid GridFitter::fitGridGradient(const Ellipse &ellipse, double angle, int startX,
  int startY) const {
    int step_size = INITIAL_STEP_SIZE;     // Amount of pixel the walk should jump

    const float gsize = (ellipse.getAxis().width / 3.0);
    const int x       = startX;
    const int y       = startY;
    // best grid so far
    Grid best = fitGridAngle(ellipse, gsize, angle, x, y);
    const int gs    = cvRound(ellipse.getAxis().width / 3.0);

    while (step_size > FINAL_STEP_SIZE) {
        // investigate the surrounding positions
        std::vector<Grid> grids;

        // right
        if (sqrt(
              static_cast<float>((ellipse.getCen().x - (x + step_size))
              * (ellipse.getCen().x - (x + step_size))
              + (ellipse.getCen().y - y) * (ellipse.getCen().y - y))) <= gs) {
            grids.push_back(
                fitGridAngle(ellipse, gsize, angle, x + step_size, y));
        }
        // left
        if (sqrt(
              static_cast<float>((ellipse.getCen().x - (x - step_size))
              * (ellipse.getCen().x - (x - step_size))
              + (ellipse.getCen().y - y) * (ellipse.getCen().y - y))) <= gs) {
            grids.push_back(
                fitGridAngle(ellipse, gsize, angle, x - step_size, y));
        }
        // down
        if (sqrt(
              static_cast<float>((ellipse.getCen().x - x) * (ellipse.getCen().x - x)
              + (ellipse.getCen().y - (y + step_size))
              * (ellipse.getCen().y - (y + step_size)))) <= gs) {
            grids.push_back(
                fitGridAngle(ellipse, gsize, angle, x, y + step_size));
        }
        // up
        if (sqrt(
              static_cast<float>((ellipse.getCen().x - x) * (ellipse.getCen().x - x)
              + (ellipse.getCen().y - (y - step_size))
              * (ellipse.getCen().y - (y - step_size)))) <= gs) {
            grids.push_back(
                fitGridAngle(ellipse, gsize, angle, x, y - step_size));
        }

        Grid best_neighbor = getBestGrid(grids);

        // the < is for the new score
        if (best_neighbor > best) {
            best      = best_neighbor;
            step_size = static_cast<int>(ceil(step_size * UP_SPEED));
        } else {
            step_size = static_cast<int>(step_size * DOWN_SPEED);
        }
    }

    return (best);
}

Grid GridFitter::fitGridAngle(const Ellipse &ellipse, float gsize, double angle,
  int x, int y) const {
    Grid cur(scoringMethod);
    Grid best(gsize, scoringMethod);

    int step_size = 3;
    int a         = angle;

    //for (int i = 0; i < 30; i++) {
    //cur = Grid(gsize, a + i, 0, x, y, ellipse, scoringMethod);

    //if (cur > best) {
    //best = cur;
    //}
    //}

    // Similar approach like in fitGridGradient, just using the angle
    while (step_size > 0) {
        Grid g1(gsize, a + step_size, x, y, ellipse, scoringMethod);
        Grid g2(gsize, a - step_size, x, y, ellipse, scoringMethod);

        int new_a;
        if (g1 > g2) {
            cur   = g1;
            new_a = a + step_size;
        } else {
            cur   = g2;
            new_a = a - step_size;
        }

        if (cur > best) {
            best       = cur;
            step_size *= 3;
            a          = new_a;
        } else {
            step_size /= 2;
        }
    }

    return best;
}

Grid GridFitter::getBestGrid(const std::vector<Grid> &grids) const {
    const auto it = std::max_element(grids.cbegin(), grids.cend());
    Grid best(scoringMethod);
    if (it != grids.cend() && *it > best) {
    	best = *it;
    }
    return best;
}

int GridFitter::bestGridAngleCorrection(const Grid &g) const {
    // index encoding 30Â°-step angles ranging from [0,5]
    int i    = 0;
    const cv::Mat &roi = g.ell().getTransformedImage();

    float mean1c = 0;
    float mean2c = 0;

    for (int offset = 0; offset < 6; offset++) {
        cv::Mat mask1(roi.rows, roi.cols, roi.type(), cv::Scalar(0));
        g.renderGridCell(mask1, cv::Scalar(255), 13, offset);
        cv::Scalar mean1;
        cv::Scalar std1;

        meanStdDev(roi, mean1, std1, mask1);

        cv::Mat mask2(roi.rows, roi.cols, roi.type(), cv::Scalar(0));
        g.renderGridCell(mask2, cv::Scalar(255), 14, offset);
        cv::Scalar mean2;
        cv::Scalar std2;

        meanStdDev(roi, mean2, std2, mask2);

        if (std::abs(mean1c - mean2c) < std::abs(mean1[0] - mean2[0])) {
            mean1c = mean1[0];
            mean2c = mean2[0];
            i      = offset;
        }
    }

    // 180-flip if supposed white half circle is darker than supposed black half circle
    if (mean1c < mean2c)
        i += 6;

    return i;
}
}
