/*
 * gridFitter.cpp
 *
 *  Created on: 05.05.2014
 *	  Author: mareikeziese
 */

#include "GridFitter.h"
#include "util/ThreadPool.h"

using namespace cv;

namespace decoder {
GridFitter::GridFitter(Grid::ScoringMethod scoringMethod) {
    this->scoringMethod = scoringMethod;
}

GridFitter::~GridFitter() {
    // TODO Auto-generated destructor stub
}

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
                  grids.emplace_back(grid.size(), grid.angle() + 15, 0, grid.x(), grid.y(), grid.ell(), true, scoringMethod);
                  grids.emplace_back(grid.size(), grid.angle() - 15, 0, grid.x(), grid.y(), grid.ell(), true, scoringMethod);
                  candidate.setGrids(std::move(grids));
        }
        }));
    }
    for(auto && result: results) result.get();
    return std::move(taglist);
}

Grid GridFitter::fitGrid(Ellipse& ellipse) const {
    // Convert image to gray scale (maybe obsolete)
    Mat grayImage;
    if (ellipse.transformedImage.channels() > 2) {
        cvtColor(ellipse.transformedImage, grayImage, CV_BGR2GRAY);
        ellipse.transformedImage = grayImage;
    }

    // Binarize image first (just for new Scoring)
    //threshold(ellipse.transformedImage, ellipse.binarizedImage, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    //adaptiveThreshold(ellipse.transformedImage, ellipse.binarizedImage, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 13, 5);
    adaptiveThreshold(ellipse.transformedImage, ellipse.binarizedImage, 255,
      ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 21, 3);

    // Get ellipse orientation
    const auto &orient = getOrientationVector(ellipse);
    const Vec2f v = orient[1] - orient[0];
    float alph = atan2(v[1], v[0]) * 180 / CV_PI + 90;

    //check for NaN values
    if (alph != alph) {
        alph = 0;
    }

    // Run multiple Grid Fittings with several start positions
    Grid bestGrid = fitGridGradient(ellipse, static_cast<double>(alph), ellipse.cen.x,
        ellipse.cen.y);
    srand(time(NULL));     // Seed the random generator
    // TODO 16 ist besser als 4
    for (int i = 0; i < 16; i++) {
        // Calculate offset to the center of the ellipse
        const int offsetX = rand() % ellipse.axis.width - (ellipse.axis.width / 2);
        const int offsetY = rand() % ellipse.axis.width - (ellipse.axis.width / 2);
        Grid grid = fitGridGradient(ellipse, static_cast<double>(alph),
            ellipse.cen.x + offsetX, ellipse.cen.y + offsetY);

        if (grid > bestGrid) {
            bestGrid = grid;
        }
    }

    return bestGrid;
}

std::array<Point2f, 2> GridFitter::getOrientationVector(const Ellipse &ellipse) const {

    const Point3f circle(ellipse.cen.x, ellipse.cen.y,
        (ellipse.axis.width / 3.0));
    const Mat &roi = ellipse.binarizedImage;

    // create circular cutout
    Mat circMask(roi.rows, roi.cols, CV_8UC1, Scalar(0));
    cv::circle(circMask, Point(circle.x, circle.y), circle.z, Scalar(1),
      CV_FILLED);

    // apply otsu binarization to this part
    //double otsut = getOtsuThreshold(roi);

    Mat hcWhite = roi.mul(circMask);

    Mat hcBlack = circMask.mul(255 - hcWhite);

    // Calculate moment => orientation of the tag
    const Moments momw = moments(hcWhite, true);
    const Moments momb = moments(hcBlack, true);

    const auto p0_x = momb.m10 / momb.m00;
    const auto p0_y = momb.m01 / momb.m00;

    const auto p1_x = momw.m10 / momw.m00;
    const auto p1_y = momw.m01 / momw.m00;

    return {Point2f(p0_x, p0_y), Point2f(p1_x, p1_y)};
}

double GridFitter::getOtsuThreshold(const Mat &srcMat) const {
    //Code Snippet from
    //http://stackoverflow.com/questions/12953993/otsu-thresholding-for-depth-image
    Mat copyImg = srcMat.clone();
    uchar* ptr     = copyImg.datastart;
    uchar* ptr_end = copyImg.dataend;
    while (ptr < ptr_end) {
        if (*ptr == 0) {         // swap if zero
            uchar tmp = *ptr_end;
            *ptr_end = *ptr;
            *ptr     = tmp;
            ptr_end--;             // make array smaller
        } else {
            ptr++;
        }
    }

    // make a new matrix with only valid data
    Mat nz = Mat(std::vector<uchar>(copyImg.datastart, ptr_end), true);

    // compute  Otsu threshold
    const double thresh = threshold(nz, nz, 0, 255,
        CV_THRESH_BINARY | CV_THRESH_OTSU);

    return thresh;
}

Grid GridFitter::fitGridGradient(const Ellipse &ellipse, double angle, int startX,
  int startY) const {
    int step_size = INITIAL_STEP_SIZE;     // Amount of pixel the walk should jump

    float gsize = (ellipse.axis.width / 3.0);
    int x       = startX;
    int y       = startY;
    // best grid so far
    Grid best = fitGridAngle(ellipse, gsize, angle, x, y);
    int gs    = cvRound(ellipse.axis.width / 3.0);

    while (step_size > FINAL_STEP_SIZE) {
        // investigate the surrounding positions
        std::vector<Grid> grids;

        // right
        if (sqrt(
              static_cast<float>((ellipse.cen.x - (x + step_size))
              * (ellipse.cen.x - (x + step_size))
              + (ellipse.cen.y - y) * (ellipse.cen.y - y))) <= gs) {
            grids.push_back(
                fitGridAngle(ellipse, gsize, angle, x + step_size, y));
        }
        // left
        if (sqrt(
              static_cast<float>((ellipse.cen.x - (x - step_size))
              * (ellipse.cen.x - (x - step_size))
              + (ellipse.cen.y - y) * (ellipse.cen.y - y))) <= gs) {
            grids.push_back(
                fitGridAngle(ellipse, gsize, angle, x - step_size, y));
        }
        // down
        if (sqrt(
              static_cast<float>((ellipse.cen.x - x) * (ellipse.cen.x - x)
              + (ellipse.cen.y - (y + step_size))
              * (ellipse.cen.y - (y + step_size)))) <= gs) {
            grids.push_back(
                fitGridAngle(ellipse, gsize, angle, x, y + step_size));
        }
        // up
        if (sqrt(
              static_cast<float>((ellipse.cen.x - x) * (ellipse.cen.x - x)
              + (ellipse.cen.y - (y - step_size))
              * (ellipse.cen.y - (y - step_size)))) <= gs) {
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
        Grid g1(gsize, a + step_size, 0, x, y, ellipse, scoringMethod);
        Grid g2(gsize, a - step_size, 0, x, y, ellipse, scoringMethod);

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

int GridFitter::bestGridAngleCorrection(Grid g) const {
    // index encoding 30Â°-step angles ranging from [0,5]
    int i    = 0;
    const Mat &roi = g.ell().transformedImage;

    float mean1c = 0;
    float mean2c = 0;

    for (int j = 0; j < 6; j++) {
        Mat mask1(roi.rows, roi.cols, roi.type(), Scalar(0));
        std::vector<std::vector<Point> > conts1;
        conts1.push_back(g.renderGridCell(13, j));
        drawContours(mask1, conts1, 0, Scalar(255), CV_FILLED);
        Scalar mean1;
        Scalar std1;

        meanStdDev(roi, mean1, std1, mask1);

        Mat mask2(roi.rows, roi.cols, roi.type(), Scalar(0));
        std::vector<std::vector<Point> > conts2;
        conts2.push_back(g.renderGridCell(14, j));
        drawContours(mask2, conts2, 0, Scalar(255), CV_FILLED);
        Scalar mean2;
        Scalar std2;

        meanStdDev(roi, mean2, std2, mask2);

        if (abs(mean1c - mean2c) < abs(mean1[0] - mean2[0])) {
            mean1c = mean1[0];
            mean2c = mean2[0];
            i      = j;
        }
    }

    // 180-flip if supposed white half circle is darker than supposed black half circle
    if (mean1c < mean2c)
        i += 6;

    return i;
}
}
