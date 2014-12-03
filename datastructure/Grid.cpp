#include "Grid.h"

namespace decoder {
// === Constructors and initializer ===
Grid::Grid(float size, float angle, float tilt,  int x,  int y, Ellipse ell, ScoringMethod scoringMethod)
	: Grid(size, angle, tilt, x, y, ell, false, scoringMethod)
{
}

Grid::Grid(float size, float angle, float tilt, int x, int y, Ellipse ell, bool permutation, ScoringMethod scoringMethod)
	: _score(scoringMethod)
	, size(size)
	, x(x)
	, y(y)
	, angle(angle)
	, tilt(tilt)
	, ell(ell)
	, permutation(permutation)
{
    // Need to binarize the image, because we need it for scoring
    if (this->ell.transformedImage.type() != CV_8U) {
        Mat grayImage;
        cvtColor(this->ell.transformedImage, grayImage, CV_BGR2GRAY);
        this->ell.transformedImage = grayImage;
    }

    // Binarize image first (just for new Scoring)
    adaptiveThreshold(this->ell.transformedImage, this->ell.binarizedImage, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 21, 3);
}

Grid::Grid(ScoringMethod scoringMethod)
	: Grid(0, 0, 0, 0, 0, Ellipse(), false, scoringMethod)
{
}

Grid::Grid(float size, ScoringMethod scoringMethod)
	: Grid(size, 0, 0, 0, 0, Ellipse(), false, scoringMethod)
{
}

Grid::~Grid() = default;

// ===

// === Scoring methods ===
Grid::ScoringMethod Grid::scoringMethod() const {
    return _score.metric;
}

double Grid::score() {
    // determine whether the grid is a dummy or not
    if (_score.metric == BINARYCOUNT && _score.value == BINARYCOUNT_INIT && ell.binarizedImage.total() > 0) {
        _score.value = binaryCountScore();
    } else if (_score.metric == FISHER && _score.value == FISHER_INIT && ell.transformedImage.total() > 0) {
        _score.value = fisherScore();
    }
    return _score.value;
}

double Grid::binaryCountScore() const {
    const Mat &binImg = ell.binarizedImage;

    Mat scores (3, 1, CV_64FC1);
    // for each cell calculate its size (cell size) and its mean intensity (means)
    for (int j = 12; j < 15; j++) {
        Mat mask = Mat(binImg.rows, binImg.cols, binImg.type(), Scalar(0));

        std::vector< std::vector <Point> > conts;
        conts.push_back(renderGridCell(j));
        drawContours(mask, conts, 0, Scalar(1), CV_FILLED);

        const Mat whiteCellPixel   = binImg.mul(mask);       // just keep the pixel within the cell
        const double whitePixelAmount = static_cast<double>(countNonZero(whiteCellPixel));
        const double blackPixelAmount = static_cast<double>(countNonZero(mask - whiteCellPixel));

        if (j == 12 || j == 13) {
            // white inner half circle or white outer border
            scores.at<double>(14 - j) =  whitePixelAmount == 0. ? blackPixelAmount : blackPixelAmount / whitePixelAmount;
        } else if (j == 14) {
            //supposed black inner half circle
            scores.at<double>(14 - j) =  blackPixelAmount == 0. ? whitePixelAmount : whitePixelAmount / blackPixelAmount;
        }
    }

    return sum(scores)[0];
}

double Grid::fisherScore() const {
    // 37/46 = 80.43% with Sb = |black - white| look into other intervariances
    // 41/47 = 89.13% with kind of A scaling
    // determine best orientation
    const Mat &roi = ell.transformedImage;

    double black = -1;
    double white = -1;

    Mat smoothROI;
    GaussianBlur(roi, smoothROI, Size(7, 7), 0, 0, BORDER_DEFAULT);
    minMaxIdx(smoothROI, &black, &white);

    Mat means  = Mat(15, 1, CV_32F);
    Mat labels = Mat(15, 1, CV_32S);
    TermCriteria max_it = TermCriteria(TermCriteria::COUNT, 3, 2);
    Mat centers;
    std::vector<Mat> masks;

    // for each cell calculate its size (cellsize) and its mean intensity (means)
    for (int j = 0; j < 15; j++) {
        Mat mask(roi.rows, roi.cols, roi.type(), Scalar(0));
        std::vector< std::vector <Point> > conts;
        conts.push_back(renderGridCell(j));
        drawContours(mask, conts, 0, Scalar(255), CV_FILLED);
        masks.push_back(mask);

        Scalar mean;
        Scalar std;
        meanStdDev(roi, mean, std, mask);

        means.at<float>(j) = mean[0];
    }

    // assume the color for each cell
    for (int j = 0; j < 15; j++) {
        if (abs(black - means.at<float>(j)) < abs(white - means.at<float>(j))) {
            // cell is assumed black
            labels.at<int>(j) = 0;
        } else {
            // cell is assumed white
            labels.at<int>(j) = 1;
        }
    }

    // tag design 12,13 are white, 14 is black
    labels.at<int>(12) = 1;
    labels.at<int>(13) = 1;
    labels.at<int>(14) = 0;

    kmeans(means, 2, labels, max_it, 1, KMEANS_USE_INITIAL_LABELS, centers);

    // tag design 12,13 are white, 14 is black
    labels.at<int>(12) = 1;
    labels.at<int>(13) = 1;
    labels.at<int>(14) = 0;

    black = centers.at<float>(0, 0);
    white = centers.at<float>(1, 0);

    if (_score.value != -1) {
        for (int lab = 0; lab < labels.rows; lab++) {
            if (labels.at<int>(lab) == 0 && lab < 12) {
                if (abs(means.at<float>(lab) - white) < abs(means.at<float>(lab) - black)) {
                    std::cout << "eh, uppsb" << std::endl;
                }
            } else if (lab < 12) {
                if (abs(means.at<float>(lab) - white) > abs(means.at<float>(lab) - black)) {
                    std::cout << "eh, uppsw" << std::endl;
                }
            }
            std::cout << labels.at<int>(lab);
            if (lab == 11) {
                std::cout << " | ";
            }
        }
        std::cout << std::endl;
        return -1;
    }

    float Swb = 0;
    int nw_b  = 0;
    float Sww = 0;
    int nw_w  = 0;

    // calculate intra variance (Score between - Sw)
    for (int j = 0; j < 15; j++) {
        float vari = 0;
        for (int r = 0; r < roi.cols; r++) {
            for (int c = 0; c < roi.rows; c++) {
                if (masks[j].at<unsigned char>(c, r) == 255) {
                    if (labels.at<int>(j) == 0) {
                        vari += (roi.at<unsigned char>(c, r) - black) * (roi.at<unsigned char>(c, r) - black);
                        nw_b++;
                    } else if (labels.at<int>(j) == 1) {
                        vari += (roi.at<unsigned char>(c, r) - white) * (roi.at<unsigned char>(c, r) - white);
                        nw_w++;
                    } else {
                        std::cout << "something went wrong" << std::endl;
                    }
                }
            }
        }

        if (j == 13) {
            Sww += 2.5 * vari;
        } else if (j == 14) {
            Swb += 2.5 * vari;
        } else if (labels.at<int>(j) == 0) {
            Swb += vari;
        } else if (labels.at<int>(j) == 1) {
            Sww += vari;
        }
    }

    const float Sw = Sww / (static_cast<float>(nw_w)) + Swb / (static_cast<float>(nw_b));

    if (Sw == 0) {
        std::cout << "Sw=0 - what now?" << std::endl;
    }

    // calculate inter variance (Score between - Sb)
    // inter variance = sum((mu - mi)^2)/n

    float mb = 0;
    int nb   = 0;
    float mw = 0;
    int nw   = 0;

    for (int j = 0; j < 15; j++) {
        if (labels.at<int>(j) == 1) {
            mw += means.at<float>(j);
            nw++;
        } else if (labels.at<int>(j) == 0) {
            mb += means.at<float>(j);
            nb++;
        } else {
            std::cout << "something went wrong" << std::endl;
        }
        //Sb += (means.at<float>(j,0) - muinter) * (means.at<float>(j,0) - muinter);
    }

    //Sb /= 15;
    mb /= static_cast<float>(nb);
    mw /= static_cast<float>(nw);

    float Sb = abs(mb - mw) * abs(mb - mw);

    // how much "black" is covered?

    Mat tagMask(roi.rows, roi.cols, CV_8UC1, Scalar(0));
    // different center!! ellipse stuff!!!
    circle(tagMask, ell.cen, static_cast<int>(size * TRR), Scalar(1), CV_FILLED);

    Mat matMask(roi.rows, roi.cols, CV_8UC1, Scalar(0));
    circle(matMask, Point(x, y), static_cast<int>(size * ORR), Scalar(1), CV_FILLED);

    Mat tagBlack;
    threshold(roi, tagBlack, (black + white) / 2, 255, CV_THRESH_BINARY_INV);
    tagBlack = tagBlack.mul(tagMask);

    Mat matBlack;
    threshold(roi, matBlack, (black + white) / 2, 255, CV_THRESH_BINARY_INV);
    matBlack = matBlack.mul(matMask);

    const float blackratio = (static_cast<float>(countNonZero(matBlack))) / (static_cast<float>(countNonZero(tagBlack)));
    Sb *= blackratio;

    // use fisher score Sb/Sw
    return Sb / Sw;
}

// ======

std::vector<Point> Grid::renderScaledGridCell(unsigned short cell, double scale, int offset) const {
    // TODO caching???
	std::vector<Point> cont;


    // Outer cells
    if (cell < 12) {
        const double outerInnerRadiusDiff = ORR * size - IORR * size;
        const double outerCircleRadius    = IORR * size + outerInnerRadiusDiff * 0.5 + (outerInnerRadiusDiff * 0.5 * scale);
        const double innerCircleRadius    = IORR * size + outerInnerRadiusDiff * 0.5 - (outerInnerRadiusDiff * 0.5 * scale);

        const int arcStart = -180 + (cell    ) * 30 + 15 * (1 - scale);
        const int arcEnd   = -180 + (cell + 1) * 30 - 15 * (1 - scale);
        // outer arc
        ellipse2Poly(Point2f(x, y), Size2f(outerCircleRadius, outerCircleRadius), angle, arcStart, arcEnd, 1, cont);
        // inner arc
        std::vector<Point> inner_arc_poly;
        ellipse2Poly(Point2f(x, y), Size2f(innerCircleRadius, innerCircleRadius), angle, arcStart, arcEnd, 1, inner_arc_poly);
        // join outer and inner arc
        cont.insert(cont.end(), inner_arc_poly.rbegin(), inner_arc_poly.rend());
        return cont;
    } else if (cell == 13) {
        // supposed white inner half circle
        ellipse2Poly(Point2f(x, y), Size2f(IRR * size * scale, IRR * size * scale), angle, -180 + offset * 30, offset * 30, 1, cont);
    } else if (cell == 14) {
        //supposed black inner half circle
        ellipse2Poly(Point2f(x, y), Size2f(IRR * size * scale, IRR * size * scale), angle, 180 + offset * 30, offset * 30, 1, cont);
    } else if (cell == 12) {
        // outer (white) border
        const double outerInnerRadiusDiff = TRR * size - ORR * size;
        const double outerCircleRadius    = ORR * size + outerInnerRadiusDiff * 0.5 + (outerInnerRadiusDiff * 0.5 * scale);
        const double innerCircleRadius    = ORR * size + outerInnerRadiusDiff * 0.5 - (outerInnerRadiusDiff * 0.5 * scale);

        std::vector < Point > cont2;
        ellipse2Poly(Point2f(x, y), Size(outerCircleRadius, outerCircleRadius), angle + 90, 0, 360, 1,cont);
        ellipse2Poly(Point2f(x, y), Size(innerCircleRadius, innerCircleRadius), angle + 90, 0, 360, 1,cont2);
        cont.insert(cont.end(), cont2.rbegin(), cont2.rend());
    }

    return cont;
}

std::vector<Point> Grid::renderGridCell(unsigned short cell, int offset) const {
    return renderScaledGridCell(cell, 1, offset);
}

// === operators ===

bool Grid::operator>(Grid &rhs) {
    assert(this->scoringMethod() == rhs.scoringMethod());     // both grids need the same scoring method

    if (this->scoringMethod() == BINARYCOUNT) {
        return this->score() < rhs.score();
    } else {
        return this->score() > rhs.score();
    }
}

bool Grid::operator<(Grid &rhs) {
	return rhs > *this;
}

std::vector<float> Grid::generateEdge(int radius, int width, bool useBinaryImage) const {
    // Uses some kind of super resolution with getMeanAlongLine

    const int outerRadius = width > 1 ? radius + ceil(width / 2) : radius;

    // Using the Bresenham algorithm to generate a circle
    int x   = outerRadius;
    int y   = 0;
    int err = 1 - x;

    std::vector< std::vector<float> > subEdges (8);

    subEdges[0].push_back(getMeanAlongLine(this->x - outerRadius, this->y, this->x, this->y, width, useBinaryImage));
    subEdges[2].push_back(getMeanAlongLine(this->x, this->y - outerRadius, this->x, this->y, width, useBinaryImage));
    subEdges[4].push_back(getMeanAlongLine(this->x + outerRadius, this->y, this->x, this->y, width, useBinaryImage));
    subEdges[6].push_back(getMeanAlongLine(this->x, this->y + outerRadius, this->x, this->y, width, useBinaryImage));

    // Generating the octant in clockwise order
    while (x > y + 1) {
        y++;
        if (err < 0) {
            err += 2 * y + 1;
        } else {
            x--;
            err += 2 * (y - x + 1);
        }

        subEdges[0].push_back(getMeanAlongLine(this->x - x, this->y - y, this->x, this->y, width, useBinaryImage));
        subEdges[1].push_back(getMeanAlongLine(this->x - y, this->y - x, this->x, this->y, width, useBinaryImage));
        subEdges[2].push_back(getMeanAlongLine(this->x + y, this->y - x, this->x, this->y, width, useBinaryImage));
        subEdges[3].push_back(getMeanAlongLine(this->x + x, this->y - y, this->x, this->y, width, useBinaryImage));
        subEdges[4].push_back(getMeanAlongLine(this->x + x, this->y + y, this->x, this->y, width, useBinaryImage));
        subEdges[5].push_back(getMeanAlongLine(this->x + y, this->y + x, this->x, this->y, width, useBinaryImage));
        subEdges[6].push_back(getMeanAlongLine(this->x - y, this->y + x, this->x, this->y, width, useBinaryImage));
        subEdges[7].push_back(getMeanAlongLine(this->x - x, this->y + y, this->x, this->y, width, useBinaryImage));

        // This part is sometimes useful for debugging
        //image.at<unsigned char>(Point(this->x - x, this->y - y)) = 255;
        //image.at<unsigned char>(Point(this->x - y, this->y - x)) = 255;
        //image.at<unsigned char>(Point(this->x + y, this->y - x)) = 255;
        //image.at<unsigned char>(Point(this->x + x, this->y - y)) = 255;
        //image.at<unsigned char>(Point(this->x + x, this->y + y)) = 255;
        //image.at<unsigned char>(Point(this->x + y, this->y + x)) = 255;
        //image.at<unsigned char>(Point(this->x - y, this->y + x)) = 255;
        //image.at<unsigned char>(Point(this->x - x, this->y + y)) = 255;
    }

    // Merge all subedges
    std::vector<float> mergedEdges;
    for (size_t i = 0; i < 8; i++) {
    	const std::vector<float>& subEdge = subEdges[i];
        // Reverse some octant to get the wished order (some octant has anti-clockwise order)
        if (i % 2 == 1) {
            mergedEdges.insert(mergedEdges.end(), subEdge.crbegin(), subEdge.crend());
        }
        else {
        	mergedEdges.insert(mergedEdges.end(), subEdge.cbegin(), subEdge.cend());
        }
    }

    // Move the beginning of the Edge, depending on the angle
    const int a        = static_cast<int>(angle + 360) % 360;
    const size_t firstIdx = static_cast<size_t>(a / 360.0 * mergedEdges.size()) % mergedEdges.size();
    std::rotate(mergedEdges.begin(), mergedEdges.begin() + firstIdx, mergedEdges.end());
    return mergedEdges;
}

Mat Grid::generateEdgeAsMat(int radius, int width, bool useBinaryImage) const {
	const std::vector<float> &edge = generateEdge(radius, width, useBinaryImage);

    Mat newEdge(edge.size(), 1, CV_32FC1);
    for (size_t i = 0; i < edge.size(); i++) {
        newEdge.at<float>(i) = edge[i];
    }

    return newEdge;
}

float Grid::getMeanAlongLine(int xStart, int yStart, int xEnd, int yEnd, int size, bool useBinaryImage) const {
    // It's possibly better just to get the position of the pixel along the line, but lets fuck performance
    const Mat &image = useBinaryImage ? ell.binarizedImage : ell.transformedImage;

    Mat profile (size, 1, CV_8UC1);
    int x = xStart;
    int y = yStart;
    // Distances according to axis
    const int dx =  abs(xEnd - xStart);
    const int dy = -abs(yEnd - yStart);
    // Step size for the directions (because it's a line)
    const int sx = xStart < xEnd ? 1 : -1;
    const int sy = yStart < yEnd ? 1 : -1;
    // Error to determine the next pixel to step on
    int err = dx + dy;

    for (int i = 0; i < size; i++) {
        profile.at<unsigned char>(i) = image.at<unsigned char>(Point(x, y));

        const int e2 = 2 * err;
        if (e2 > dy) {
            err += dy;
            x   += sx;
        }
        if (e2 < dx) {
            err += dx;
            y   += sy;
        }
    }

    Scalar mean;
    Scalar std;
    meanStdDev(profile, mean, std);

    return mean[0];
}
// ======

// ======= DEBUG METHODS ========
Mat Grid::drawGrid(double scale, bool useBinaryImage) const {
    Mat draw;     // Matrix the image will be drawn into
    const Mat &roi = useBinaryImage ? ell.binarizedImage : ell.transformedImage;
    roi.copyTo(draw);

    if (roi.type() == CV_8UC1) {
        // the grid is drawn with several colors so a RGB image is needed
        cvtColor(draw, draw, CV_GRAY2BGR);
    }

    // contour vector
    std::vector< std::vector <Point> > conts;

    int ites = 16;
    std::vector < Point > cont;

    // render half of the inner circle (circular matrix design)
    ellipse2Poly(Point2f(x, y), Size2f(IRR * size, IRR * size), angle, 0, -180, 1, cont);
    std::vector < Point > cont2;

    // take first and last vertex of the polygon to get the respective diameter of the inner circle
    cont2.push_back(cont[0]);
    cont2.push_back(cont[cont.size() - 1]);
    conts.push_back(cont2);
    drawContours(draw, conts, 0, Scalar(255, 0, 0), 1);

    conts.clear();

    for (int i = 0; i < ites; i++) {
        conts.push_back(renderScaledGridCell(i, scale, 0));
    }

    drawContours(draw, conts, -1, Scalar(255, 0, 0), 1);
    drawContours(draw, conts, 0, Scalar(0, 255, 0), 1);

    return draw;
}

Mat Grid::drawGrid() const {
    return drawGrid(1, false);
}

Mat Grid::drawGrid(double scale) const {
    return drawGrid(scale, false);
}

Mat Grid::drawGrid(bool useBinaryImage) const {
    return drawGrid(1, useBinaryImage);
}
}
