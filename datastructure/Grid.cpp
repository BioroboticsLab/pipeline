#include "Grid.h"
#include <stdexcept>          // std::invalid_argument
#include <iostream>           // std::cout
#include <algorithm>          // std::rotate
#include <opencv2/opencv.hpp> // cv::GaussianBlur,  cv::circle, cv::threshold, cv::cvtColor, cv::drawContours

namespace decoder {
// === Constructors and initializer ===
Grid::Grid(float size, float angle, int x, int y, Ellipse ell, ScoringMethod scoringMethod)
	: m_score(scoringMethod)
	, m_size(size)
	, m_x(x)
	, m_y(y)
	, m_angle(angle)
	, m_ell(ell)
{ }

Grid::Grid(ScoringMethod scoringMethod)
	: Grid(0, 0, 0, 0, Ellipse(), scoringMethod)
{
}

Grid::Grid(float size, ScoringMethod scoringMethod)
	: Grid(size, 0, 0, 0, Ellipse(), scoringMethod)
{
}

/**
 * new-angle copy constructor
 */
Grid::Grid(const Grid &rhs, float angle)
	: Grid(rhs)
{
	// angle has changed --> invalidate copied score
	if (angle != m_angle) {
		m_angle = angle;
		m_score.value = scoringMethod() == BINARYCOUNT ? BINARYCOUNT_INIT : FISHER_INIT;
	}
}

Grid::~Grid() = default;

// ===

// === Scoring methods ===
Grid::ScoringMethod Grid::scoringMethod() const {
    return m_score.metric;
}

double Grid::score() const {
	// determine whether the grid is a dummy or not
	if (! m_score.is_initialized() && ! m_ell.getBinarizedImage().empty() ) {
		switch (m_score.metric) {
			case BINARYCOUNT:
				m_score.value = binaryCountScore();
				break;
			case FISHER:
				m_score.value = fisherScore();
				break;
		}
	}
	return m_score.value;
}

/**
 * - total score = sum( score of each area )
 * - score of an area: "#pixel that don't match" / "' pixel that match"
 *
 * --> the smaller the score, the better the match
 *
 * @return score >= 0
 */
double Grid::binaryCountScore() const {

    const cv::Mat &binImg = m_ell.getBinarizedImage();

    cv::Mat scores (3, 1, CV_64FC1);
    cv::Mat mask(binImg.rows, binImg.cols, binImg.type());
    // for each cell calculate its size (cell size) and its mean intensity (means)
    for (int cell_id = 12; cell_id < 15; ++cell_id) {
        mask = cv::Scalar(0);

		renderGridCell(mask, cv::Scalar(1), cell_id);

        const auto num_masked_pixel = cv::countNonZero(mask);       // count polygon pixel (i.e. nonzero pixel)

        // just keep the pixel that are in the binary image and in the polygon
        mask &= binImg; // "cv::Mat whiteCellPixel = binImg.mul(mask);" yields the same result, BUT it is stored on a new matrix
        const auto num_masked_white_pixel = countNonZero(mask);

        const auto num_masked_black_pixel = num_masked_pixel - num_masked_white_pixel;

        const double whitePixelAmount = static_cast<double>(num_masked_white_pixel);
        const double blackPixelAmount = static_cast<double>(num_masked_black_pixel);

        if (cell_id == 12 || cell_id == 13) {
            // white inner half circle or white outer border
            scores.at<double>(14 - cell_id) =  whitePixelAmount == 0. ? blackPixelAmount : blackPixelAmount / whitePixelAmount;
        } else if (cell_id == 14) {
            //supposed black inner half circle
            scores.at<double>(14 - cell_id) =  blackPixelAmount == 0. ? whitePixelAmount : whitePixelAmount / blackPixelAmount;
        }
    }
    return sum(scores)[0];
}

double Grid::fisherScore() const {
    // 37/46 = 80.43% with Sb = |black - white| look into other intervariances
    // 41/47 = 89.13% with kind of A scaling
    // determine best orientation
    const cv::Mat &roi = m_ell.getTransformedImage();

    double black = -1;
    double white = -1;

    cv::Mat smoothROI;
    cv::GaussianBlur(roi, smoothROI, cv::Size(7, 7), 0, 0, cv::BORDER_DEFAULT);
    minMaxIdx(smoothROI, &black, &white);

    cv::Mat means(15, 1, CV_32F);
    cv::Mat labels(15, 1, CV_32S);
    cv::TermCriteria max_it(cv::TermCriteria::COUNT, 3, 2);
    cv::Mat centers;
    std::vector<cv::Mat> masks;

    // for each cell calculate its size (cellsize) and its mean intensity (means)
    for (int cell_id = 0; cell_id < 15; ++cell_id) {
        cv::Mat mask(roi.rows, roi.cols, roi.type(), cv::Scalar(0));
		renderGridCell(mask, cv::Scalar(255), cell_id);
        masks.push_back(mask);

        cv::Scalar mean;
        cv::Scalar std;
        meanStdDev(roi, mean, std, mask);

        means.at<float>(cell_id) = static_cast<float>(mean[0]);
    }

    // assume the color for each cell
    for (int j = 0; j < 15; j++) {
        if (std::abs(black - means.at<float>(j)) < std::abs(white - means.at<float>(j))) {
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

    cv::kmeans(means, 2, labels, max_it, 1, cv::KMEANS_USE_INITIAL_LABELS, centers);

    // tag design 12,13 are white, 14 is black
    labels.at<int>(12) = 1;
    labels.at<int>(13) = 1;
    labels.at<int>(14) = 0;

    black = centers.at<float>(0, 0);
    white = centers.at<float>(1, 0);

    if (m_score.value != -1) {
        for (int lab = 0; lab < labels.rows; lab++) {
            if (labels.at<int>(lab) == 0 && lab < 12) {
                if (std::abs(means.at<float>(lab) - white) < std::abs(means.at<float>(lab) - black)) {
                    std::cout << "eh, uppsb" << std::endl;
                }
            } else if (lab < 12) {
                if (std::abs(means.at<float>(lab) - white) > std::abs(means.at<float>(lab) - black)) {
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
                        vari += static_cast<float>((roi.at<unsigned char>(c, r) - black) * (roi.at<unsigned char>(c, r) - black));
                        nw_b++;
                    } else if (labels.at<int>(j) == 1) {
                        vari += static_cast<float>((roi.at<unsigned char>(c, r) - white) * (roi.at<unsigned char>(c, r) - white));
                        nw_w++;
                    } else {
                        std::cout << "something went wrong" << std::endl;
                    }
                }
            }
        }

        if (j == 13) {
            Sww += 2.5f * vari;
        } else if (j == 14) {
            Swb += 2.5f * vari;
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

    float Sb = std::abs(mb - mw) * std::abs(mb - mw);

    // how much "black" is covered?

    cv::Mat tagMask(roi.rows, roi.cols, CV_8UC1, cv::Scalar(0));
    // different center!! ellipse stuff!!!
    cv::circle(tagMask, m_ell.getCen(), static_cast<int>(m_size * TRR), cv::Scalar(1), CV_FILLED);

    cv::Mat matMask(roi.rows, roi.cols, CV_8UC1, cv::Scalar(0));
	cv::circle(matMask, cv::Point(static_cast<int>(m_x), static_cast<int>(m_y)), static_cast<int>(m_size * ORR), cv::Scalar(1), CV_FILLED);

    cv::Mat tagBlack;
    cv::threshold(roi, tagBlack, (black + white) / 2, 255, CV_THRESH_BINARY_INV);
    tagBlack = tagBlack.mul(tagMask);

    cv::Mat matBlack;
    cv::threshold(roi, matBlack, (black + white) / 2, 255, CV_THRESH_BINARY_INV);
    matBlack = matBlack.mul(matMask);

    const float blackratio = (static_cast<float>(countNonZero(matBlack))) / (static_cast<float>(countNonZero(tagBlack)));
    Sb *= blackratio;

    // use fisher score Sb/Sw
    return Sb / Sw;
}

// ======

const std::vector<std::vector<cv::Point> >& Grid::gridCellScaled2poly(unsigned short cell, float scale, int offset) const {

	//static thread_local std::vector<std::vector<cv::Point>> result(1);
	//static thread_local std::vector<cv::Point> buffer;

	static  std::vector<std::vector<cv::Point> > result(1);
	static  std::vector<cv::Point> buffer;

	const cv::Point2f center(m_x, m_y);
	const int step_size = 1;

    // Outer cells
    if (cell < 12)
    {
        const float outerInnerRadiusDiff = ORR * m_size - IORR * m_size;
        const float outerCircleRadius    = IORR * m_size + outerInnerRadiusDiff * 0.5f + (outerInnerRadiusDiff * 0.5f * scale);
        const float innerCircleRadius    = IORR * m_size + outerInnerRadiusDiff * 0.5f - (outerInnerRadiusDiff * 0.5f * scale);

		const int arcStart = static_cast<int>(-180 + (cell    ) * 30 + 15 * (1 - scale));
		const int arcEnd   = static_cast<int>(-180 + (cell + 1) * 30 - 15 * (1 - scale));

        // outer arc
		cv::ellipse2Poly(center, cv::Size2f(outerCircleRadius, outerCircleRadius), static_cast<int>(m_angle), arcStart, arcEnd, step_size, result[0]);

        // inner arc
		cv::ellipse2Poly(center, cv::Size2f(innerCircleRadius, innerCircleRadius), static_cast<int>(m_angle), arcStart, arcEnd, step_size, buffer);

        // join outer and inner arc
        result[0].insert(result[0].end(), buffer.rbegin(), buffer.rend());
    }
    else if (cell == 13)
    {
    	const float CircleRadius = IRR * m_size * scale;

        const int arcStart = -180 + offset * 30;
        const int arcEnd   =        offset * 30;

        // supposed white inner half circle
		cv::ellipse2Poly(center, cv::Size2f(CircleRadius, CircleRadius), static_cast<int>(m_angle), arcStart, arcEnd, step_size, result[0]);
    }
    else if (cell == 14)
    {
    	const float CircleRadius = IRR * m_size * scale;

        const int arcStart =        offset * 30;
        const int arcEnd   =  180 + offset * 30;

        //supposed black inner half circle
		cv::ellipse2Poly(center, cv::Size2f(CircleRadius, CircleRadius), static_cast<int>(m_angle), arcStart, arcEnd, step_size, result[0]);
    }
    else if (cell == 12)
    {
        // outer (white) border
        const float outerInnerRadiusDiff = TRR * m_size - ORR * m_size;
        const float outerCircleRadius    = ORR * m_size + outerInnerRadiusDiff * 0.5f + (outerInnerRadiusDiff * 0.5f * scale);
        const float innerCircleRadius    = ORR * m_size + outerInnerRadiusDiff * 0.5f - (outerInnerRadiusDiff * 0.5f * scale);

        const int arcStart =   0;
        const int arcEnd   = 360;

		ellipse2Poly(center, cv::Size2f(outerCircleRadius, outerCircleRadius), static_cast<int>(m_angle + 90), arcStart, arcEnd, step_size, result[0]);
		ellipse2Poly(center, cv::Size2f(innerCircleRadius, innerCircleRadius), static_cast<int>(m_angle + 90), arcStart, arcEnd, step_size, buffer);
        result[0].insert(result[0].end(), buffer.rbegin(), buffer.rend());
    }
    else {
    	throw std::invalid_argument("invalid cell id");
    }

    return result;
}


void Grid::renderGridCellScaled(cv::Mat &img, const cv::Scalar &color, unsigned short cell, double scale, int offset) const {

	const auto &polylines = gridCellScaled2poly(cell, static_cast<float>(scale), offset);
	// TODO: try using "cv::fillConvexPoly(img, polylines, color);" for half circles
	cv::fillPoly(img, polylines, color);

}


// === operators ===

bool Grid::operator>(const Grid &rhs) const {
	assert(scoringMethod() == rhs.scoringMethod());     // both grids need the same scoring method

	switch (scoringMethod()) {
		case BINARYCOUNT:
			return score() < rhs.score();
		case FISHER:
			return score() > rhs.score();
		default:
			assert(false);
			return false;
	}
}

bool Grid::operator<(const Grid &rhs) const {
	return rhs > *this;
}

std::vector<float> Grid::generateEdge(int radius, int width, bool useBinaryImage) const {
    // Uses some kind of super resolution with getMeanAlongLine

	const int outerRadius = width > 1 ? static_cast<int>(radius + ceil(width / 2)) : radius;

    // Using the Bresenham algorithm to generate a circle
    int x   = outerRadius;
    int y   = 0;
    int err = 1 - x;

    std::vector< std::vector<float> > subEdges (8);

	const int mx = static_cast<int>(m_x);
	const int my = static_cast<int>(m_y);

	subEdges[0].push_back(getMeanAlongLine(mx - outerRadius, my, mx, my, width, useBinaryImage));
	subEdges[2].push_back(getMeanAlongLine(mx, my - outerRadius, mx, my, width, useBinaryImage));
	subEdges[4].push_back(getMeanAlongLine(mx + outerRadius, my, mx, my, width, useBinaryImage));
	subEdges[6].push_back(getMeanAlongLine(mx, my + outerRadius, mx, my, width, useBinaryImage));

    // Generating the octant in clockwise order
    while (x > y + 1) {
        y++;
        if (err < 0) {
            err += 2 * y + 1;
        } else {
            x--;
            err += 2 * (y - x + 1);
        }

		subEdges[0].push_back(getMeanAlongLine(mx - x, my - y, mx, my, width, useBinaryImage));
		subEdges[1].push_back(getMeanAlongLine(mx - y, my - x, mx, my, width, useBinaryImage));
		subEdges[2].push_back(getMeanAlongLine(mx + y, my - x, mx, my, width, useBinaryImage));
		subEdges[3].push_back(getMeanAlongLine(mx + x, my - y, mx, my, width, useBinaryImage));
		subEdges[4].push_back(getMeanAlongLine(mx + x, my + y, mx, my, width, useBinaryImage));
		subEdges[5].push_back(getMeanAlongLine(mx + y, my + x, mx, my, width, useBinaryImage));
		subEdges[6].push_back(getMeanAlongLine(mx - y, my + x, mx, my, width, useBinaryImage));
		subEdges[7].push_back(getMeanAlongLine(mx - x, my + y, mx, my, width, useBinaryImage));

        // This part is sometimes useful for debugging
		//image.at<unsigned char>(cv::Point(x - x, y - y)) = 255;
		//image.at<unsigned char>(cv::Point(x - y, y - x)) = 255;
		//image.at<unsigned char>(cv::Point(x + y, y - x)) = 255;
		//image.at<unsigned char>(cv::Point(x + x, y - y)) = 255;
		//image.at<unsigned char>(cv::Point(x + x, y + y)) = 255;
		//image.at<unsigned char>(cv::Point(x + y, y + x)) = 255;
		//image.at<unsigned char>(cv::Point(x - y, y + x)) = 255;
		//image.at<unsigned char>(cv::Point(x - x, y + y)) = 255;
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
    const int a        = static_cast<int>(m_angle + 360) % 360;
    const size_t firstIdx = static_cast<size_t>(a / 360.0 * mergedEdges.size()) % mergedEdges.size();
    std::rotate(mergedEdges.begin(), mergedEdges.begin() + firstIdx, mergedEdges.end());
    return mergedEdges;
}

cv::Mat Grid::generateEdgeAsMat(int radius, int width, bool useBinaryImage) const {
	const std::vector<float> &edge = generateEdge(radius, width, useBinaryImage);

    cv::Mat newEdge(edge.size(), 1, CV_32FC1);
    for (size_t i = 0; i < edge.size(); i++) {
        newEdge.at<float>(i) = edge[i];
    }

    return newEdge;
}

float Grid::getMeanAlongLine(int xStart, int yStart, int xEnd, int yEnd, int size, bool useBinaryImage) const {
    // It's possibly better just to get the position of the pixel along the line, but lets fuck performance
    const cv::Mat &image = useBinaryImage ? m_ell.getBinarizedImage() : m_ell.getTransformedImage();

    cv::Mat profile (size, 1, CV_8UC1);
    int x = xStart;
    int y = yStart;
    // Distances according to axis
    const int dx =  std::abs(xEnd - xStart);
    const int dy = -std::abs(yEnd - yStart);
    // Step size for the directions (because it's a line)
    const int sx = xStart < xEnd ? 1 : -1;
    const int sy = yStart < yEnd ? 1 : -1;
    // Error to determine the next pixel to step on
    int err = dx + dy;

    for (int i = 0; i < size; i++) {
        profile.at<unsigned char>(i) = image.at<unsigned char>(cv::Point(x, y));

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

    cv::Scalar mean;
    cv::Scalar std;
    meanStdDev(profile, mean, std);

    return static_cast<float>(mean[0]);
}
// ======

// ======= DEBUG METHODS ========
cv::Mat Grid::drawGrid(float scale, bool useBinaryImage) const {
    cv::Mat draw;     // Matrix the image will be drawn into
    const cv::Mat &roi = useBinaryImage ? m_ell.getBinarizedImage() : m_ell.getTransformedImage();
    roi.copyTo(draw);

    if (roi.type() == CV_8UC1) {
        // the grid is drawn with several colors so a RGB image is needed
        cv::cvtColor(draw, draw, CV_GRAY2BGR);
    }

    // contour vector
    std::vector< std::vector <cv::Point> > conts;

    int ites = 16;
    std::vector < cv::Point > cont;

    // render half of the inner circle (circular matrix design)
	cv::ellipse2Poly(cv::Point2f(m_x, m_y), cv::Size2f(IRR * m_size, IRR * m_size), static_cast<int>(m_angle), 0, -180, 1, cont);
    std::vector < cv::Point > cont2;

    // take first and last vertex of the polygon to get the respective diameter of the inner circle
    cont2.push_back(cont[0]);
    cont2.push_back(cont[cont.size() - 1]);
    conts.push_back(cont2);
    cv::drawContours(draw, conts, 0, cv::Scalar(255, 0, 0), 1);

    conts.clear();

    for (int i = 0; i < ites; i++) {
        conts.push_back(gridCellScaled2poly(i, scale, 0)[0]);
    }

    cv::drawContours(draw, conts, -1, cv::Scalar(255, 0, 0), 1);
    cv::drawContours(draw, conts, 0, cv::Scalar(0, 255, 0), 1);

    return draw;
}

cv::Mat Grid::drawGrid() const {
    return drawGrid(1, false);
}

cv::Mat Grid::drawGrid(float scale) const {
    return drawGrid(scale, false);
}

cv::Mat Grid::drawGrid(bool useBinaryImage) const {
    return drawGrid(1, useBinaryImage);
}

Grid3D Grid::grid2Grid3D(cv::Point cen) const{
    //TODO: double radius_px, double angle_z, double angle_y, double angle_x
    return Grid3D(cen, m_ell.getAxis().height, m_ell.getAngle(), 0, 0);
}
}
