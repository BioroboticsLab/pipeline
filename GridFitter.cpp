/*
 * gridFitter.cpp
 *
 *  Created on: 05.05.2014
 *	  Author: mareikeziese
 */

#include "GridFitter.h"

using namespace std;
using namespace cv;

namespace decoder {

GridFitter::GridFitter(Grid::ScoringMethod scoringMethod) {

	this->scoringMethod = scoringMethod;
}

GridFitter::~GridFitter() {
	// TODO Auto-generated destructor stub
}

vector<Tag> GridFitter::process(vector<Tag> const& taglist) {
	vector <Tag> editedTags  = vector <Tag>();
    for (size_t k = 0; k < taglist.size(); k++) {
		Tag tag = taglist[k];

		if (tag.isValid()) {
			Vector<TagCandidate> candidates = tag.getCandidates();

			//iterate over the candidates of the tag
			for (unsigned int i = 0; i < candidates.size(); i++) {
				vector<Grid> grids;
				TagCandidate candidate = candidates[i];
				Grid grid = this->fitGrid(candidate.getEllipse());
				grids.push_back(grid);

				// Rotation by half cell (in both directions), because in some cases it's all you need to get a correct decoding
				grids.push_back(
						Grid(grid.size, grid.angle + 15, 0, grid.x, grid.y,
								grid.ell, true, scoringMethod));
				grids.push_back(
						Grid(grid.size, grid.angle - 15, 0, grid.x, grid.y,
								grid.ell, true, scoringMethod));
				candidate.setGrids(grids);

			}
			 editedTags.push_back(tag);
		}

	}


    		return editedTags;
}

Grid GridFitter::fitGrid(Ellipse ellipse) {
	// Convert image to gray scale (maybe obsolete)
	Mat grayImage;
	if(ellipse.transformedImage.channels()> 2){
}


	// Binarize image first (just for new Scoring)
	//threshold(ellipse.transformedImage, ellipse.binarizedImage, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	//adaptiveThreshold(ellipse.transformedImage, ellipse.binarizedImage, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 13, 5);
	adaptiveThreshold(ellipse.transformedImage, ellipse.binarizedImage, 255,
			ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 21, 3);

	// Get ellipse orientation
	vector<Point2f> orient = getOrientationVector(ellipse);
	Vec2f v = Vec2f(orient[1].x - orient[0].x, orient[1].y - orient[0].y);
	float alph = atan2(v[1], v[0]) * 180 / CV_PI + 90;

	//check for NaN values
	if (alph != alph) {
		alph = 0;
	}

	// Run multiple Grid Fittings with several start positions
    Grid bestGrid = fitGridGradient(ellipse, static_cast<double>(alph), ellipse.cen.x,
			ellipse.cen.y);
	int offsetX, offsetY;
	srand(time(NULL)); // Seed the random generator
	// TODO 16 ist besser als 4
	for (int i = 0; i < 16; i++) {
		// Calculate offset to the center of the ellipse
		offsetX = rand() % ellipse.axis.width - (ellipse.axis.width / 2);
		offsetY = rand() % ellipse.axis.width - (ellipse.axis.width / 2);
        Grid grid = fitGridGradient(ellipse, static_cast<double>(alph),
				ellipse.cen.x + offsetX, ellipse.cen.y + offsetY);

		if (grid > bestGrid) {
			bestGrid = grid;
		}
	}

	return bestGrid;
}

vector<Point2f> GridFitter::getOrientationVector(Ellipse &ellipse) {

	vector<Point2f> res(2, Point());

	Point3f circle = Point3f(ellipse.cen.x, ellipse.cen.y,
			(ellipse.axis.width / 3.0));
	Mat &roi = ellipse.binarizedImage;

	// create circular cutout
	Mat circMask = Mat(roi.rows, roi.cols, CV_8UC1, Scalar(0));
	cv::circle(circMask, Point(circle.x, circle.y), circle.z, Scalar(1),
			CV_FILLED);

	// apply otsu binarization to this part
	//double otsut = getOtsuThreshold(roi);

	Mat hcWhite = roi.mul(circMask);

	Mat hcBlack = circMask.mul(255 - hcWhite);


	// Calculate moment => orientation of the tag
	Moments momw = moments(hcWhite, true);
	Moments momb = moments(hcBlack, true);

	res[0].x = momb.m10 / momb.m00;
	res[0].y = momb.m01 / momb.m00;

	res[1].x = momw.m10 / momw.m00;
	res[1].y = momw.m01 / momw.m00;

	return (res);
}

double GridFitter::getOtsuThreshold(Mat &srcMat) {
	//Code Snippet from
	//http://stackoverflow.com/questions/12953993/otsu-thresholding-for-depth-image
	Mat copyImg;
	srcMat.copyTo(copyImg);
	uchar* ptr = copyImg.datastart;
	uchar* ptr_end = copyImg.dataend;
	while (ptr < ptr_end) {
		if (*ptr == 0) { // swap if zero
			uchar tmp = *ptr_end;
			*ptr_end = *ptr;
			*ptr = tmp;
			ptr_end--; // make array smaller
		} else {
			ptr++;
		}
	}

	// make a new matrix with only valid data
	Mat nz = Mat(vector<uchar>(copyImg.datastart, ptr_end), true);

	// compute  Otsu threshold
	double thresh = threshold(nz, nz, 0, 255,
			CV_THRESH_BINARY | CV_THRESH_OTSU);

	return (thresh);
}

Grid GridFitter::fitGridGradient(Ellipse &ellipse, double angle, int startX,
		int startY) {

	int step_size = INITIAL_STEP_SIZE; // Amount of pixel the walk should jump

	float gsize = (ellipse.axis.width / 3.0);
	int x = startX;
	int y = startY;
	// best grid so far
	Grid best = fitGridAngle(ellipse, gsize, angle, x, y);
	int gs = cvRound(ellipse.axis.width / 3.0);

	while (step_size > FINAL_STEP_SIZE) {
		// investigate the surrounding positions
		vector<Grid> grids;

		// right
		if (sqrt(
                static_cast<float> ((ellipse.cen.x - (x + step_size))
						* (ellipse.cen.x - (x + step_size))
						+ (ellipse.cen.y - y) * (ellipse.cen.y - y))) <= gs) {
			grids.push_back(
					fitGridAngle(ellipse, gsize, angle, x + step_size, y));
		}
		// left
		if (sqrt(
                static_cast<float> ((ellipse.cen.x - (x - step_size))
						* (ellipse.cen.x - (x - step_size))
						+ (ellipse.cen.y - y) * (ellipse.cen.y - y))) <= gs) {
			grids.push_back(
					fitGridAngle(ellipse, gsize, angle, x - step_size, y));
		}
		// down
		if (sqrt(
                static_cast<float> ((ellipse.cen.x - x) * (ellipse.cen.x - x)
						+ (ellipse.cen.y - (y + step_size))
								* (ellipse.cen.y - (y + step_size)))) <= gs) {
			grids.push_back(
					fitGridAngle(ellipse, gsize, angle, x, y + step_size));
		}
		// up
		if (sqrt(
                static_cast<float> ((ellipse.cen.x - x) * (ellipse.cen.x - x)
						+ (ellipse.cen.y - (y - step_size))
								* (ellipse.cen.y - (y - step_size)))) <= gs) {
			grids.push_back(
					fitGridAngle(ellipse, gsize, angle, x, y - step_size));
		}

		Grid best_neighbor = getBestGrid(grids);

		// the < is for the new score
		if (best_neighbor > best) {
			best = best_neighbor;
            step_size = static_cast<int>(ceil(step_size * UP_SPEED));
		} else {
            step_size = static_cast<int>(step_size * DOWN_SPEED);
		}
	}

	return (best);
}

Grid GridFitter::fitGridAngle(Ellipse &ellipse, float gsize, double angle,
		int x, int y) {
	Grid cur(scoringMethod);
	Grid best(gsize, scoringMethod);

	int step_size = 3;
	int a = angle;

	//for (int i = 0; i < 30; i++) {
	//cur = Grid(gsize, a + i, 0, x, y, ellipse, scoringMethod);

	//if (cur > best) {
	//best = cur;
	//}
	//}

	// Similar approach like in fitGridGradient, just using the angle
	while (step_size > 0) {
		Grid g1 = Grid(gsize, a + step_size, 0, x, y, ellipse, scoringMethod);
		Grid g2 = Grid(gsize, a - step_size, 0, x, y, ellipse, scoringMethod);

		int new_a;
		if (g1 > g2) {
			cur = g1;
			new_a = a + step_size;
		} else {
			cur = g2;
			new_a = a - step_size;
		}

		if (cur > best) {
			best = cur;
			step_size *= 3;
			a = new_a;
		} else {
			step_size *= 0.5;
		}
	}

	return best;
}

Grid GridFitter::getBestGrid(vector<Grid> grids) {

	Grid best(scoringMethod);
	Grid cur(scoringMethod);

	while (grids.size() > 0) {
		cur = grids.back();
		grids.pop_back();
		if (cur > best) {
			best = cur;
		}
	}

	return best;
}

int GridFitter::bestGridAngleCorrection(Grid g) {

	// index encoding 30Â°-step angles ranging from [0,5]
	int i = 0;
	Mat &roi = g.ell.transformedImage;

	float mean1c = 0;
	float mean2c = 0;

	for (int j = 0; j < 6; j++) {
		Mat mask1 = Mat(roi.rows, roi.cols, roi.type(), Scalar(0));
		vector<vector<Point> > conts1;
		conts1.push_back(g.renderGridCell(13, j));
		drawContours(mask1, conts1, 0, Scalar(255), CV_FILLED);
		Scalar mean1;
		Scalar std1;

		meanStdDev(roi, mean1, std1, mask1);

		Mat mask2 = Mat(roi.rows, roi.cols, roi.type(), Scalar(0));
		vector<vector<Point> > conts2;
		conts2.push_back(g.renderGridCell(14, j));
		drawContours(mask2, conts2, 0, Scalar(255), CV_FILLED);
		Scalar mean2;
		Scalar std2;

		meanStdDev(roi, mean2, std2, mask2);

		if (abs(mean1c - mean2c) < abs(mean1[0] - mean2[0])) {
			mean1c = mean1[0];
			mean2c = mean2[0];
			i = j;
		}
	}

	// 180-flip if supposed white half circle is darker than supposed black half circle
	if (mean1c < mean2c)
		i += 6;

	return (i);
}

}
