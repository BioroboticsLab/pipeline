#include "GridFitter.h"

#include <boost/optional.hpp>

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "datastructure/Tag.h"
#include "datastructure/TagCandidate.h"
#include "util/Util.h"
#include "source/tracking/algorithm/BeesBook/Common/Grid.h"
#include "source/utility/CvHelper.h"
#include "source/utility/util.h"

namespace pipeline {
GridFitter::GridFitter()
{}

void GridFitter::loadSettings(gridfitter_settings_t &&settings)
{
	_settings = std::move(settings);
}

std::vector<Tag> GridFitter::process(std::vector<Tag> &&taglist)
{
	for (Tag& tag : taglist) {
		tag.setValid(false);
		for (TagCandidate& candidate : tag.getCandidates()) {
			std::vector<Grid> grids = fitGrid(tag, candidate);
			if (!grids.empty()) { tag.setValid(true); };
			candidate.setGrids(std::move(grids));
		}
	}

	// remove tags without any fitted grids
	taglist.erase(std::remove_if(taglist.begin(), taglist.end(), [](Tag& tag) { return !tag.isValid(); }), taglist.end());

	return taglist;
}

#define DEBUG_GRIDFITTER

std::vector<Grid> GridFitter::fitGrid(const Tag& tag, const TagCandidate &candidate)
{
	const Ellipse& ellipse_orig = candidate.getEllipse();
	const cv::Point2i cen       = ellipse_orig.getCen();
	const double theta          = ellipse_orig.getAngle();
	const cv::Size2i scale      = ellipse_orig.getAxis();

	double min_error = std::numeric_limits<double>::max();
	boost::optional<PipelineGrid> bestGrid;

    const cv::Size2i roiSize = tag.getBox().size();
    cv::Mat roi;
    // TODO: shouldn't be BGR in the first place
    cv::cvtColor(tag.getOrigSubImage(), roi, CV_BGR2GRAY);

    cv::Mat binarizedROI(roiSize, CV_8UC1);
    cv::adaptiveThreshold(roi, binarizedROI, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 21, 3);

//    cv::Mat binarizedROI;
//    cv::adaptiveThreshold(roi, binarizedROI, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 21, 3);

	static const auto pos_offsets   = util::linspace<int>(-2, 2, 5);
	static const auto scale_offsets = util::linspace<int>(-2, 2, 5);
	static const auto theta_offsets = util::linspace<double>(-0.2, 0.2, 5);
	static const auto rotations     = util::linspace<double>(0, 2 * CV_PI, 32);

	static const double alpha_inner    = 4.0;
	static const double alpha_outer    = 2.0;
	static const double alpha_variance = 1.0;

    auto evaluateCandidate = [&](PipelineGrid grid) {
        double error = 0;

        const cv::Rect boundingBox = grid.getBoundingBox();

        const cv::Point2i gridCenter = grid.getCenter();
        // temporarily shift grid position so that top left corner is at
        // position (0, 0) of boundingBox
        // TODO: maybe it's better to adjust get*Coordinates functions so that
        // they take the roi location into account
        grid.setCenter(gridCenter - boundingBox.tl());

        // define region of interest based on bounding box of grid candidate
        cv::Mat subROI;
        subROI = roi(boundingBox);
        const cv::Size subROIsize = subROI.size();

        cv::Mat subBinarized;
        subBinarized = binarizedROI(boundingBox);

        // get coordinates of current grid configuration
        const cv::Mat& outerRingCoordinates  = grid.getOuterRingCoordinates(subROIsize);
        const cv::Mat& innerWhiteCoordinates = grid.getInnerWhiteRingCoordinates(subROIsize);
        const cv::Mat& innerBlackCoordinates = grid.getInnerBlackRingCoordinates(subROIsize);
        const std::vector<cv::Mat>& cellCoordinates = grid.getGridCellCoordinates(subROIsize);

//        cv::Mat errorMat(roiSize, CV_8UC1, cv::Scalar::all(128));

        // compare outer circle
        for (size_t idx = 0; idx < outerRingCoordinates.total(); ++idx) {
            const cv::Point2i coords = outerRingCoordinates.at<cv::Point2i>(idx);
            const uint8_t pixel = subBinarized.at<uint8_t>(coords);
            assert(pixel == 0 || pixel == 255);
            // no branching for better performance
            error += alpha_outer * (255 - pixel);
//            errorMat.at<uint8_t>(coords) = pixel;
        }

        // compare inner white semicircle
        for (size_t idx = 0; idx < innerWhiteCoordinates.total(); ++idx) {
            const cv::Point2i coords = innerWhiteCoordinates.at<cv::Point2i>(idx);
            const uint8_t pixel = subBinarized.at<uint8_t>(coords);
            assert(pixel == 0 || pixel == 255);
            // no branching for better performance
            error += alpha_inner * (255 - pixel);
//            errorMat.at<uint8_t>(coords) = pixel;
        }

        // compare inner black semicircle
        for (size_t idx = 0; idx < innerBlackCoordinates.total(); ++idx) {
            const cv::Point2i coords = innerBlackCoordinates.at<cv::Point2i>(idx);
            const uint8_t pixel = subBinarized.at<uint8_t>(coords);
            assert(pixel == 0 || pixel == 255);
            // no branching for better performance
            error += alpha_inner * pixel;
//            errorMat.at<uint8_t>(coords) = 255 - pixel;
        }

        error /= 255.;

        // add variance of each grid cells to error
        for (cv::Mat const& cell : cellCoordinates) {
            size_t sum = 0;
            for (size_t idx = 0; idx < cell.total(); ++idx) {
                const cv::Point2i coords = cell.at<cv::Point2i>(idx);
                const uint8_t pixel = subBinarized.at<uint8_t>(coords);
                sum += pixel;
            }
            const double mean = static_cast<double>(sum) / static_cast<double>(cell.total());

            double sum2 = 0;
            for (size_t idx = 0; idx < cell.total(); ++idx) {
                const cv::Point2i coords = cell.at<cv::Point2i>(idx);
                const uint8_t pixel = subBinarized.at<uint8_t>(coords);
                const double val = static_cast<double>(pixel);
                sum2 += (val - mean) * (val - mean);
            }

            const double variance = (sum2 / (cell.total() - 1)) / 255.;

//            for (size_t idx = 0; idx < cell.total(); ++idx) {
//                const cv::Point2i coords = cell.at<cv::Point2i>(idx);
//                errorMat.at<uint8_t>(coords) = 255 - static_cast<uint8_t>(variance);
//            }

            error += alpha_variance * variance;

//            // online variance calculation
//            // see: http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Online_algorithm
//            double n = 0;
//            double mean = 0;
//            double m2 = 0;
//            for (size_t idx = 0; idx < cell.total(); ++idx) {
//                const cv::Point2i coords = cell.at<cv::Point2i>(idx);
//                const uint8_t value = subBinarized.at<uint8_t>(coords);

//                ++n;
//                const double delta = (value / 255.) - mean;
//                mean += (delta / n);
//                m2 += delta * (value - mean);
//            }

//            const double var = m2 / (n - 1);
//            error += var;
        }

//        std::cout << "error: " << error << std::endl;


//        cv::cvtColor(errorMat, errorMat, CV_GRAY2BGR);
//        cv::namedWindow("test");
//        cv::imshow("test", errorMat);

//        cv::cvtColor(subBinarized, subBinarized, CV_GRAY2BGR);
//        cv::namedWindow("test2");
//        cv::imshow("test2", subBinarized);

//        cv::namedWindow("test3");
//        cv::imshow("test3", roi);

//        cv::namedWindow("test4");
//        cv::imshow("test4", grid.getProjectedImage(roiSize));
//        cv::waitKey();


//        const cv::Mat firstCandidateImg  = grid.getProjectedImage(maskedROI.size());
//        cv::Mat firstCandidateMask = grid.getInnerCircleMask(maskedROI.size());

//        cv::Mat firstCandidateMasked(firstCandidateImg.size(), firstCandidateImg.type());
//        firstCandidateMasked.setTo(cv::Scalar(0));
//        firstCandidateImg.copyTo(firstCandidateMasked, firstCandidateMask);

//        cv::Mat firstCandidateMaskGray;
//        cv::cvtColor(firstCandidateMask, firstCandidateMaskGray, CV_BGR2GRAY);

//        cv::Mat subBinarizedmasked2;
//        subBinarizedmasked2.setTo(cv::Scalar(0));
//        subBinarized.copyTo(subBinarizedmasked2, firstCandidateMaskGray);
//        subBinarized.copyTo(subBinarizedmasked, ellipse.getMask());

//        cv::cvtColor(firstCandidateMasked, firstCandidateMasked, CV_BGR2GRAY);

//        cv::Mat firstDiff;
//        cv::absdiff(subBinarizedmasked2, firstCandidateMasked, firstDiff);

//        const double firstError  = cv::sum(firstDiff)[0] / static_cast<double>(firstDiff.rows * firstDiff.cols);

        if (error < min_error) {
            min_error = error;
#ifdef DEBUG_GRIDFITTER2
            cv::cvtColor(subBinarizedmasked2, subBinarizedmasked2, CV_GRAY2BGR);
            cv::cvtColor(subBinarizedmasked, subBinarizedmasked, CV_GRAY2BGR);
            cv::cvtColor(firstDiff, firstDiff, CV_GRAY2BGR);
            cv::cvtColor(firstCandidateMasked, firstCandidateMasked, CV_GRAY2BGR);

            cv::Mat blended;
            cv::addWeighted(firstCandidateImg, 0.2, maskedROI, 0.8, 0.0, blended);

            images = std::vector<cv::Mat>({maskedROI, subBinarizedmasked, firstCandidateImg, firstCandidateMask, firstCandidateMasked, subBinarizedmasked2, firstDiff, blended});
#endif

            grid.setCenter(gridCenter);
            bestGrid = std::move(grid);
        }
	};

	auto evaluate = [&](const int offset_x, const int offset_y, const double rotation,
			const int offset_major, const int offset_minor, const double offset_theta) {
		// TODO: copying ellipse in every iteration is costly
		Ellipse ellipse(ellipse_orig);
		ellipse.setCen(cv::Point2i(cen.x + offset_x, cen.y + offset_y));
		ellipse.setAngle(theta + offset_theta);

		if (scale.width + offset_major > scale.height + offset_minor) {
				ellipse.setAxis(cv::Size2i(scale.width + offset_major, scale.height + offset_minor));
		} else {
			return;
		}

//        cv::Mat maskedROI(roiSize, tag.getOrigSubImage().type(), cv::Scalar::all(0));
//        tag.getOrigSubImage().copyTo(maskedROI, ellipse.getMask());

//        cv::Mat convertedROI(roiSize, CV_8UC1);
//        // TODO: shouldn't be BGR in the first place
//        cv::cvtColor(maskedROI, convertedROI, CV_BGR2GRAY);

//        cv::Mat binarizedROI;
//        cv::adaptiveThreshold(convertedROI, binarizedROI, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 21, 3);

//        cv::Mat binarizedROImasked(roiSize, binarizedROI.type());
//        binarizedROImasked.setTo(cv::Scalar(0));
//        binarizedROI.copyTo(binarizedROImasked, ellipse.getMask());

        std::pair<PipelineGrid, PipelineGrid> gridCandidates = Util::gridCandidatesFromEllipse(ellipse, rotation);

//        auto& coordinatesvec = gridCandidates.first.getGridCellCoordinates(roiSize);
//        for (size_t i = 0; i < Grid::NUM_MIDDLE_CELLS; ++i) {
//            cv::namedWindow("test1");
//            cv::Mat imgTest(roiSize, CV_8UC3, cv::Scalar(0,0,0));
//            auto& coordinates = coordinatesvec[i];
//            for (size_t idx = 0; idx < coordinates.total(); ++idx) {
//                const cv::Point2i pt = coordinates.at<cv::Point2i>(idx);
//                imgTest.at<cv::Vec<uint8_t, 3>>(pt.y, pt.x, 0) = cv::Vec<uint8_t, 3>(255, 255, 255);
//            }
//            cv::imshow("test1", imgTest);
//            cv::waitKey();
//        }

        evaluateCandidate(std::move(gridCandidates.first));
        evaluateCandidate(std::move(gridCandidates.second));
    };

	for (const double rotation : rotations) {
		for (const int offset_x : pos_offsets) {
			for (const int offset_y : pos_offsets) {
				for (const double offset_theta : theta_offsets) {
					for (const int offset_major : scale_offsets) {
						for (const int offset_minor : scale_offsets) {
							evaluate(offset_x, offset_y, rotation,
									 offset_major, offset_minor, offset_theta);
						}
					}
				}
			}
		}
	}

    std::cout << "min error: " << min_error << std::endl;

#ifdef DEBUG_GRIDFITTER
    std::vector<cv::Mat> images;
    images.push_back(tag.getOrigSubImage());
    cv::Mat binarizedROIBGR;
//    cv::cvtColor(binarizedROI, binarizedROIBGR, CV_GRAY2BGR);
//    images.push_back(binarizedROIBGR);
    images.push_back((*bestGrid).getProjectedImage(roiSize));

    cv::Mat blended;
    cv::addWeighted(tag.getOrigSubImage(), 0.8, (*bestGrid).getProjectedImage(roiSize), 0.2, 0.0, blended);

    images.push_back(blended);

    cv::Mat outerCircle(roi.size().width, roi.size().height, CV_8UC1, cv::Scalar::all(128));
    auto const coords = (*bestGrid).getOuterRingCoordinates(roi.size());
    for (size_t idx = 0; idx < coords.total(); ++idx) {
        const cv::Point2i coord = coords.at<cv::Point2i>(idx);
        outerCircle.at<uint8_t>(coord) = 255;
    }
    auto const coords2 = (*bestGrid).getInnerWhiteRingCoordinates(roi.size());
    for (size_t idx = 0; idx < coords2.total(); ++idx) {
        const cv::Point2i coord = coords2.at<cv::Point2i>(idx);
        outerCircle.at<uint8_t>(coord) = 255;
    }
    auto const coords3 = (*bestGrid).getInnerBlackRingCoordinates(roi.size());
    for (size_t idx = 0; idx < coords3.total(); ++idx) {
        const cv::Point2i coord = coords3.at<cv::Point2i>(idx);
        outerCircle.at<uint8_t>(coord) = 0;
    }
    cv::cvtColor(outerCircle, outerCircle, CV_GRAY2BGR);

    images.push_back(outerCircle);

    const auto canvas = CvHelper::makeCanvas(images, images[0].rows + 10, 1);

	std::string title("best fit (error: " + std::to_string(min_error) + ")");
	cv::namedWindow(title);
	cv::imshow(title, canvas);
#endif

	cv::waitKey();

	// TODO
	return std::vector<Grid>();
}

}
