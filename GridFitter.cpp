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

//    cv::Mat binarizedROI;
//    cv::adaptiveThreshold(roi, binarizedROI, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 21, 3);

	// TODO: use compile-time evaluation
	static const auto pos_offsets   = util::linspace<int>(-2, 2, 1);
	static const auto scale_offsets = util::linspace<int>(-1, 1, 1);
	static const auto theta_offsets = util::linspace<double>(-0.3, 0.3, 0.1);
	static const auto rotations     = util::linspace<double>(0., CV_PI, (CV_PI) / 64.);

    auto evaluateCandidate = [&](PipelineGrid& grid, Ellipse const& ellipse) {
        double error = 0;

        // get coordinates of current grid configuration
        const cv::Mat& outerRingCoordinates  = grid.getOuterRingCoordinates(roiSize);
        const cv::Mat& innerWhiteCoordinates = grid.getInnerWhiteRingCoordinates(roiSize);
        const cv::Mat& innerBlackCoordinates = grid.getInnerBlackRingCoordinates(roiSize);
        const std::vector<cv::Mat>& cellCoordinates = grid.getGridCellCoordinates(roiSize);

        cv::Mat maskedROI;
        roi.copyTo(maskedROI, grid.getOuterRingMask(roiSize));

        cv::Mat binarizedROI;
        cv::adaptiveThreshold(maskedROI, binarizedROI, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 21, 3);

        cv::cvtColor(binarizedROI, binarizedROI, CV_GRAY2BGR);
        cv::namedWindow("test");
        cv::imshow("test", binarizedROI);
        cv::waitKey();

        // compare outer circle
        for (size_t idx = 0; idx < outerRingCoordinates.total(); ++idx) {
            const cv::Point2i coords = outerRingCoordinates.at<cv::Point2i>(idx);
            const uint8_t pixel = binarizedROI.at<uint8_t>(coords);
            assert(pixel == 0 || pixel == 255);
            // no branching for better performance
            error += (255 - pixel);
        }

        // compare inner white semicircle
        for (size_t idx = 0; idx < innerWhiteCoordinates.total(); ++idx) {
            const cv::Point2i coords = innerWhiteCoordinates.at<cv::Point2i>(idx);
            const uint8_t pixel = binarizedROI.at<uint8_t>(coords);
            assert(pixel == 0 || pixel == 255);
            // no branching for better performance
            error += (255 - pixel);
        }

        // compare inner black semicircle
        for (size_t idx = 0; idx < innerBlackCoordinates.total(); ++idx) {
            const cv::Point2i coords = innerBlackCoordinates.at<cv::Point2i>(idx);
            const uint8_t pixel = binarizedROI.at<uint8_t>(coords);
            assert(pixel == 0 || pixel == 255);
            // no branching for better performance
            error += pixel;
        }

        error /= 255.;



//        const cv::Mat firstCandidateImg  = grid.getProjectedImage(maskedROI.size());
//        cv::Mat firstCandidateMask = grid.getInnerCircleMask(maskedROI.size());

//        cv::Mat firstCandidateMasked(firstCandidateImg.size(), firstCandidateImg.type());
//        firstCandidateMasked.setTo(cv::Scalar(0));
//        firstCandidateImg.copyTo(firstCandidateMasked, firstCandidateMask);

//        cv::Mat firstCandidateMaskGray;
//        cv::cvtColor(firstCandidateMask, firstCandidateMaskGray, CV_BGR2GRAY);

//        cv::Mat binarizedROImasked2;
//        binarizedROImasked2.setTo(cv::Scalar(0));
//        binarizedROI.copyTo(binarizedROImasked2, firstCandidateMaskGray);
//        binarizedROI.copyTo(binarizedROImasked, ellipse.getMask());

//        cv::cvtColor(firstCandidateMasked, firstCandidateMasked, CV_BGR2GRAY);

//        cv::Mat firstDiff;
//        cv::absdiff(binarizedROImasked2, firstCandidateMasked, firstDiff);

//        const double firstError  = cv::sum(firstDiff)[0] / static_cast<double>(firstDiff.rows * firstDiff.cols);
        const double firstError = error;

        if (firstError < min_error) {
            min_error = firstError;
            bestGrid = grid;

#ifdef DEBUG_GRIDFITTER2
            cv::cvtColor(binarizedROImasked2, binarizedROImasked2, CV_GRAY2BGR);
            cv::cvtColor(binarizedROImasked, binarizedROImasked, CV_GRAY2BGR);
            cv::cvtColor(firstDiff, firstDiff, CV_GRAY2BGR);
            cv::cvtColor(firstCandidateMasked, firstCandidateMasked, CV_GRAY2BGR);

            cv::Mat blended;
            cv::addWeighted(firstCandidateImg, 0.2, maskedROI, 0.8, 0.0, blended);

            images = std::vector<cv::Mat>({maskedROI, binarizedROImasked, firstCandidateImg, firstCandidateMask, firstCandidateMasked, binarizedROImasked2, firstDiff, blended});
#endif
        }

		std::cout << "first error: " << firstError << std::endl;
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

        evaluateCandidate(gridCandidates.first, ellipse);
        evaluateCandidate(gridCandidates.second, ellipse);
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
