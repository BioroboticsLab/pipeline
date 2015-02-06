#include "GridFitter.h"

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

std::vector<Grid> GridFitter::fitGrid(const Tag& tag, const TagCandidate &candidate)
{
	const Ellipse& ellipse_orig = candidate.getEllipse();
	const cv::Point2i cen = ellipse_orig.getCen();
	const double theta = ellipse_orig.getAngle();
	const cv::Size2i scale = ellipse_orig.getAxis();

	static const double min_rotation = 0.;
	static const double max_rotation = 2 * CV_PI;

	static const size_t num_steps = 64;

	double min_error = std::numeric_limits<double>::max();

	std::vector<cv::Mat> images;

	for (double rotation = min_rotation; rotation <= max_rotation; rotation += (max_rotation / num_steps)) {

		static const std::vector<int> offsets {-3, -2, -1, 0, 1, 2, 3};
		static const std::vector<double> theta_offsets {-0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3};
		static const std::vector<double> scale_offsets {-0.01, 0, 0.01};

        for (const int offset_x : offsets) {
            for (const int offset_y : offsets) {
                for (const double offset_theta : theta_offsets) {
                for (const double offset_scale : scale_offsets) {
                Ellipse ellipse(ellipse_orig);
                ellipse.setCen(cv::Point2i(cen.x + offset_x, cen.y + offset_y));
                ellipse.setAngle(theta + offset_theta);
                ellipse.setAxis(cv::Size2i(static_cast<int>(scale.width + scale.width * offset_scale), static_cast<int>(scale.height + scale.height * offset_scale)));

                cv::Mat maskedROI(tag.getBox().size(), tag.getOrigSubImage().type());
                maskedROI.setTo(cv::Scalar(0));
                tag.getOrigSubImage().copyTo(maskedROI, ellipse.getMask());

                cv::Mat convertedROI(maskedROI.size(), CV_8UC1);
                cv::cvtColor(maskedROI, convertedROI, CV_BGR2GRAY);

                cv::Mat binarizedROI;
                cv::adaptiveThreshold(convertedROI, binarizedROI, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 21, 3);
                //cv::threshold(convertedROI, binarizedROI, 0, 255, CV_THRESH_OTSU);

                cv::Mat binarizedROImasked(binarizedROI.size(), binarizedROI.type());
                binarizedROImasked.setTo(cv::Scalar(0));
                binarizedROI.copyTo(binarizedROImasked, ellipse.getMask());

                const auto gridCandidates = Util::gridCandidatesFromEllipse(ellipse, rotation);
                const std::array<PipelineGrid, 2> candidates { gridCandidates.first, gridCandidates.second };
                for (PipelineGrid const& grid : candidates) {
                    const cv::Mat firstCandidateImg  = grid.getProjectedImage(maskedROI.size());
                    cv::Mat firstCandidateMask = grid.getInnerCircleMask(maskedROI.size());

                    cv::Mat firstCandidateMasked(firstCandidateImg.size(), firstCandidateImg.type());
                    firstCandidateMasked.setTo(cv::Scalar(0));
                    firstCandidateImg.copyTo(firstCandidateMasked, firstCandidateMask);

                    cv::Mat firstCandidateMaskGray;
                    cv::cvtColor(firstCandidateMask, firstCandidateMaskGray, CV_BGR2GRAY);

                    cv::Mat binarizedROImasked2;
                    binarizedROImasked2.setTo(cv::Scalar(0));
                    binarizedROI.copyTo(binarizedROImasked2, firstCandidateMaskGray);
                    binarizedROI.copyTo(binarizedROImasked, ellipse.getMask());

                    cv::cvtColor(firstCandidateMasked, firstCandidateMasked, CV_BGR2GRAY);

                    cv::Mat firstDiff;
                    cv::absdiff(binarizedROImasked2, firstCandidateMasked, firstDiff);

                    const double firstError  = cv::sum(firstDiff)[0] / static_cast<double>(firstDiff.rows * firstDiff.cols);

                    if (firstError < min_error) {
                        min_error = firstError;

                        cv::cvtColor(binarizedROImasked2, binarizedROImasked2, CV_GRAY2BGR);
                        cv::cvtColor(binarizedROImasked, binarizedROImasked, CV_GRAY2BGR);
                        cv::cvtColor(firstDiff, firstDiff, CV_GRAY2BGR);
                        cv::cvtColor(firstCandidateMasked, firstCandidateMasked, CV_GRAY2BGR);

                        cv::Mat blended;
                        cv::addWeighted(firstCandidateImg, 0.2, maskedROI, 0.8, 0.0, blended);

                        images = std::vector<cv::Mat>({maskedROI, binarizedROImasked, firstCandidateImg, firstCandidateMask, firstCandidateMasked, binarizedROImasked2, firstDiff, blended});
                    }

                    std::cout << "first error: " << firstError << std::endl;
                }
                }
                }
            }
        }
    }

    std::cout << "min error: " << min_error << std::endl;

    images.insert(images.begin(), tag.getOrigSubImage());

    const auto canvas = CvHelper::makeCanvas(images, images[0].rows + 10, 1);

	std::string title("best fit (error: " + std::to_string(min_error) + ")");
	cv::namedWindow(title);
	cv::imshow(title, canvas);

	cv::waitKey();

	// TODO
	return std::vector<Grid>();
}

}
