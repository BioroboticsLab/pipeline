#include "Decoder.h"

#include <opencv2/opencv.hpp>

#include "datastructure/Tag.h"
#include "datastructure/TagCandidate.h"
#include "util/ThreadPool.h"
#include "source/utility/CvHelper.h"

#include "datastructure/PipelineGrid.impl.h"

//#define DEBUG_DECODER

namespace pipeline {

Decoder::Decoder()
{}

void Decoder::loadSettings(decoder_settings_t &&settings)
{
	_settings = std::move(settings);
}

std::vector<Tag> Decoder::process(std::vector<Tag> &&taglist)
{
#ifdef DEBUG_DECODER
	for (Tag& tag : taglist) {
		for (TagCandidate& candidate : tag.getCandidates()) {
			std::vector<decoding_t> decodings = getDecodings(tag, candidate);
			candidate.setDecodings(std::move(decodings));
		}
	}
#else
	static const size_t numThreads = std::thread::hardware_concurrency() ?
	            std::thread::hardware_concurrency() * 2 : 1;
	ThreadPool pool(numThreads);

	std::vector<std::future<void>> results;
	for (Tag& tag : taglist) {
		results.emplace_back(
		            pool.enqueue([&] {
			for (TagCandidate& candidate : tag.getCandidates()) {
				std::vector<decoding_t> decodings = getDecodings(tag, candidate);
				candidate.setDecodings(std::move(decodings));
			}
		}));
	}

	for (auto && result : results) result.get();
#endif

	return std::move(taglist);
}

std::vector<decoding_t> Decoder::getDecodings(const Tag &tag, TagCandidate &candidate) const
{
	// region of interest of tag candidate
	cv::Mat roi;
	// TODO: shouldn't be BGR in the first place
	cv::cvtColor(tag.getOrigSubImage(), roi, CV_BGR2GRAY);

	const cv::Point roiOffset = tag.getBox().tl();

	std::vector<decoding_t> decodings;
	for (PipelineGrid& grid : candidate.getGrids()) {
		decoding_t decoding;

		mean_calculator_t blackMeanCalculator(roi, roiOffset);
		blackMeanCalculator = grid.processInnerBlackRingCoordinates(std::move(blackMeanCalculator));

		mean_calculator_t whiteMeanCalculator(roi, roiOffset);
		whiteMeanCalculator = grid.processInnerWhiteRingCoordinates(std::move(whiteMeanCalculator));

		// grid cell coordinates have to be initialized before outer ring coordinates. only small
		// performance overhead because they will be cached by the PipelineGrid for the next use
		for (size_t idx = 0; idx < Grid::NUM_MIDDLE_CELLS; ++ idx) {
			// just calculate the coordinates, do nothing with them
			grid.processGridCellCoordinates(idx, dummy_functor_t());
		}
		whiteMeanCalculator = grid.processOuterRingCoordinates(std::move(whiteMeanCalculator));

		const double meanBlack = blackMeanCalculator.getMean();
		const double meanWhite = whiteMeanCalculator.getMean();

		for (size_t idx = 0; idx < Grid::NUM_MIDDLE_CELLS; ++ idx) {
			distance_calculator_t distanceCalculator(roi, roiOffset, meanBlack, meanWhite);
			distanceCalculator = grid.processGridCellCoordinates(idx, std::move(distanceCalculator));

			const double distanceBlack = distanceCalculator.getDistanceBlack();
			const double distanceWhite = distanceCalculator.getDistanceWhite();

			if (distanceWhite < distanceBlack) {
				decoding.set(idx, true);
			}
		}

#ifdef DEBUG_DECODER
		visualizeDebug(tag, grid, decoding);
#endif

		decodings.push_back(decoding);
	}
	return decodings;
}

void Decoder::visualizeDebug(const Tag &tag, PipelineGrid &grid, pipeline::decoding_t const& decoding) const
{
	grid.setCenter(grid.getCenter() - tag.getBox().tl());

	cv::Mat roi = tag.getOrigSubImage();
	cv::Size roiSize = roi.size();

	std::vector<cv::Mat> images;

	images.push_back(tag.getOrigSubImage());

	images.push_back(grid.getProjectedImage(roiSize));

	cv::Mat blended;
	cv::addWeighted(tag.getOrigSubImage(), 0.8, grid.getProjectedImage(roiSize), 0.2, 0.0, blended);
	images.push_back(blended);

	cv::Mat origCopyOverlay;
	tag.getOrigSubImage().copyTo(origCopyOverlay);
	grid.drawContours(origCopyOverlay, 0.5);
	images.push_back(origCopyOverlay);

	cv::Mat test(roiSize, CV_8UC1, cv::Scalar::all(0));
	grid.draw(test, boost::make_optional(decoding));
	cv::Mat testCpy;
	cv::cvtColor(test, testCpy, CV_GRAY2BGR);
	images.push_back(testCpy);

	const auto canvas = CvHelper::makeCanvas(images, images[0].rows + 10, 1);

	grid.setCenter(grid.getCenter() + tag.getBox().tl());

	std::string title("decoding: " + decoding.to_string());
	cv::namedWindow(title);
	cv::imshow(title, canvas);

	bool cont = true;
	while (cont) {
		const char c = cv::waitKey();
		if (c == 'd') {
			cv::destroyAllWindows();
			cont = false;
		} else if (c == 'c') {
			cont = false;
		}
	}
}

double Decoder::getMeanIntensity(const cv::Mat &image, const PipelineGrid::coordinates_t &coords, const cv::Point& offset) const
{
	size_t sum = 0;
	for (cv::Point const& loc : coords.areaCoordinates) {
		sum += image.at<uint8_t>(loc - offset);
	}
	return static_cast<double>(sum) / static_cast<double>(coords.areaCoordinates.size());
}

double Decoder::getMeanDistance(const cv::Mat &image, const PipelineGrid::coordinates_t &coords, const cv::Point& offset, const double mean) const
{
	double sum = 0;
	for (cv::Point const& loc : coords.areaCoordinates) {
		sum += std::abs(image.at<uint8_t>(loc - offset) - mean);
	}
	return sum / static_cast<double>(coords.areaCoordinates.size());
}
}
