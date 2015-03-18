#include "Decoder.h"

#include <opencv2/opencv.hpp>

#include "datastructure/Tag.h"
#include "datastructure/TagCandidate.h"
#include "util/ThreadPool.h"

#define DEBUG_DECODER

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
    static const size_t numThreads = 1;
#else
    static const size_t numThreads = std::thread::hardware_concurrency() ?
                std::thread::hardware_concurrency() * 2 : 1;
#endif
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

			if (distanceBlack < distanceWhite) {
				decoding.set(idx, true);
			}
		}
		decodings.push_back(decoding);
	}
	return decodings;
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
