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
	const cv::Size2i roiSize = tag.getBox().size();
	cv::Mat roi;
	// TODO: shouldn't be BGR in the first place
	cv::cvtColor(tag.getOrigSubImage(), roi, CV_BGR2GRAY);

	std::vector<decoding_t> decodings;
	for (PipelineGrid& grid : candidate.getGrids()) {
		decoding_t decoding;
		// TODO: fix PipelineGrid bugs
		try {
		// TODO: remove workaround
		cv::Point gridCenter = grid.getCenter();
		grid.setCenter(gridCenter - tag.getBox().tl());
		const std::vector<cv::Mat>& gridCellCoordinates = grid.getGridCellCoordinates(roiSize);

		const double meanBlack = getMeanIntensity(roi, grid.getInnerBlackRingCoordinates(roiSize));
		const double meanWhite = getMeanIntensity(roi, grid.getInnerWhiteRingCoordinates(roiSize));

		for (size_t idx = 0; idx < Grid::NUM_MIDDLE_CELLS; ++ idx) {
			const double distanceBlack = getMeanDistance(roi, gridCellCoordinates[idx], meanBlack);
			const double distanceWhite = getMeanDistance(roi, gridCellCoordinates[idx], meanWhite);

			if (distanceBlack < distanceWhite) {
				decoding.set(idx, true);
			}
		}
		// TODO: remove workaround
		grid.setCenter(gridCenter);
		} catch (std::exception& ex) {
			std::cout << ex.what() << std::endl;
			continue;
		}
		decodings.push_back(decoding);
	}
	return decodings;
}

double Decoder::getMeanIntensity(const cv::Mat &image, const cv::Mat &coords) const
{
	size_t sum = 0;
    for (size_t idx = 0; idx < coords.total(); ++idx) {
        const cv::Point2i loc = coords.at<cv::Point2i>(idx);
		sum += image.at<uint8_t>(loc);
    }
	return static_cast<double>(sum) / static_cast<double>(coords.total());
}

double Decoder::getMeanDistance(const cv::Mat &image, const cv::Mat &coords, const double mean) const
{
	double sum = 0;
    for (size_t idx = 0; idx < coords.total(); ++idx) {
        const cv::Point2i loc = coords.at<cv::Point2i>(idx);
		sum += std::abs(image.at<uint8_t>(loc) - mean);
    }
	return sum / static_cast<double>(coords.total());
}
}
