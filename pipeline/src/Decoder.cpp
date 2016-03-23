#include "../Decoder.h"

#include <opencv2/opencv.hpp>

#include "../datastructure/Tag.h"
#include "../datastructure/TagCandidate.h"

#include "../util/ThreadPool.h"
#include "../util/CvHelper.h"

#include "../datastructure/PipelineGrid.impl.h"

// #define DEBUG_DECODER

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
    // TODO: add to settings
    static const double BLUR_KERNEL [3] = {0.25,0.50,0.25};
    static const double SHARPENING_FACTOR = 2.0;

    // region of interest of tag candidate
    const cv::Mat& roi = tag.getRepresentations().clahe;

    const cv::Point roiOffset = tag.getRoi().tl();

    std::vector<decoding_t> decodings;
    for (PipelineGrid& grid : candidate.getGrids()) {

        decoding_t decoding;
        double cellMedians[Grid::NUM_MIDDLE_CELLS];
        double blurredCellMedians[Grid::NUM_MIDDLE_CELLS];
        double sharpenedCellMedians[Grid::NUM_MIDDLE_CELLS];

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

            mean_calculator_t gridCellMeanCalculator(roi, roiOffset);
            gridCellMeanCalculator = grid.processGridCellCoordinates(idx, std::move(gridCellMeanCalculator));

            median_calculator_t gridCellMedianCalculator(roi, roiOffset);
            gridCellMedianCalculator = grid.processGridCellCoordinates(idx, std::move(gridCellMedianCalculator));

            const double gridCellMedian = gridCellMedianCalculator.getMedian();

            const double distanceBlack = std::abs(gridCellMedian - meanBlack);
            const double distanceWhite = std::abs(gridCellMedian - meanWhite);

            cellMedians[idx] = gridCellMedian;

#ifdef DEBUG_DECODER
            std::cout << "Cell median " << idx << ": " << gridCellMedian << "\n";
#endif

            // superwhite - mark as a reference TODO
            if (gridCellMedian > meanWhite) {
                decoding.set(idx, true);
            }
            // superblack - mark as a reference TODO
            else if (gridCellMedian < meanBlack) {
                decoding.set(idx, false);
            }
            else if (distanceWhite < distanceBlack) {
                decoding.set(idx, true);
            }
        }

        for (size_t idx = 0; idx < Grid::NUM_MIDDLE_CELLS; ++ idx) {

            const size_t idxCurr = (idx + 1) % Grid::NUM_MIDDLE_CELLS;
            const size_t idxNext = (idx + 2) % Grid::NUM_MIDDLE_CELLS;
            const size_t idxPrev = (idx);

            blurredCellMedians[idxCurr] = BLUR_KERNEL[0] * cellMedians[idxPrev] + BLUR_KERNEL[1] * cellMedians[idxCurr] + BLUR_KERNEL[2] * cellMedians[idxNext];
            sharpenedCellMedians[idxCurr] = cellMedians[idxCurr] + SHARPENING_FACTOR*(cellMedians[idxCurr] - blurredCellMedians[idxCurr]);

#ifdef DEBUG_DECODER
            std::cout << "Cell median sharpened " << idxCurr << ": " << sharpenedCellMedians[idxCurr] << "\n";
#endif

            const double distanceBlack = std::abs(sharpenedCellMedians[idxCurr] - meanBlack);
            const double distanceWhite = std::abs(sharpenedCellMedians[idxCurr] - meanWhite);

            if (distanceWhite < distanceBlack) {
                decoding.set(idxCurr, true);
            } else {
                decoding.set(idxCurr, false);
            }
        }

#ifdef DEBUG_DECODER
        size_t shift = 9;
        std::cout << "Decoding: ";
        for (size_t i = Grid::NUM_MIDDLE_CELLS + shift - 1; i >= shift; --i) {
            std::cout << decoding[(i)%Grid::NUM_MIDDLE_CELLS];
        }
        std::cout << "\n";

        visualizeDebug(tag, grid, decoding);
#endif

        decodings.push_back(decoding);
    }
    return decodings;
}

void Decoder::visualizeDebug(const Tag &tag, PipelineGrid &grid, pipeline::decoding_t const& decoding) const
{
    grid.setCenter(grid.getCenter() - tag.getRoi().tl());

    cv::Mat roi = tag.getRepresentations().orig;
    cv::Size roiSize = roi.size();

    std::vector<cv::Mat> images;

    images.push_back(tag.getRepresentations().orig);

    images.push_back(grid.getProjectedImage(roiSize));

    cv::Mat blended;
    cv::addWeighted(tag.getRepresentations().orig, 0.8, grid.getProjectedImage(roiSize), 0.2, 0.0, blended);
    images.push_back(blended);

    cv::Mat origCopyOverlay;
    tag.getRepresentations().orig.copyTo(origCopyOverlay);
    grid.drawContours(origCopyOverlay, 0.5);
    images.push_back(origCopyOverlay);

    cv::Mat test(roiSize, CV_8UC1, cv::Scalar::all(0));
    grid.draw(test, boost::make_optional(decoding));
    images.push_back(test.clone());

    const auto canvas = CvHelper::makeCanvas(images, images[0].rows + 10, 1);

    grid.setCenter(grid.getCenter() + tag.getRoi().tl());

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
