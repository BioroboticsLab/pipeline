#pragma once

#include <set>
#include <vector>

#include "source/tracking/algorithm/BeesBook/Common/Grid.h"

#include "datastructure/PipelineGrid.h"

namespace pipeline {

class Ellipse;
class Tag;
class TagCandidate;

typedef struct {
	// error function weights
	double alpha_inner    = 275.0;
	double alpha_outer    = 100.0;
	double alpha_variance = 0.3;
	double alpha_edge     = 10.0;

    // size of neighbourhood area
    int adaptiveBlockSize = 23;
    // constant which is substracted from mean of neighborhood area
    double adaptiveC      = 3;
} gridfitter_settings_t;

class GridFitter {
public:
	GridFitter();

	void loadSettings(gridfitter_settings_t&& settings);

	std::vector<Tag> process(std::vector<Tag>&& taglist);
private:
    struct candidate_t {
        double error;
        PipelineGrid::gridconfig_t config;

        bool operator<(const candidate_t& other) const {
            return error < other.error;
        }

        bool operator==(const candidate_t& other) const {
            return error == other.error;
        }
    };

    gridfitter_settings_t _settings;

    std::vector<Grid> fitGrid(const Tag &tag, TagCandidate const& candidate) const;
    double evaluateCandidate (PipelineGrid& grid, const cv::Mat& roi, const cv::Mat& binarizedROI) const;

    cv::Mat calculateHistogram(const cv::Mat& roi, const Ellipse& ellipse_orig) const;
    void visualizeDebug(const std::multiset<candidate_t>& bestGrids, const size_t numResults, const cv::Size2i roiSize, const Tag& tag, const cv::Mat& binarizedROI) const;
};
}
