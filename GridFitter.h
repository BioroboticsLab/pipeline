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
	double alpha_inner    = 500.0;
	double alpha_outer    = 100.0;
	double alpha_variance = 0.8;
	double alpha_outer_edge = 5.0;
	double alpha_inner_edge = 10.0;

    // size of neighbourhood area
    int adaptiveBlockSize = 23;
    // constant which is substracted from mean of neighborhood area
    double adaptiveC      = 3;

	// gradient descent parameters
	size_t numInitial = 1;
	size_t numResults = 1;

	double errorThreshold = 80.;
	size_t maxIterations  = 100;

	double eps_angle = 0.02;
	int eps_pos      = 1;
	double eps_scale = 0.1;
	double alpha     = 0.01;
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

	typedef std::multiset<candidate_t> candidate_set;

	class GradientDescent {
	public:
		GradientDescent(const candidate_set& initialCandidates,
		                const cv::Mat& roi,
		                const cv::Mat& binarizedRoi,
						const cv::Mat& edgeRoi,
		                const gridfitter_settings_t& settings);

		void optimize();

		candidate_set const& getBestGrids() const { return _bestGrids; }

	private:
		const candidate_set& _initialCandidates;
		const gridfitter_settings_t& _settings;
		const cv::Mat& _roi;
		const cv::Mat& _binarizedRoi;
		const cv::Mat& _edgeRoi;

		candidate_set _bestGrids;

		enum StepParameter {
			POSX = 0,
			POSY,
			ANGLE_X,
			ANGLE_Y,
			ANGLE_Z,
			SCALE
		};

		std::pair<double, PipelineGrid::gridconfig_t> step(const PipelineGrid::gridconfig_t &config, double error, const StepParameter param);

		void storeConfig(const double error, const PipelineGrid::gridconfig_t &config);
	};

    gridfitter_settings_t _settings;

    std::vector<PipelineGrid> fitGrid(const Tag &tag, TagCandidate const& candidate) const;
    candidate_set getInitialCandidates(cv::Mat const& binarizedROI, const cv::Mat& edgeROI, const Ellipse& ellipse_orig, cv::Mat const& roi) const;
    static double evaluateCandidate (PipelineGrid& grid, const cv::Mat& roi, const cv::Mat& binarizedROI, const cv::Mat& edgeROI, const gridfitter_settings_t& settings);

    cv::Mat calculateHistogram(const cv::Mat& roi, const Ellipse& ellipse_orig) const;
    void visualizeDebug(const std::multiset<candidate_t>& bestGrids, const cv::Size2i roiSize, const Tag& tag, const cv::Mat& binarizedROI, std::string winName) const;
};
}
