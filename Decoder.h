#pragma once

#include <vector>

#include "datastructure/Tag.h"
#include "datastructure/TagCandidate.h"

#include "util/Util.h"

namespace pipeline {

typedef struct {
} decoder_settings_t;

class Decoder {
public:
	Decoder();

	void loadSettings(decoder_settings_t&& settings);

	std::vector<Tag> process(std::vector<Tag>&& taglist);

private:
	decoder_settings_t _settings;

	std::vector<decoding_t> getDecodings(pipeline::Tag const& tag, TagCandidate &candidate) const;

	void visualizeDebug(pipeline::Tag const& tag, PipelineGrid &grid, const decoding_t &decoding) const;

	class mean_calculator_t {
	public:
		explicit mean_calculator_t(cv::Mat const& roi, const cv::Point roiOffset)
		    : _roi(roi), _roiOffset(roiOffset), _sum(0), _pixelNum(0) {}

		inline void operator()(const cv::Point coords) {
			if (Util::pointInBounds(_roi.get().size(), coords - _roiOffset)) {
				const uint8_t value = _roi.get().template at<uint8_t>(coords - _roiOffset);
				_sum += value;
				++_pixelNum;
			}
		}

		inline double getMean() const {
			return static_cast<double>(_sum) / static_cast<double>(_pixelNum);
		}

	private:
		std::reference_wrapper<const cv::Mat> _roi;
		cv::Point _roiOffset;
		size_t _sum;
		size_t _pixelNum;
	};

	class distance_calculator_t {
	public:
		explicit distance_calculator_t(cv::Mat const& roi, const cv::Point roiOffset, const double meanBlack, const double meanWhite)
		    : _roi(roi), _roiOffset(roiOffset), _meanBlack(meanBlack), _meanWhite(meanWhite), _distanceSumBlack(0.), _distanceSumWhite(0.), _pixelNum(0) {}

		inline void operator()(const cv::Point coords) {
			if (Util::pointInBounds(_roi.get().size(), coords - _roiOffset)) {
				const double value = static_cast<double>(_roi.get().template at<uint8_t>(coords - _roiOffset));

				_distanceSumBlack += std::abs(value - _meanBlack);
				_distanceSumWhite += std::abs(value - _meanWhite);

				++_pixelNum;
			}
		}

		inline double getDistanceBlack() const {
			return _distanceSumBlack / static_cast<double>(_pixelNum);
		}

		inline double getDistanceWhite() const {
			return _distanceSumWhite / static_cast<double>(_pixelNum);
		}

	private:
		std::reference_wrapper<const cv::Mat> _roi;
		cv::Point _roiOffset;
		double _meanBlack;
		double _meanWhite;
		double _distanceSumBlack;
		double _distanceSumWhite;
		size_t _pixelNum;
	};

	class dummy_functor_t {
	public:
		inline void operator()(const cv::Point) {}
	};

	double getMeanIntensity(cv::Mat const& image, const PipelineGrid::coordinates_t& coords, const cv::Point& offset) const;
	double getMeanDistance(cv::Mat const& image, const PipelineGrid::coordinates_t& coords, const cv::Point& offset, const double mean) const;
};
}
