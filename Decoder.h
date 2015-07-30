#pragma once

#include <vector>
#include <algorithm>

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


    class median_calculator_t {
    public:
        explicit median_calculator_t(cv::Mat const& roi, const cv::Point roiOffset)
            : _roi(roi), _roiOffset(roiOffset), _pixelNum(0) {}

        inline void operator()(const cv::Point coords) {
            if (Util::pointInBounds(_roi.get().size(), coords - _roiOffset)) {
                // add value to vector
                const uint8_t value = _roi.get().template at<uint8_t>(coords - _roiOffset);
                _values.push_back(static_cast<int>(value));
                ++_pixelNum;
            }
        }

        inline double getMedian() {

            if (_values.empty()) {
                return 0;
            }

            int size = _values.size();
            std::sort(_values.begin(), _values.end());

            if(size % 2 == 0) {
                return (_values[size/2 - 1] + _values[size/2]) / 2;
            } else {
                return _values[size/2];
            }
        }

    private:
        std::reference_wrapper<const cv::Mat> _roi;
        cv::Point _roiOffset;
        std::vector<int> _values;
        size_t _pixelNum;
    };


    class median_calculator_w_radius_t {
    public:
        explicit median_calculator_w_radius_t(cv::Mat const& roi, const cv::Point roiOffset, const cv::Point center, const int radius)
            : _roi(roi), _roiOffset(roiOffset), _center(center), _radius(radius), _pixelNum(0) {}

        inline void operator()(const cv::Point coords) {
            if (Util::pointInBounds(_roi.get().size(), coords - _roiOffset)) {
                int distance = static_cast<int>(std::hypot((_center.x - coords.x),(_center.y - coords.y)));
                if (distance <= _radius) {
                    // add value to vector
                    const uint8_t value = _roi.get().template at<uint8_t>(coords - _roiOffset);
                    _values.push_back(static_cast<int>(value));
                    ++_pixelNum;
                }
            }
        }

        inline double getMedian() {

            if (_values.empty()) {
                return 0;
            }

            int size = _values.size();
            std::sort(_values.begin(), _values.end());

            if(size % 2 == 0) {
                return (_values[size/2 - 1] + _values[size/2]) / 2;
            } else {
                return _values[size/2];
            }
        }

    private:
        std::reference_wrapper<const cv::Mat> _roi;
        cv::Point _roiOffset;
        cv::Point _center;
        int _radius;
        std::vector<int> _values;
        size_t _pixelNum;
    };

    class mean_calculator_w_radius_t {
    public:
        explicit mean_calculator_w_radius_t(cv::Mat const& roi, const cv::Point roiOffset, const cv::Point center, const int radius)
            : _roi(roi), _roiOffset(roiOffset), _center(center), _radius(radius), _sum(0), _pixelNum(0) {}

        inline void operator()(const cv::Point coords) {
            if (Util::pointInBounds(_roi.get().size(), coords - _roiOffset)) {
                int distance = static_cast<int>(std::hypot((_center.x - coords.x),(_center.y - coords.y)));
                if (distance <= _radius) {
                    //std::cout << "x: " << coords.x << ", y: " << coords.y << "; ";
                    const uint8_t value = _roi.get().template at<uint8_t>(coords - _roiOffset);
                    _sum += value;
                    ++_pixelNum;
                }
            }
        }

        inline double getMean() const {
            return static_cast<double>(_sum) / static_cast<double>(_pixelNum);
        }

    private:
        std::reference_wrapper<const cv::Mat> _roi;
        cv::Point _roiOffset;
        cv::Point _center;
        int _radius;
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

    // Pawel begin

    class center_calculator_t {
    public:
        explicit center_calculator_t(cv::Mat const& roi, const cv::Point roiOffset)
            : _roi(roi), _roiOffset(roiOffset), _sumX(0), _sumY(0), _pixelNum(0) {}

        inline void operator()(const cv::Point coords) {
            if (Util::pointInBounds(_roi.get().size(), coords - _roiOffset)) {
                _sumX += coords.x;
                _sumY += coords.y;
                ++_pixelNum;
            }
        }

        inline cv::Point getCenter() const {
            int _meanX = static_cast<int>(_sumX / static_cast<double>(_pixelNum));
            int _meanY = static_cast<int>(_sumY / static_cast<double>(_pixelNum));
            cv::Point center(_meanX, _meanY);
            return center;
        }

    private:
        std::reference_wrapper<const cv::Mat> _roi;
        cv::Point _roiOffset;
        double _sumX;
        double _sumY;
        size_t _pixelNum;
    };

    // Pawel end

	class dummy_functor_t {
	public:
		inline void operator()(const cv::Point) {}
	};

	double getMeanIntensity(cv::Mat const& image, const PipelineGrid::coordinates_t& coords, const cv::Point& offset) const;
	double getMeanDistance(cv::Mat const& image, const PipelineGrid::coordinates_t& coords, const cv::Point& offset, const double mean) const;
};
}
