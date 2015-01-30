#include "GridFitter.h"

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "datastructure/Tag.h"
#include "datastructure/TagCandidate.h"
#include "source/tracking/algorithm/BeesBook/Common/Grid.h"

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
	const Ellipse& ellipse = candidate.getEllipse();

	cv::namedWindow("mask");
	cv::imshow("mask", ellipse.getMask());

	cv::Mat maskedROI(tag.getBox().size(), tag.getOrigSubImage().type());
	maskedROI.setTo(cv::Scalar(0));
	tag.getOrigSubImage().copyTo(maskedROI, ellipse.getMask());

	cv::namedWindow("ellipse");
	cv::imshow("ellipse", maskedROI);

	cv::Mat convertedROI(maskedROI.size(), CV_8UC1);
	cv::cvtColor(maskedROI, convertedROI, CV_BGR2GRAY);
	//maskedROI.convertTo(convertedROI, CV_8UC1);

	std::cout << CV_8UC1 << std::endl;
	std::cout << convertedROI.type() << std::endl;

	cv::Mat binarizedROI;
	cv::adaptiveThreshold(convertedROI, binarizedROI, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 21, 3);

	cv::namedWindow("binarized");
	cv::imshow("binarized", binarizedROI);

	cv::Mat binarizedROImasked(binarizedROI.size(), binarizedROI.type());
	binarizedROImasked.setTo(cv::Scalar(0));
	binarizedROI.copyTo(binarizedROImasked, ellipse.getMask());

	cv::namedWindow("binarizedMasked");
	cv::imshow("binarizedMasked", binarizedROImasked);

	cv::waitKey();


	// TODO
	return std::vector<Grid>();
}

}
