
#pragma once

#include <opencv2/features2d.hpp>

struct BlobDetectorParams2 {
	int contour_idx_percent_start;
	int contour_idx_percent_stop;
	
	BlobDetectorParams2() :
		contour_idx_percent_start(0),
		contour_idx_percent_stop(100)
	{}
};

void detect_blobs(
	cv::InputArray image,
	std::vector<cv::KeyPoint>& keypoints,
	const cv::SimpleBlobDetector::Params& params,
	const BlobDetectorParams2& params2 = BlobDetectorParams2()
);
