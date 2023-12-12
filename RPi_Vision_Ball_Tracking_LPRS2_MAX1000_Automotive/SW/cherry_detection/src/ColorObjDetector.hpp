
#pragma once

#include "Settings.hpp"
#include "FrameProducer.hpp"

#include <string>
using namespace std;
using namespace cv;



class ColorObjDetector {
public:
	ColorObjDetector(
		const string& cfg_fn,
		const string& img = ""
	);
	~ColorObjDetector() {
		delete fp;
	}
	
	void process_image();
	void blob_detect();
	void mean_detect();
	
	//TODO private
	std::vector<KeyPoint> blob_keypoints;
	std::vector<KeyPoint> mean_keypoints;
	
	Mat src;
	
	void draw_marker(
		Point p,
		int id
	);
	void show_results();
	
	void settings_changed() {
		fp->repeat_frame();
	}
	bool next_frame();
protected:
	FrameProducer* fp;
	
	Settings s;
	int start_H[2];
	int stop_H[2];
	int low_S;
	int high_S;
	int low_V;
	int high_V;
	
	int blob_minArea;
	int blob_maxArea;
	int blob_minCircularity;
	int blob_minConvexity;
	
	int contour_idx_percent_start;
	int contour_idx_percent_stop;
	
	Mat src_orig, src_resized, hsv, threshold, inv_threshold;
	Mat eroded, dilated, blured, for_detection;
	Mat threshold_with_keypoints, 
		for_detection_with_keypoints, src_with_keypoints;
	Mat inv_blured, for_mean, loc;
};
