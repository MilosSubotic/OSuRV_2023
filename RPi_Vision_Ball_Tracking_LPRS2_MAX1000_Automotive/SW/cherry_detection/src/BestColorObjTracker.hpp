
#pragma once

#include <vector>
#include <string>
#include <deque>
using namespace std;
#include <opencv2/features2d.hpp>
using namespace cv;

#include "vec3.hpp"
#include "Settings.hpp"

class ColorObjDetector;

class BestColorObjTracker {
public:
	BestColorObjTracker(
		const string& track_cfg,
		const string& color_cfg,
		const string& img = ""
	);
	~BestColorObjTracker();
	/**
	 * @param x_speed + is Forward
	 * @param y_speed + is Left
	 * @param z_speed + is Up
	 */
	void track_best(
		vec3& cmd
	);
	void settings_changed();
	bool next_frame();
private:
	ColorObjDetector* cod;
	Point middle;

	Settings s;
	int middle_x;
	int middle_y;
	int size_th_0;
	int size_th_1;
	int dist_th_0;
	int dist_th_1;
	
	// front is newest
	deque<KeyPoint> best_history;
	deque<vec3> prop_cmd_history;
	vec3 established_cmd;
};
