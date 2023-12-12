
#pragma once

#include <string>
#include "vec3.hpp"

class BestColorObjTracker;

/**
 * Just a wrapper to have minimal (non-OpenCV stuff) in header.
 */
class CherryDetector {
public:
	CherryDetector(
		const std::string& track_cfg,
		const std::string& color_cfg,
		const std::string& img = ""
	);
	~CherryDetector();
	/**
	 * @param cmd.x + is Forward
	 * @param cmd.y + is Left
	 * @param cmd.z + is Up
	 */
	void detect_on_frame(
		vec3& cmd
	);
	bool next_frame();
private:
	BestColorObjTracker* bcot;
};
