
#include "CherryDetector.hpp"

#include "BestColorObjTracker.hpp"

CherryDetector::CherryDetector(
	const std::string& track_cfg,
	const std::string& color_cfg,
	const std::string& img
) :
	bcot(new BestColorObjTracker(track_cfg, color_cfg, img))
{
}

CherryDetector::~CherryDetector() {
	delete bcot;
}

void CherryDetector::detect_on_frame(
	vec3& cmd
) {
	bcot->track_best(
		cmd
	);
}

bool CherryDetector::next_frame() {
	return bcot->next_frame();
}
