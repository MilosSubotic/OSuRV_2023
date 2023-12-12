
#ifndef TRACKBALL_HELPER_HPP
#define TRACKBALL_HELPER_HPP

#include "opencv2/highgui.hpp"
#include <functional>

using namespace cv;
using namespace std;

namespace __OpenCV_Utils {
	void trackballHelper(
		const char* name,
		int& var,
		int max,
		//TODO Remove this int value
		function<void(int)> callback = nullptr
	);
};

#define TRACKBALLS_WINDOW_NAME "Trackballs"

#define TRACKBALL(var, max) \
	do{ \
		__OpenCV_Utils::trackballHelper(#var, var, max); \
	}while(0)
	
#define TRACKBALL_WITH_CALLBACK(var, max, callback) \
	do{ \
		__OpenCV_Utils::trackballHelper(#var, var, max, callback); \
	}while(0)
	
#define UPDATE_TRACKBALL(var) \
	do{ \
		setTrackbarPos(#var, TRACKBALLS_WINDOW_NAME, var); \
	}while(0)



#endif // TRACKBALL_HELPER_HPP
