
#ifndef DEBUG_IMG_UTILS_HPP
#define DEBUG_IMG_UTILS_HPP

#include <string>
#include "opencv2/core.hpp"

using namespace cv;
using namespace std;

namespace __OpenCV_Utils {
	void show_img_and_print_stats_just_first_time(
		const char* name,
		InputArray m,
		const char* channelNames = NULL
	);
	void show_img_and_print_stats_just_first_time_BGR(
		const char* name,
		InputArray m
	);
	void show_img_and_print_stats_just_first_time_HSV(
		const char* name,
		InputArray m
	);
};
#define DEBUG_IMG(m) \
	do{ \
		__OpenCV_Utils::show_img_and_print_stats_just_first_time(#m, m); \
	}while(0)
#define DEBUG_IMG_CHANNELS(m, ch_names) \
	do{ \
		__OpenCV_Utils::show_img_and_print_stats_just_first_time(#m, m, ch_names); \
	}while(0)

#define DEBUG_IMG_BGR(m) \
	do{ \
		__OpenCV_Utils::show_img_and_print_stats_just_first_time_BGR(#m, m); \
	}while(0)

#define DEBUG_IMG_HSV(m) \
	do{ \
		__OpenCV_Utils::show_img_and_print_stats_just_first_time_HSV(#m, m); \
	}while(0)

string mat_stats(InputArray m);

#endif // DEBUG_IMG_UTILS_HPP
