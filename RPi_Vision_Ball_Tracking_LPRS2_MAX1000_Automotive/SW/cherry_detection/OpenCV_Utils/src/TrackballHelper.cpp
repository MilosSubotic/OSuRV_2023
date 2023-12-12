
#include "TrackballHelper.hpp"


namespace __OpenCV_Utils {
	class Callback {
	public:
		function<void(int)> callback;
		Callback(function<void(int)> callback_) : callback(callback_) {}
	};
	
	void trackballCallback(int value, void* th) {
		static_cast<Callback*>(th)->callback(value);
	}
	
	void trackballHelper(
		const char* name,
		int& var,
		int max,
		function<void(int)> callback
	) {
		static bool firstTime = true;
		if(firstTime){
			firstTime = false;
			namedWindow(TRACKBALLS_WINDOW_NAME);
			Mat m(Size(1000, 1), CV_8UC1, Scalar(0));
			imshow(TRACKBALLS_WINDOW_NAME, m);
		}
		
		if(callback == nullptr){
			createTrackbar(
				name,
				TRACKBALLS_WINDOW_NAME,
				&var,
				max
			);
		}else{
			createTrackbar(
				name,
				TRACKBALLS_WINDOW_NAME,
				&var,
				max,
				trackballCallback,
				static_cast<void*>(new Callback(callback))
			);
		}
	}
};
