
#ifndef FRAME_PRODUCER_HPP
#define FRAME_PRODUCER_HPP

#include "opencv2/opencv.hpp"
#include <stdexcept>
#include <vector>

using namespace std;
using namespace cv;

enum LoopKeyCtrl{
	VIDEO,
	SLIDES,
	ONE_FRAME
};

class FrameProducer {
public:
	FrameProducer(LoopKeyCtrl default_ctrl)
		: default_ctrl(default_ctrl), update_without_move_to_next(false) {
		
	}
	
	virtual Mat getFrame() = 0;
	virtual void seekForward() = 0;
	virtual void seekBackward() = 0;
	
	void repeat_frame() {
		update_without_move_to_next = true;
	}
	bool loopKeyCtrl(LoopKeyCtrl ctrl);
	bool next_frame() {
		return loopKeyCtrl(default_ctrl);
	}
protected:
	LoopKeyCtrl default_ctrl;
	bool update_without_move_to_next;
};

class ImageFrameProducer : public FrameProducer {
public:
	ImageFrameProducer(const string& img_file_name_pattern);
	
	virtual Mat getFrame();
	virtual void seekForward();
	virtual void seekBackward();
protected:
	vector<string> img_file_names;
	int img_idx;
};

class CameraFrameProducer : public FrameProducer {
public:
	CameraFrameProducer(int index = 0);
	
	virtual Mat getFrame();
	virtual void seekForward(){}
	virtual void seekBackward(){}
protected:
	VideoCapture cap;
};



#endif // FRAME_PRODUCER_HPP
