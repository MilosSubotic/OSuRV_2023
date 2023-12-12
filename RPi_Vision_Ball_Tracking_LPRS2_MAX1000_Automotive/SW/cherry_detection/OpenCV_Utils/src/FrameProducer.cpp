
#include "FrameProducer.hpp"

#include "PrintUtils.hpp"

#include "glob.hpp"

bool FrameProducer::loopKeyCtrl(LoopKeyCtrl ctrl) {
	do{
		int key = cv::waitKey(30) & 0xff;
		if(key == 27){
			// Break when ESC key is pressed.
			return false;
		}
		switch(ctrl){
			case VIDEO:
				// Go for next frame.
				return true;
				break;
			case SLIDES:
				if(key == 83){ // Right arrow.
					seekForward();
					return true;
				}else if(key == 81){ // Left arrow.
					seekBackward();
					return true;
				}
				// Else, wait more.
				break;
			case ONE_FRAME:
				// Loop until ESC.
				break;
		}
		// Allow another loop with same image because settings changed.
		if(update_without_move_to_next){
			update_without_move_to_next = false;
			return true;
		}
	}while(true);
}

ImageFrameProducer::ImageFrameProducer(const string& img_file_name_pattern) :
	FrameProducer(SLIDES)
{
	img_idx = 0;
	
	img_file_names = glob(img_file_name_pattern);
}

Mat ImageFrameProducer::getFrame() {
	auto fn = img_file_names[img_idx];
	
	Mat m = imread(fn);
	if(m.empty()){
		throw runtime_error("Cannot open image!");
	}

	return m;
}


void ImageFrameProducer::seekForward() {
	img_idx++;
	if(img_idx >= img_file_names.size()){
		img_idx = 0;
		cout << "Wrapping image slideshow..." << endl;
	}
}
void ImageFrameProducer::seekBackward() {
	img_idx--;
	if(img_idx < 0){
		img_idx = img_file_names.size()-1;
		cout << "Wrapping image slideshow..." << endl;
	}
}

Mat CameraFrameProducer::getFrame() {
	Mat m;
	cap.read(m);
	if(m.empty()){
		throw runtime_error("Cannot grab frame!");
	}
	
	return m;
}

CameraFrameProducer::CameraFrameProducer(int index) :
	FrameProducer(VIDEO)
{
	cap.open(index);
	if(!cap.isOpened()){
		throw runtime_error("Cannot open camera!");
	}
}
