
#include "PrintUtils.hpp"
#include "ColorObjDetector.hpp"

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
	
	ColorObjDetector cod("./build/thresholds.cfg.jl");
	do{
		cod.process_image();
		cod.blob_detect();
		cod.mean_detect();
		
	}while(cod.next_frame());
	

	return 0;
}
