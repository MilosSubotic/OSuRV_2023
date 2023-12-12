
#include "PrintUtils.hpp"
#include "ColorObjDetector.hpp"

using namespace std;
using namespace cv;

#define D_DIR "../../../Agriculture_Machinery_"
#define LD_DIR D_DIR"Light_Data/Cherry_Picker_Arm"
#define HD_DIR D_DIR"Heavy_Data/Media/Cherry_Picker_Arm"
#define CFG_DIR LD_DIR"/cherry_detection_thresholds"

//#define IMG HD_DIR"/yellow_ball.jpg"
//#define CFG CFG_DIR"/yellow_ball.thresholds.cfg.jl"

//#define IMG HD_DIR"/lego_frames/lego_frame_*.jpg"
//#define CFG CFG_DIR"/cherry_1_frames.thresholds.cfg.jl"

//#define IMG HD_DIR"/cherry_1_frames/cherry_1_frame_00008.jpg"
//#define IMG HD_DIR"/cherry_1_frames/cherry_1_frame_*.jpg"
//#define CFG CFG_DIR"/cherry_1_frames.thresholds.cfg.jl"

//#define IMG HD_DIR"/cherry_2_imgs/*.jpg"
//#define CFG CFG_DIR"/cherry_2_imgs.thresholds.cfg.jl"

//#define IMG HD_DIR"/cherry_1_frames/cherry_1_frame_*.jpg"
//#define IMG HD_DIR"/cherry_2_imgs/*.jpg"
//#define IMG HD_DIR"/cherry_3_pair_at_night__frames/*.jpg"
//#define IMG HD_DIR"/cherry_3_pair_at_night__frames/frame020.jpg"
//#define IMG HD_DIR"/cherry_4_imgs/*.jpg"
//#define IMG HD_DIR"/cherry_4_imgs/2022-06-11-002622.jpg"
#define IMG "" // Camera
#define TRACK_CFG CFG_DIR"/cherry_camera_1.track.cfg.jl"
#define COLOR_CFG CFG_DIR"/cherry_camera_1.color.cfg.jl"

#include "CherryDetector.hpp"

int main(int argc, char** argv) {
	auto cd = new CherryDetector(
		TRACK_CFG,
		COLOR_CFG,
		IMG
	);
	do{
		vec3 cmd;
		cd->detect_on_frame(cmd);
		DEBUG(cmd.x);
		DEBUG(cmd.y);
		DEBUG(cmd.z);
	}while(cd->next_frame());
	
	return 0;
}
