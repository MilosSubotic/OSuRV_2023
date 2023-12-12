
#include "PrintUtils.hpp"
#include "UART.hpp"
#include "ColorObjDetector.hpp"

using namespace std;
using namespace cv;

#define EN_BLOB 0


enum t_turn {TURN_LEFT, TURN_NONE, TURN_RIGHT};
enum t_move {MOVE_FORWARD, MOVE_NONE, MOVE_REVERSE};

int main(int argc, char** argv) {
	UART uart("/dev/serial0", 115200);
	
	ColorObjDetector cod("./build/thresholds.cfg.jl");
	do{
		cod.process_image();
#if EN_BLOB
		cod.blob_detect();
#endif
		cod.mean_detect();
		
		if(cod.mean_keypoints.empty()){
			char x_diff = '?';
			char y_diff = '?';
			DEBUG(x_diff);
			DEBUG(y_diff);
		}else{
			int x_diff = cod.src.cols/2-cod.mean_keypoints[0].pt.x;
			int y_diff = cod.src.rows/2-cod.mean_keypoints[0].pt.y;
			DEBUG(x_diff);
			DEBUG(y_diff);
			
			t_turn turn = TURN_NONE;
			if(x_diff > 50){
				turn = TURN_RIGHT;
			}else if(x_diff < -50){
				turn = TURN_LEFT;
			}

			t_move move = MOVE_NONE;
			if(y_diff < -130){
				move = MOVE_FORWARD;
			}else if(y_diff > -100){
				move = MOVE_REVERSE;
			}
			
			vector<uint8_t> cmd;
			uint8_t chassis_move;
			switch(move){
				case MOVE_NONE:
					chassis_move = 0;
					break;
				case MOVE_FORWARD:
					chassis_move = 2;
					break;
				case MOVE_REVERSE:
					chassis_move = 3;
					break;
			}
			uint8_t chassis_turn;
			switch(turn){
				case TURN_NONE:
					chassis_turn = 0;
					break;
				case TURN_RIGHT:
					chassis_turn = 1;
					break;
				case TURN_LEFT:
					chassis_turn = 2;
					break;
			}
			uint8_t chassis = (chassis_turn << 2) | chassis_move;
			cmd.push_back(chassis);
			uart.write(cmd);
		}
		
		
	}while(cod.next_frame());
	

	return 0;
}
