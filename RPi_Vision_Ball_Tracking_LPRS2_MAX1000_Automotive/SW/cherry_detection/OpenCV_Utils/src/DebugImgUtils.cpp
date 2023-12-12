
#include "DebugImgUtils.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <sstream>
#include <set>

#include "PrintUtils.hpp"

namespace __OpenCV_Utils {
	set<const char*> alreadyPrintedStats;
	
	static void print_stats_just_first_time(
		const char* name,
		InputArray m
	) {
		if(alreadyPrintedStats.find(name) == alreadyPrintedStats.end()){
			alreadyPrintedStats.insert(name);
			cout << "Mat " << name << " is " << mat_stats(m) << endl;
		}
	}

	void show_img_and_print_stats_just_first_time(
		const char* name,
		InputArray m,
		const char* channelNames
	) {
		print_stats_just_first_time(name, m);
		
		if(channelNames == NULL){
			imshow(name, m);
		}else{
			vector<Mat> s;
			Mat zeros(m.size(), CV_8UC1, Scalar::all(0));
			Mat tmp;
			split(m, s);
			int ch_H = -1;
			int ch_S = -1;
			for(int i = 0; i < s.size(); i++){
				ostringstream oss;
				oss << name << "--" << channelNames[i];
				imshow(oss.str(), s[i]);
			}
		}
	}

	void show_img_and_print_stats_just_first_time_BGR(
		const char* name,
		InputArray m
	) {
		print_stats_just_first_time(name, m);

		vector<Mat> s;
		split(m, s);
		Mat zeros(m.size(), CV_8UC1, Scalar::all(0));
		Mat tmp;
		ostringstream oss;
		
		oss.str("");
		oss << name << "--B";
		merge(vector<Mat>({s[0], zeros, zeros}), tmp);
		imshow(oss.str(), tmp);

		oss.str("");
		oss << name << "--G";
		merge(vector<Mat>({zeros, s[1], zeros}), tmp);
		imshow(oss.str(), tmp);

		oss.str("");
		oss << name << "--R";
		merge(vector<Mat>({zeros, zeros, s[2]}), tmp);
		imshow(oss.str(), tmp);
	}

	void show_img_and_print_stats_just_first_time_HSV(
		const char* name,
		InputArray m
	) {
		print_stats_just_first_time(name, m);

		vector<Mat> s;
		split(m, s);
		Mat zeros(m.size(), CV_8UC1, Scalar::all(0));
		Mat tmp;
		ostringstream oss;


		Mat H = s[0];
		Mat S = s[1];
		Mat V = s[2];
		
		oss.str("");
		oss << name << "--H";

		Mat hsv, color;
		Mat ones(m.size(), CV_8UC1, Scalar::all(255));
		if(true){
			// If saturation is 0, then set pixel to black.
			Mat masked_H;
			Mat masked_ones;
			H.copyTo(masked_H, S);
			ones.copyTo(masked_ones, S);
			merge(vector<Mat>({masked_H, masked_ones, masked_ones}), hsv);
		}else{
			merge(vector<Mat>({H, ones, ones}), hsv);
		}
		cvtColor(hsv, color, COLOR_HSV2BGR);
		imshow(oss.str(), color);
	

		oss.str("");
		oss << name << "--S";
		imshow(oss.str(), S);

		oss.str("");
		oss << name << "--V";
		imshow(oss.str(), V);

	}


};

string mat_stats(InputArray m) {
	
	ostringstream oss;

	int type = m.getMat().type();
	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar channels = 1 + (type >> CV_CN_SHIFT);

	switch(depth){
		case CV_8U:
			oss << "8U";
			break;
		case CV_8S:
			oss << "8S";
			break;
		case CV_16U:
			oss << "16U";
			break;
		case CV_16S:
			oss << "16S";
			break;
		case CV_32S:
			oss << "32S";
			break;
		case CV_32F:
			oss << "32F";
			break;
		case CV_64F:
			oss << "64F";
			break;
		default:
			oss << "User";
			break;
	}

	oss << "C" << (int)channels;

	oss << ' ' << m.cols() << 'x' << m.rows();

	return oss.str();
}
