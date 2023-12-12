
#include "MyImplOpenCV.hpp"

#include <stdexcept>

using namespace std;

namespace my_cv {
	
	/**
	 * Calc min and max of 3 values.
	 */
	static void min_max(float R, float G, float B, float& min, float& max){
		max = 0.0, min = 255.0;

		if (B>max)
			max = B;
		if (B<min)
			min = B;

		if (G>max)
			max = G;
		if (G<min)
			min = G;

		if (R>max)
			max = R;
		if (R<min)
			min = R;
	}

	// r,g,b values are from 0 to 1
	// h = [0,360], s = [0,1], v = [0,1]
	// if s == 0, then h = -1 (undefined)
	static void pixel_to_hsv(
		float r, float g, float b,
		float &h, float &s, float &v
	){
		float min, max, delta;
		min_max(r, g, b, min, max);

		v = max;
		delta = max - min;

		if(max != 0){
			s = delta / max; // Saturation.
		}else{
			s = 0;
			h = -1; //TODO Maybe to return max value (360).
			return;
		}

		if(r == max){
			h = ( g - b ) / delta; // between yellow & magenta
		}else if(g == max){
			h = 2 + ( b - r ) / delta; // between cyan & yellow
		}else{
			h = 4 + ( r - g ) / delta; // between magenta & cyan
		}

		h *= 60; // Hue is in degrees.
		if(h < 0){
			h += 360;
		}
	}

	static void rgb_to_hsv(
		InputArray src,
		OutputArray dst
	) {
		dst.create(src.size(), src.type());
		
		Mat s = src.getMat();
		Mat& d = dst.getMatRef();
		
		for(int r = 0; r < s.rows; r++){
			for(int c = 0; c < s.cols; c++){

				float R, G, B; // [0,255]
				float S = 0, V = 0; // [0,255]
				float H = 0; // [0,180]
				
				Vec3b BGR = s.at<Vec3b>(r, c);
				
				B = BGR.val[0];
				G = BGR.val[1];
				R = BGR.val[2];

				pixel_to_hsv(R/255, G/255, B/255, H, S, V);
				H /= 2;
				S *= 255;
				V *= 255;

				d.at<Vec3b>(r, c).val[0] = H;
				d.at<Vec3b>(r, c).val[1] = S;
				d.at<Vec3b>(r, c).val[2] = V;
			}
		}
	}
	
	void cvtColor(
		InputArray src,
		OutputArray dst,
		int code,
		int dstCn
	) {
		if(code != COLOR_BGR2HSV && dstCn != 0){
			throw logic_error("Not implemented, nor it will be!");
		}
		
		rgb_to_hsv(src, dst);
	}
}

/*
static void test_hsv(Mat hsv, Mat my, Mat original){	// ISPISUJE VREDNSOTI ZA OVE SLIKE U ODREDJENOM PIXELU

	int a;
	int b;

	cin >> a;
	cin >> b;

	cout << "O\tR: " << int(original.at<Vec3b>(a, b).val[2]);
	cout << "\tG: " << int(original.at<Vec3b>(a, b).val[1]);
	cout << "\tB: " << int(original.at<Vec3b>(a, b).val[0]) << endl;

	cout << "H\tH: " << int(hsv.at<Vec3b>(a, b).val[0]); 
	cout << "\tS: " << int(hsv.at<Vec3b>(a, b).val[1]);
	cout << "\tV: " << int(hsv.at<Vec3b>(a, b).val[2]) << endl;

	cout << "M\tH: " << int(my.at<Vec3b>(a, b).val[0]);
	cout << "\tS: " << int(my.at<Vec3b>(a, b).val[1]);
	cout << "\tV: " << int(my.at<Vec3b>(a, b).val[2]) << endl;
}



static void in_range(Mat &in, int H_d, int S_d, int V_d, int H_u, int S_u, int V_u){ //UZIMA PIKSELE KOJI ODGOVARAJU OPSEGU I SETUJE IH NA 0 ili 1 
	Mat range(in.size(),CV_8U);

	for (int i = 0; i < in.cols; i++) {
		for (int j = 0; j < in.rows; j++) {
			
			float H = in.at<Vec3b>(j, i).val[0];
			float S = in.at<Vec3b>(j, i).val[1];
			float V = in.at<Vec3b>(j, i).val[2];

			if (H_d <= H & H <= H_u & S_d <= S & S <= S_u & V_d <= V & V <= V_u) {

				range.at<unsigned char>(j, i) = 0;
				//cout << "da" << endl;
			}
			else {
				range.at<unsigned char>(j, i) = 255;
				//cout << "ne" << endl;
			}
		}
	}

	in = range.clone();
}


static void erode_image(Mat& image) {

	Mat erode = image.clone();

	for (int i = 1; i < image.cols-1; i++) {
		for (int j = 1; j < image.rows-1; j++) {

			int min = 255;

			for (int m = -1; m <= 1 ; m++) {
				for (int n = -1; n <= 1; n++) {

					int value = image.at<unsigned char>(j + m, i + n);
					if (value < min) {
						min = value;
					}

				}
			}

			erode.at<unsigned char>(j, i) = min;

		}
	}

	image = erode.clone();
}


static void dilate_image(Mat& image) {

	Mat dilate = image.clone();

	for (int i = 1; i < image.cols - 1; i++) {
		for (int j = 1; j < image.rows - 1; j++) {

			int max = -1;

			for (int m = -1; m <= 1; m++) {
				for (int n = -1; n <= 1; n++) {

					int value = image.at<unsigned char>(j + m, i + n);
					if (value > max) {
						max = value;
					}

				}
			}

			dilate.at<unsigned char>(j, i) = max;

		}
	}

	image = dilate.clone();
}



static void border_interpolate(Mat& image, int size) {
	
	int size2 = size *2;

	Mat median (Size(image.cols + size2, image.rows + size2), CV_8U);

	for (int i = 0; i < median.cols; i++) {
		for (int j = 1; j < median.rows; j++) {

			if ( i >= size && j >= size && i < (median.cols - size) && j < (median.rows - size) ){
				median.at<unsigned char>(j, i) = image.at<unsigned char>(j-size, i-size);
			}
			else{
				median.at<unsigned char>(j, i) = 255;
			}
		}
	}

	image = median.clone();
}

void insertion_sort(int arr[], int n)
{
	int i, key, j;
	for (i = 1; i < n; i++) {
		key = arr[i];
		j = i - 1;


		while (j >= 0 && arr[j] > key) {
			arr[j + 1] = arr[j];
			j = j - 1;
		}
		arr[j + 1] = key;
	}
}

static void median_filteer(Mat& image) {

	Mat border = image.clone();
	border_interpolate(border, 2);


	Mat median = image.clone();
	
	int arr[24];


	for (int i = 2; i < border.cols - 2; i++) {
		for (int j = 2; j < border.rows - 2; j++) {

			int counter = 0;

			for (int m = -2; m <= 2; m++) {
				for (int n = -2; n <= 2; n++) {

					if (m != 0 || n != 0) {
						arr[counter] = border.at<unsigned char>(j + m, i + n);
						counter++;
					}

				}
			}

			insertion_sort(arr, 24);

			int value = (arr[11]+arr[12])/2;

			//cout << value << endl;

			median.at<unsigned char>(j-2, i-2) = value;

		}
	}


	cout << "test" << endl;
	image = median.clone();

}
*/
