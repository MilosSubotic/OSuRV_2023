#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int joypad[13] = {-32000,-20000,-16000,-10000, -5000,-1000, 0,1000, 5000, 10000, 16000, 20000, 32000};
int speed[13];


int joypad2speed(int joyNum){

	// Provera da li je vrednost unutar opsega
	if (joyNum < -32767 || joyNum > 32767) {
		fprintf(stderr, "Vrednost van opsega (-32767 do 0)\n");
		return -1; // Indikacija greške
	}

	// Linearno preslikavanje
	int novo_min = -100;
	int novo_max = 100;
	int staro_min = 32767;
	int staro_max = -32767;

	int speed = (joyNum - staro_min) * (novo_max - novo_min) / (staro_max - staro_min) + novo_min;
	return speed;


}


int main(){

	int abs_speed[13];
	int threshold[13];
	int duty[13];

	for(int i = 0; i < 13; i++){
		
		speed[i] = joypad2speed(joypad[i]);
		printf("%d => %d\n", joypad[i], speed[i]);
	}
	
	printf("\n-------------------\n");
	
	
	for(int i = 0; i < 13; i++){
	
		if(speed[i] > 0){
			abs_speed[i] = speed[i];
			threshold[i] = (100-abs_speed[i])/10;
			duty[i] = threshold[i];
		}else{
			abs_speed[i] = -speed[i];
			threshold[i] = (100-abs_speed[i])/10;
			duty[i] = -threshold[i];
		}
		printf("abs_speed[%d] = %d\n", i, abs_speed[i]);
		printf("threshold[%d] = %d\n",i, threshold[i]);
		printf("duty[%d] = %d\n\n", i, duty[i]);
	
	
	}


	return 0;
}
