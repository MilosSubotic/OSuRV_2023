#include <stdio.h>
#ifdef __linux
  #include <unistd.h>
#else
  #include <windows.h>
#endif

#include "../src/libenjoy.h"


#include <stdint.h> // uint16_t and family
#include <stdbool.h> // bool
#include <stdio.h> // printf and family
#include <string.h> // strerror()
#include <unistd.h> // file ops
#include <fcntl.h> // open() flags
#include <sys/ioctl.h> // ioctl()
#include <errno.h> //errno

#include "/home/rtrk/Desktop/OSuRV_2023/OSuRV_2023/SW/Driver/motor_ctrl/include/motor_ctrl.h"// dev/motor_ctrl


//Funkcija za konvertovanje brojeva joypad-a u duty kod servomotora
int joypad2duty(int joyNum){


	// Provera da li je vrednost unutar opsega
	if (joyNum < -32767 || joyNum > 32767) {
		fprintf(stderr, "Vrednost van opsega (-32767 do 32767)\n");
		return -1; // Indikacija greške
	}

	// Linearno preslikavanje
	int novo_min = 350;
	int novo_max = 650;
	int staro_min = -32767;
	int staro_max = 32767;

	int duty = (joyNum - staro_min) * (novo_max - novo_min) / (staro_max - staro_min) + novo_min;
	return duty;

}




//Funkija za konvertovanje brojeva joypada-a u speed kod bldc motora
int joypad2speed(int joyNum){

	// Provera da li je vrednost unutar opsega
	if (joyNum < -32767 || joyNum > 0) {
		fprintf(stderr, "Vrednost van opsega (-32767 do 0)\n");
		return -1; // Indikacija greške
	}

	// Linearno preslikavanje
	int novo_min = 0;
	int novo_max = 100;
	int staro_min = -32767;
	int staro_max = 0;

	int speed = (joyNum - staro_min) * (novo_max - novo_min) / (staro_max - staro_min) + novo_min;
	return speed;


}



//Funkcija za upravljacki servo za skretanje
int runServo(int duty){
	
	uint16_t duties[MOTOR_CLTR__N_SERVO] = {0};
	int servo_idx = 1; //Servo motor je na channel 1

	printf("duty = %d\n", duty);
	duties[servo_idx] = duty; // [permilles]
	
	for(int i = 0; i < MOTOR_CLTR__N_SERVO; i++){
		printf("duties[%d] = %d\n", i, duties[i]);
	}
	
	int fd;
	int r = write(fd, (char*)&duties, sizeof(duties));
	if(r != sizeof(duties)){
		fprintf(stderr, "ERROR: write went wrong!\n");
		return 4;
		
	}
	return 0;


}

//Funkcija run_bldc
int runBLDC(int speed){ //BLDC je channel 0 ostali su servo

	const uint16_t moduo = 20; // 5kHz
	printf("moduo = %d\n", moduo);
	
	motor_ctrl__ioctl_arg_moduo_t ia;
	ia.ch = 0;
	ia.moduo = moduo;
	int fd;
	int r = ioctl(fd, IOCTL_MOTOR_CLTR_SET_MODUO, *(unsigned long*)&ia);
	if(r){

		return 4; //ioctl went wrong returning
	}
	
	int16_t duty[2];
	
	// |speed| = [0, 100] -> threshold = [10, 0].
	// it will be <<1 in write() so then will be [20, 0].
	uint8_t abs_speed;
	uint16_t threshold;
	if(speed > 0){
		abs_speed = speed;
		threshold = (100-abs_speed)/10;
		duty[0] = threshold;
	}else{
		abs_speed = -speed;
		threshold = (100-abs_speed)/10;
		duty[0] = -threshold;
	}
	
	printf("threshold = %d\n", threshold);
	printf("duty = %d\n", duty[0]);
	
	// Write just channel 0, which is BLDC.
	// Channel 1 is servo and here is not changed.
	
	int s = sizeof(duty[0])*1;
	r = write(fd, (char*)&duty, s);
	if(r != s){

		return 4; //write went wrong
	}
	
	
	return 0;

}



// This tels msvc to link agains winmm.lib. Pretty nasty though.
#pragma comment(lib, "winmm.lib")

int main()
{

    //Faktori ispune
    int speed;
    int duty;
	
    //flags
    int flagBLDC;
    int flagServo;
	

    libenjoy_context *ctx = libenjoy_init(); // initialize the library
    libenjoy_joy_info_list *info;

    libenjoy_enumerate(ctx);

    info = libenjoy_get_info_list(ctx);

    if(info->count != 0) // just get the first joystick
    {
        libenjoy_joystick *joy;
        printf("Opening joystick %s...", info->list[0]->name);
 
        joy = libenjoy_open_joystick(ctx, info->list[0]->id);
        if(joy)
        {
            int counter = 0;
            libenjoy_event ev;

            printf("Success!\n");
            printf("Axes: %d btns: %d\n", libenjoy_get_axes_num(joy),libenjoy_get_buttons_num(joy));

            //Inicijalizovati vrednosti faktora ispune za sve motore 
	    uint16_t duties[MOTOR_CLTR__N_SERVO] = {0};
		
            //Otvoriti fd 
	    int fd;
	    fd = open(DEV_FN, O_RDWR); //dev/motor_ctrl
	    if(fd < 0){

		fprintf(stderr, "ERROR: \"%s\" not opened!\n", DEV_FN);
		fprintf(stderr, "fd = %d %s\n", fd, strerror(-fd));
		return 4;
	    }


            
            while(1)
            {
                // Value data are not stored in library. if you want to use
                // them, you have to store them

                // That's right, only polling available
                while(libenjoy_poll(ctx, &ev))
                {
                    switch(ev.type)
                    {
                    case LIBENJOY_EV_AXIS:
                        printf("%u: axis %d val %d\n", ev.joy_id, ev.part_id, ev.data);

                        if(ev.part_id == 1){ //axis 1 - y-osa

                            speed = joypad2speed(ev.data);
                            
                            //poziva se BLDC motor (pogon) - runBLDC(povratna vrednost od joypad2speed)
			    flagBLDC = runBLDC(speed);

			    if(flagBLDC == 4){

			    	fprintf(stderr, "ERROR: write went wrong!\n");			
		 	    }
				
                        }else if(ev.part_id == 2){ //axis 2 - x-osa

                            duty = joypad2duty(ev.data);

                            //poziva se servo motor za skretanje levo - desno
			    flagServo = runServo(duty);

			    if(flagServo == 4){
					
				fprintf(stderr, "ERROR: write went wrong!\n");	

			    }
                        }


                        break;
                    case LIBENJOY_EV_BUTTON:
                        printf("%u: button %d val %d\n", ev.joy_id, ev.part_id, ev.data);
                        break;
                    case LIBENJOY_EV_CONNECTED:
                        printf("%u: status changed: %d\n", ev.joy_id, ev.data);
                        break;
                    }
                }
#ifdef __linux
                usleep(50000);
#else
                Sleep(50);
#endif
                counter += 50;
                if(counter >= 1000)
                {
                    libenjoy_enumerate(ctx);
                    counter = 0;
                }
            }

            //Zatvoriti fd
		
	    lseek(fd, SEEK_SET, 0); // Seek on start.

	    close(fd);

            libenjoy_close_joystick(joy);

	    printf("End.\n");
        }
        else
            printf("Failed!\n");
    }
    else
        printf("No joystick available\n");

    // Frees memory allocated by that joystick list. Do not forget it!
    libenjoy_free_info_list(info);

    // deallocates all memory used by lib. Do not forget this!
    // libenjoy_poll must not be called during or after this call
    libenjoy_close(ctx);
    return 0;
}
