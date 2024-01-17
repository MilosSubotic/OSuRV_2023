#include <stdint.h> // uint16_t and family
#include <stdio.h> // printf and family
#include <unistd.h> // file ops
#include <fcntl.h> // open() flags
#include <string.h> // strerror()
#include <errno.h> // errno

#include "motor_ctrl.h"
#include "libenjoy.h"

#define ANGLE_CHANGE 1

int main()
{
	int duty, r;
	int selectedServo = 0;

	uint16_t duties[MOTOR_CLTR__N_SERVO] = { 500 };
	
	int fd;
	fd = open(DEV_FN, O_RDWR);
	if(fd < 0){
		fprintf(stderr, "ERROR: \"%s\" not opened!\n", DEV_FN);
		fprintf(stderr, "fd = %d %s\n", fd, strerror(-fd));
		return 4;
	}
	
	libenjoy_context *ctx = libenjoy_init();  // initialize the library
    libenjoy_joy_info_list *info;

	// Updates internal list of joysticks. If you want auto-reconnect
    // after re-plugging the joystick, you should call this every 1s or so
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

		
		//////////////  PROGRAM  //////////////

		while(1)
		{
			while(libenjoy_poll(ctx, &ev))
            {
                switch(ev.type)
				{
					//analog
					case LIBENJOY_EV_AXIS:
					{
						if(ev.part_id == 1) //levi analog
						{
							
							while(ev.data == 32767) //dole
							{
								if(duties[selectedServo] > ANGLE_CHANGE * 400 / 90)
									duties[selectedServo] -= ANGLE_CHANGE * 400 / 90;
								else
									continue;

								printf("%d", duties[selectedServo]);

								r = write(fd, (char*)&duties, sizeof(duties));
								if(r != sizeof(duties))
								{
									fprintf(stderr, "ERROR: write went wrong!\n");
									return 4;
								}

								usleep(100000);
							}
								
							while(ev.data == -32767) //gore
							{
								if(duties[selectedServo] < 900 - ANGLE_CHANGE * 400 / 90)
									duties[selectedServo] += ANGLE_CHANGE * 400 / 90;
								else
									continue;

								printf("%d", duties[selectedServo]);
								
								r = write(fd, (char*)&duties, sizeof(duties));
								if(r != sizeof(duties)){
									fprintf(stderr, "ERROR: write went wrong!\n");
									return 4;
								}

								usleep(100000);
							}
						}
						
						break;
					}

					
					//buttons
					case LIBENJOY_EV_BUTTON:
					{
						if(ev.part_id == 4) //menjaj motore unapred
						{
							if(selectedServo < 3)
								selectedServo++;
						}
						else if(ev.part_id == 6) //menjaj motore unazad
						{
							if(selectedServo > 0)
								selectedServo--;
						}

						break;
					}

				}
			}

        	usleep(50000);

        	counter += 50;
        	if(counter >= 1000)
        	{
        	    libenjoy_enumerate(ctx);
        	    counter = 0;
        	}
        }



		/////////////////////////////////////////////



		// Joystick is really closed in libenjoy_poll or libenjoy_close,
            // because closing it while libenjoy_poll is in process in another thread
            // could cause crash. Be sure to call libenjoy_poll(ctx, NULL); (yes,
            // you can use NULL as event) if you will not poll nor libenjoy_close
            // anytime soon.
		libenjoy_close_joystick(joy);
        
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
	close(fd);

	printf("End.\n");

	return 0;
}
