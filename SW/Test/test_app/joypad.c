
/*

1. Inicijalizacija libenjoy biblioteke.
2. Dobavlja listu dostupnih joystick-a i ispisuje njihova imena.
3. Otvara prvi joystick iz liste.
4. Ulazi u main event loop za rad sa joystick-om.
5. Ispisuje informacije za axis, button i connection dogadjaje.
6. Error handling & cleanup.

*/

#include <stdio.h>
#include <unistd.h>
#include <fntcl.h> 
#include "libenjoy.h"
#include "motor_ctrl.h"



void handleAxisEvent(const libenjoy_event& event) {
    printf("Joystick %u: Axis %d - Value %d\n", event.joy_id, event.part_id, event.data);
    //Prosiriti
    
    
}

void handleButtonEvent(const libenjoy_event& event) {
    printf("Joystick %u: Button %d - Value %d\n", event.joy_id, event.part_id, event.data);
    //Prosiriti
}

void handleConnectionEvent(const libenjoy_event& event) {
    printf("Joystick %u: Connection status changed - %d\n", event.joy_id, event.data);
    //Prosiiriti
}

int main() {
    libenjoy_context* ctx = libenjoy_init(); // inicijalizacija biblioteke

    if (!ctx) {
        fprintf(stderr, "Error initializing libenjoy.\n");
        return 1;
    }

    libenjoy_joy_info_list* info = libenjoy_get_info_list(ctx); // uzimanje liste dostupnih joystick-a

    if (info->count == 0) { // ako nijedan nije dostupan
        printf("No joysticks available.\n");
        libenjoy_free_info_list(info);
        libenjoy_close(ctx);
        return 0;
    }

    printf("Available Joysticks:\n"); // ispisivanje dostupnih joystick-a
    for (uint32_t i = 0; i < info->count; ++i) {
        printf("%u: %s\n", i + 1, info->list[i]->name);
    }

    // Otvaranje prvog dostupnog 
    libenjoy_joystick* joy = libenjoy_open_joystick(ctx, info->list[0]->id);

    if (!joy) {
        printf("Failed to open the joystick.\n");
        libenjoy_free_info_list(info);
        libenjoy_close(ctx);
        return 1;
    }

    printf("Opened joystick: %s\n", info->list[0]->name);
    printf("Axes: %d Buttons: %d\n", libenjoy_get_axes_num(joy), libenjoy_get_buttons_num(joy));

    // Main event loop
    while (1) {
        libenjoy_event ev;

        while (libenjoy_poll(ctx, &ev)) {
            switch (ev.type) {
                case LIBENJOY_EV_AXIS:
                    handleAxisEvent(ev);
                    break;
                case LIBENJOY_EV_BUTTON:
                    handleButtonEvent(ev);
                    break;
                case LIBENJOY_EV_CONNECTED:
                    handleConnectionEvent(ev);
                    break;
            }
        }

        // Sleep for a short duration to control the loop speed
        usleep(50000);
    }

    // Cleanup
    libenjoy_close_joystick(joy);
    libenjoy_free_info_list(info);
    libenjoy_close(ctx);

    return 0;
}
