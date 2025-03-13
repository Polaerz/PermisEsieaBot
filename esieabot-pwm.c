#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <pigpiod_if2.h>
#include <string.h>
#include <stdbool.h>

#define PIN_ENABLE_LEFT 24
#define PIN_ENABLE_RIGHT 4
#define PIN_BACKWARD_LEFT 25
#define PIN_BACKWARD_RIGHT 17
#define PIN_FORWARD_LEFT 23
#define PIN_FORWARD_RIGHT 22
#define JOYSTICK_MAX 32767
#define AXIS_LEFT_RIGHT 3
#define AXIS_UP_DOWN 1
#define REVERSE_LEFT_RIGHT -1 // or 1

int left_value = 0;
int right_value = 0;
int up_value = 0;
int down_value = 0;

// This function read the last joystick's event and set data to a js_event object
int read_event(int fd, struct js_event *event)
{
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event)) {
        return 0;
	}

    // Error, can't read event
    return -1;
}

int main() {
    int pi = pigpio_start(NULL, NULL);

    if (pi < 0) {
        printf("Can't start gpio lib\n");
        exit(-1);
    }

    set_mode(pi, PIN_ENABLE_LEFT, PI_OUTPUT);
    set_mode(pi, PIN_ENABLE_RIGHT, PI_OUTPUT);
    set_mode(pi, PIN_BACKWARD_LEFT, PI_OUTPUT);
    set_mode(pi, PIN_BACKWARD_RIGHT, PI_OUTPUT);
    set_mode(pi, PIN_FORWARD_LEFT, PI_OUTPUT);
    set_mode(pi, PIN_FORWARD_RIGHT, PI_OUTPUT);

    gpio_write(pi, PIN_ENABLE_LEFT, PI_HIGH);
    gpio_write(pi, PIN_ENABLE_RIGHT, PI_HIGH);

    const char *device;
    int js;
    struct js_event event;
    char quit = 0;
	

    bool joystick_connected = false;
    while (!joystick_connected) {
        device = "/dev/input/js0";

        // We open the joystick file, read only mode
        js = open(device, O_RDONLY);

        if (js == -1) {
            printf("Could not open joystick\n");
            sleep(1);
        } else {
            joystick_connected = true;
        }
    }

    // This will loop ends if the joystick is unplugged or if we want to quit
   	while (read_event(js, &event) == 0 || quit) {    
        printf("%d %d %d\n", event.type, event.number, event.value);

        // Stop
        if(event.value == 0 && event.number == AXIS_UP_DOWN) {
            printf("Stop\n");
            set_PWM_dutycycle(pi, PIN_BACKWARD_LEFT, 0);
            set_PWM_dutycycle(pi, PIN_FORWARD_LEFT, 0);
            set_PWM_dutycycle(pi, PIN_BACKWARD_RIGHT, 0);
            set_PWM_dutycycle(pi, PIN_FORWARD_RIGHT, 0);
            up_value = 0;
            down_value = 0;
        // Forward
        } else if (event.type == 2 && event.value <= -1 && event.number == AXIS_UP_DOWN) {
            up_value = abs(event.value) * 255 / JOYSTICK_MAX;
            down_value = 0;
            printf("Forward %d\n", up_value);
        // Backward
        } else if (event.type == 2 && event.value >= 1 && event.number == AXIS_UP_DOWN) {
            up_value = 0;
            down_value = abs(event.value) * 255 / JOYSTICK_MAX;
            printf("Backward %d\n", down_value);
        // Left & right
        } else if (event.type == 2 && event.number == AXIS_LEFT_RIGHT) {
            int left_right_value = REVERSE_LEFT_RIGHT * event.value * 128 / JOYSTICK_MAX;
            printf("Left/right %d\n", left_right_value);
            if(left_right_value > 0) {
                left_value = abs(left_right_value);
                right_value = 0;
            } else if(left_right_value < 0) {
                left_value = 0;
                right_value = abs(left_right_value);
            } else {
                left_value = 0;
                right_value = 0;
            }
        }

        if(up_value > 0) {
            set_PWM_dutycycle(pi, PIN_BACKWARD_LEFT, 0);
            set_PWM_dutycycle(pi, PIN_BACKWARD_RIGHT, 0);
            set_PWM_dutycycle(pi, PIN_FORWARD_LEFT, up_value - left_value);
            set_PWM_dutycycle(pi, PIN_FORWARD_RIGHT, up_value - right_value);
        } else if (down_value > 0) {
            set_PWM_dutycycle(pi, PIN_FORWARD_LEFT, 0);
            set_PWM_dutycycle(pi, PIN_FORWARD_RIGHT, 0);
            set_PWM_dutycycle(pi, PIN_BACKWARD_LEFT, down_value - left_value);
            set_PWM_dutycycle(pi, PIN_BACKWARD_RIGHT, down_value - right_value);
        } else {
            set_PWM_dutycycle(pi, PIN_BACKWARD_LEFT, left_value);
            set_PWM_dutycycle(pi, PIN_BACKWARD_RIGHT, right_value);
            set_PWM_dutycycle(pi, PIN_FORWARD_LEFT, right_value);
            set_PWM_dutycycle(pi, PIN_FORWARD_RIGHT, left_value);
        }

        fflush(stdout);
    }

    gpio_write(pi, PIN_ENABLE_LEFT, PI_LOW);
    gpio_write(pi, PIN_ENABLE_RIGHT, PI_LOW);

	pigpio_stop(pi);

    return 0;
}