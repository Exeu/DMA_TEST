#ifndef INC_LED_DRIVER_H_
#define INC_LED_DRIVER_H_

#include <stdint.h>

#define PWM_HI (58)
#define PWM_LO (29)

#define NUM_BPP (3)
#define NUM_PIXELS (8)
#define NUM_BYTES (NUM_BPP * NUM_PIXELS)
#define PI 3.14159265


void set_brightness(int bright);
void led_set_RGB(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void led_render();

#endif /* INC_LED_DRIVER_H_ */
