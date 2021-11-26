#include "effects/rainbow_fade.h"
#include "led_driver.h"
#include "color_utils.h"

uint8_t ang = 0;
const uint8_t angle_difference = 11;

void rainbow_effect_fade() {
	for(uint8_t i = 0; i < NUM_PIXELS; i++) {
		uint32_t rgb_color = hsl_to_rgb(ang + (i * angle_difference), 255, 127);
		led_set_RGB(i, (rgb_color >> 16) & 0xFF, (rgb_color >> 8) & 0xFF, rgb_color & 0xFF);
	}
  	++ang;
  	led_render();
}
