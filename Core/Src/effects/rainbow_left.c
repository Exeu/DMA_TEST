#include "led_driver.h"
#include "effects/rainbow_left.h"

uint16_t effStep = 0;

uint8_t rainbow_effect_left() {
	float factor1, factor2;
	uint16_t ind;
	for(uint16_t j=0;j<NUM_PIXELS;j++) {
	ind = effStep + j * 1.625;
	switch((int)((ind % 13) / 4.333333333333333)) {
	  case 0: factor1 = 1.0 - ((float)(ind % 13 - 0 * 4.333333333333333) / 4.333333333333333);
			  factor2 = (float)((int)(ind - 0) % 13) / 4.333333333333333;
			  led_set_RGB(j, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2);
			  break;
	  case 1: factor1 = 1.0 - ((float)(ind % 13 - 1 * 4.333333333333333) / 4.333333333333333);
			  factor2 = (float)((int)(ind - 4.333333333333333) % 13) / 4.333333333333333;
			  led_set_RGB(j, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2);
			  break;
	  case 2: factor1 = 1.0 - ((float)(ind % 13 - 2 * 4.333333333333333) / 4.333333333333333);
			  factor2 = (float)((int)(ind - 8.666666666666666) % 13) / 4.333333333333333;
			  led_set_RGB(j, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2);
			  break;
	}
	}
	if(effStep >= 13) {effStep=0; return 0x03; }
	else effStep++;

	led_render();
	return 0x01;
}
