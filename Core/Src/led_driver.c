#include "stm32f4xx_hal.h"
#include "led_driver.h"

extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_tim1_ch1;

// LED color buffer
uint8_t rgb_arr[NUM_BYTES] = {0};

// LED write buffer
#define WR_BUF_LEN (NUM_BPP * 8 * 2)
uint16_t wr_buf[WR_BUF_LEN] = {0};
uint_fast8_t wr_buf_p = 0;

// brightness 0 - 45
int brightness = 0;


static inline uint8_t scale8(uint8_t x, uint8_t scale) {
  return ((uint16_t)x * scale) >> 8;
}

void set_brightness(int bright) {
	if (bright == brightness) {
		return;
	}

	brightness = bright;
}

int get_brightness() {
	return brightness;
}

void led_set_RGB(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
  float angle = 90-brightness;  // in degrees
  angle = angle*PI / 180;  // in rad
  rgb_arr[3 * index]= (g)/(tan(angle)); // g;
  rgb_arr[3 * index + 1] = (r)/(tan(angle));
  rgb_arr[3 * index + 2] = (b)/(tan(angle));

}

void led_render() {
  //for(uint8_t i = 0; i < WR_BUF_LEN; ++i) wr_buf[i] = 0;

  if(wr_buf_p != 0 || hdma_tim1_ch1.State != HAL_DMA_STATE_READY) {
	// Ongoing transfer, cancel!
	for(uint8_t i = 0; i < WR_BUF_LEN; ++i) wr_buf[i] = 0;
	wr_buf_p = 0;
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	return;
  }

  for(uint_fast8_t i = 0; i < 8; ++i) {
    wr_buf[i     ] = PWM_LO << (((rgb_arr[0] << i) & 0x80) > 0);
    wr_buf[i +  8] = PWM_LO << (((rgb_arr[1] << i) & 0x80) > 0);
    wr_buf[i + 16] = PWM_LO << (((rgb_arr[2] << i) & 0x80) > 0);
    wr_buf[i + 24] = PWM_LO << (((rgb_arr[3] << i) & 0x80) > 0);
    wr_buf[i + 32] = PWM_LO << (((rgb_arr[4] << i) & 0x80) > 0);
    wr_buf[i + 40] = PWM_LO << (((rgb_arr[5] << i) & 0x80) > 0);
  }

  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)wr_buf, WR_BUF_LEN);
  wr_buf_p = 2;
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim) {
  // DMA buffer set from LED(wr_buf_p) to LED(wr_buf_p + 1)
  if(wr_buf_p < NUM_PIXELS) {
    // We're in. Fill the even buffer
    for(uint_fast8_t i = 0; i < 8; ++i) {
      wr_buf[i     ] = PWM_LO << (((rgb_arr[3 * wr_buf_p    ] << i) & 0x80) > 0);
      wr_buf[i +  8] = PWM_LO << (((rgb_arr[3 * wr_buf_p + 1] << i) & 0x80) > 0);
      wr_buf[i + 16] = PWM_LO << (((rgb_arr[3 * wr_buf_p + 2] << i) & 0x80) > 0);
    }
    wr_buf_p++;
  } else if (wr_buf_p < NUM_PIXELS + 2) {
    // Last two transfers are resets. SK6812: 64 * 1.25 us = 80 us == good enough reset
  	//                               WS2812B: 48 * 1.25 us = 60 us == good enough reset
    // First half reset zero fill
    for(uint8_t i = 0; i < WR_BUF_LEN / 2; ++i) wr_buf[i] = 0;
    wr_buf_p++;
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	  // DMA buffer set from LED(wr_buf_p) to LED(wr_buf_p + 1)
	  if(wr_buf_p < NUM_PIXELS) {
	    // We're in. Fill the odd buffer
	    for(uint_fast8_t i = 0; i < 8; ++i) {
	      wr_buf[i + 24] = PWM_LO << (((rgb_arr[3 * wr_buf_p    ] << i) & 0x80) > 0);
	      wr_buf[i + 32] = PWM_LO << (((rgb_arr[3 * wr_buf_p + 1] << i) & 0x80) > 0);
	      wr_buf[i + 40] = PWM_LO << (((rgb_arr[3 * wr_buf_p + 2] << i) & 0x80) > 0);
	    }
	    wr_buf_p++;
	  } else if (wr_buf_p < NUM_PIXELS + 2) {
	    // Second half reset zero fill
	    for(uint8_t i = WR_BUF_LEN / 2; i < WR_BUF_LEN; ++i) wr_buf[i] = 0;
	    ++wr_buf_p;
	  } else {
	    // We're done. Lean back and until next time!
	    wr_buf_p = 0;
	    HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	  }
}
