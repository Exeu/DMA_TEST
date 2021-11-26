// Peripheral usage
#include <stdio.h>
#include <stdlib.h>


#define PWM_HI (38)
#define PWM_LO (19)

// LED parameters
// #define NUM_BPP (3) // WS2812B
#define NUM_BPP (3) // SK6812
#define NUM_PIXELS (8)
#define NUM_BYTES (NUM_BPP * NUM_PIXELS)

// LED color buffer
uint8_t rgb_arr[NUM_BYTES] = {0};

// LED write buffer
#define WR_BUF_LEN (NUM_BPP * 8 * 2)
uint8_t wr_buf[WR_BUF_LEN] = {0};
uint_fast8_t wr_buf_p = 0;

static inline uint8_t scale8(uint8_t x, uint8_t scale) {
  return ((uint16_t)x * scale) >> 8;
}

// Set a single color (RGB) to index
void led_set_RGB(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
  rgb_arr[3 * index] = g; // g;
  rgb_arr[3 * index + 1] = r;
  rgb_arr[3 * index + 2] = b; // b;

}

// Shuttle the data to the LEDs!
void led_render() {
  for(uint_fast8_t i = 0; i < 8; ++i) {
    wr_buf[i     ] = PWM_LO << (((rgb_arr[0] << i) & 0x80) > 0);
    wr_buf[i +  8] = PWM_LO << (((rgb_arr[1] << i) & 0x80) > 0);
    wr_buf[i + 16] = PWM_LO << (((rgb_arr[2] << i) & 0x80) > 0);
    wr_buf[i + 24] = PWM_LO << (((rgb_arr[3] << i) & 0x80) > 0);
    wr_buf[i + 32] = PWM_LO << (((rgb_arr[4] << i) & 0x80) > 0);
    wr_buf[i + 40] = PWM_LO << (((rgb_arr[5] << i) & 0x80) > 0);
  }
  
  for (int i = 0; i < WR_BUF_LEN; i++) {
    printf("%d|", wr_buf[i]);
  }
  printf("\n");
}

int main() {
    led_set_RGB(0, (uint8_t) 250, (uint8_t)120, (uint8_t) 122);
    //led_set_RGB(1, (uint8_t) 32, (uint8_t)44, (uint8_t) 12);
    led_render();

    uint32_t color = ((120<<16) | (250<<8) | (122));
    uint16_t pwmData[24];
    uint32_t indx=0;

    for (int i=23; i>=0; i--) {
                if (color&(1<<i)) {
                    pwmData[indx] = 38;  // 2/3 of 90
                } else {
                    pwmData[indx] = 19;  // 1/3 of 90
                }
                indx++;
    }

    for (int i = 0; i < 24; i++) {
        printf("%d|", pwmData[i]);
    }

    return 0;
}

