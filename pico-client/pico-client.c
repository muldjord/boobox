#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "servo.pio.h"
#include "dialog.h"

#define AUDIO_PIN 28 // Pin 34
PIO pio = pio0;
int sm = 0;
int rand_line = 0;

const uint PIR_PIN = 15;
const uint SERVO_PIN = 16;

/* Write `period` to the input shift register */
void pio_pwm_set_period(PIO pio, uint sm, uint32_t period) {
  pio_sm_set_enabled(pio, sm, false);
  pio_sm_put_blocking(pio, sm, period);
  pio_sm_exec(pio, sm, pio_encode_pull(false, false));
  pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
  pio_sm_set_enabled(pio, sm, true);
}

/* Write `level` to TX FIFO. State machine will copy this into X */
void pio_pwm_set_level(PIO pio, uint sm, uint32_t level) {
  pio_sm_put_blocking(pio, sm, level);
}

#define MOUTH_OPEN 1666
#define MOUTH_CLOSED 2000
int count = 0;
int span = 20;
int accumulated = 0;
bool wav_playing = false;
int wav_position = 0;
int wav_level = MOUTH_CLOSED;

void pwm_interrupt_handler() {
  pwm_clear_irq(pwm_gpio_to_slice_num(AUDIO_PIN));
  if(wav_position < (line_sizes[rand_line][0]<<3) - 1) {
    // set pwm level
    // allow the pwm value to repeat for 8 cycles this is >>3
    if(wav_position % 8 == 0) {
      if(lines[rand_line][wav_position>>3] < 127) {
	accumulated += (127 - lines[rand_line][wav_position>>3]) + 127;
      } else {
	accumulated += lines[rand_line][wav_position>>3];
      }
      count++;
      if(count == span) {
	wav_level = accumulated / span;
	/*
	  wav_level -= 50;
	  if(wav_level < 0) {
	  wav_level = 0;
	  }
	*/
	count = 0;
	accumulated = 0;
      }
    }
    pwm_set_gpio_level(AUDIO_PIN, lines[rand_line][wav_position>>3]);
    wav_position++;
  } else {
    // reset to start
    wav_position = 0;
    wav_level = MOUTH_CLOSED;
    wav_playing = false;
    pio_pwm_set_level(pio, sm, wav_level);
    irq_set_enabled(PWM_IRQ_WRAP, false);
  }
}

int main(void) {
  /* Overclocking for fun but then also so the system clock is a 
   * multiple of typical audio sampling rates.
   */
  stdio_init_all();
  set_sys_clock_khz(176000, true); 
  gpio_set_function(AUDIO_PIN, GPIO_FUNC_PWM);

  int audio_pin_slice = pwm_gpio_to_slice_num(AUDIO_PIN);

  // Setup PWM interrupt to fire when PWM cycle is complete
  pwm_clear_irq(audio_pin_slice);
  pwm_set_irq_enabled(audio_pin_slice, true);
  // set the handle function above
  irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_interrupt_handler); 

  // Setup PWM for audio output
  pwm_config config = pwm_get_default_config();
  /* Base clock 176,000,000 Hz divide by wrap 250 then the clock divider further divides
   * to set the interrupt rate. 
   * 
   * 11 KHz is fine for speech. Phone lines generally sample at 8 KHz
   * 
   * 
   * So clkdiv should be as follows for given sample rate
   *  8.0f for 11 KHz
   *  4.0f for 22 KHz
   *  2.0f for 44 KHz etc
   */
  pwm_config_set_clkdiv(&config, 8.0f); 
  pwm_config_set_wrap(&config, 250); 
  pwm_init(audio_pin_slice, &config, true);

  pwm_set_gpio_level(AUDIO_PIN, 0);

  // Servo from here
  uint offset = pio_add_program(pio, &servo_pio_program);

  float freq = 50.0f;
  uint clk_div = 64;

  servo_pio_program_init(pio, sm, offset, clk_div, SERVO_PIN);

  uint cycles = clock_get_hz(clk_sys) / (freq * clk_div);
  uint32_t period = (cycles -3) / 3;  
  pio_pwm_set_period(pio, sm, period);

  while(true) {
    if(!wav_playing && gpio_get(PIR_PIN)) {
      // TODO: Grab wave data from server here and save it to dialog array
      wav_playing = true;
      wav_level = MOUTH_CLOSED;
      irq_set_enabled(PWM_IRQ_WRAP, true);
    }
    if(wav_playing) {
      pio_pwm_set_level(pio, sm, MOUTH_CLOSED - (wav_level * 2));
    }
    sleep_ms(50);
  }
}
