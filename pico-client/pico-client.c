/* 
   PWM audio code originally sourced from (this will be refactored to use i2s soon though,
   but I want to give credit where credit is due while the code is still in use of course):
   rgrosset, PWM audio example repository:
   https://github.com/rgrosset/pico-pwm-audio (MIT license: https://github.com/rgrosset/pico-pwm-audio/blob/main/LICENSE)

   Servo C and PIO code and TCP client C code from official Pico SDK examples:
   https://github.com/raspberrypi/pico-examples (BSD 3-clause license:
   https://github.com/raspberrypi/pico-examples/blob/master/LICENSE.TXT)
*/

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

#include "servo.pio.h"

// Motion sensor related
const int pir_pin = 15; // GP15

// Servo related
const int servo_pin = 16; // GP16
const PIO pio = pio0;
const int state_machine = 0;
const int mouth_open = 1666; // Currently unused
const int mouth_closed = 2000;
const int read_ahead = 800; // Number of samples to read ahead of audio for better sync
int servo_value = mouth_closed;

// Audio related
const int audio_pin = 28; // GP28
const float audio_volume = 0.5; // 0.1 - 1.0
const int span = 20;
bool wav_playing = false;
int wav_position = 0;

// TCP socket related
#define BUF_SIZE 128000

typedef struct TCP_CLIENT_T_ {
  struct tcp_pcb *tcp_pcb;
  ip_addr_t remote_addr;
  uint8_t buffer[BUF_SIZE];
  int buffer_len;
  bool complete;
} TCP_CLIENT_T;

TCP_CLIENT_T *tcpconn = NULL;

static err_t tcp_client_close(void *arg) {
  TCP_CLIENT_T *tcpconn = (TCP_CLIENT_T*)arg;
  err_t err = ERR_OK;
  if (tcpconn->tcp_pcb != NULL) {
    tcp_arg(tcpconn->tcp_pcb, NULL);
    //tcp_sent(tcpconn->tcp_pcb, NULL);
    tcp_recv(tcpconn->tcp_pcb, NULL);
    err = tcp_close(tcpconn->tcp_pcb);
    if (err != ERR_OK) {
      tcp_abort(tcpconn->tcp_pcb);
      err = ERR_ABRT;
    }
    tcpconn->tcp_pcb = NULL;
  }
  // FIXME: Hack to remove noise at end
  if(tcpconn->buffer_len > 200) {
    tcpconn->buffer_len -= 200;
  }
  tcpconn->complete = true;
  return err;
}

bool wav_init = false;

static err_t tcp_result(void *arg, int status) {
  TCP_CLIENT_T *tcpconn = (TCP_CLIENT_T*)arg;
  if (status == 0) {
    // Success
  } else {
    // Failed
  }
  return tcp_client_close(arg);
}

static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
  TCP_CLIENT_T *tcpconn = (TCP_CLIENT_T*)arg;
  if (err != ERR_OK) {
    return tcp_result(arg, err);
  }
  //tcp_write(tpcb, tcpconn->buffer, tcpconn->buffer_len, TCP_WRITE_FLAG_COPY);
  tcp_write(tpcb, "murray\n", 7, TCP_WRITE_FLAG_COPY);
  return ERR_OK;
}

err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
  TCP_CLIENT_T *tcpconn = (TCP_CLIENT_T*)arg;
  if (!p) {
    return tcp_result(arg, -1);
  }
  cyw43_arch_lwip_check();
  if(p == NULL) { // pbuf is NULL if server closed the connection, meaning transfer was completed
    return tcp_result(arg, err);
  }
  if(p->tot_len > 0) {
    const uint16_t buffer_left = BUF_SIZE - tcpconn->buffer_len;
    tcpconn->buffer_len += pbuf_copy_partial(p, tcpconn->buffer + tcpconn->buffer_len,
					     p->tot_len > buffer_left ? buffer_left : p->tot_len, 0);
    tcp_recved(tpcb, p->tot_len);
    if(tcpconn->buffer_len >= BUF_SIZE) {
      return tcp_result(arg, err);
    }
  }

  pbuf_free(p);

  return ERR_OK;
}

static bool tcp_client_open(void *arg) {
  TCP_CLIENT_T *tcpconn = (TCP_CLIENT_T*)arg;
  tcpconn->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(&tcpconn->remote_addr));
  if (!tcpconn->tcp_pcb) {
    return false;
  }

  tcp_arg(tcpconn->tcp_pcb, tcpconn);
  //tcp_sent(tcpconn->tcp_pcb, tcp_client_sent);
  tcp_recv(tcpconn->tcp_pcb, tcp_client_recv);

  tcpconn->buffer_len = 0;
  memset(tcpconn->buffer, 0, sizeof(tcpconn->buffer));

  cyw43_arch_lwip_begin();
  err_t err = tcp_connect(tcpconn->tcp_pcb, &tcpconn->remote_addr, atoi(TCP_SERVER_PORT), tcp_client_connected);
  cyw43_arch_lwip_end();

  return err == ERR_OK;
}

static TCP_CLIENT_T* tcp_client_init(void) {
  TCP_CLIENT_T *tcpconn = calloc(1, sizeof(TCP_CLIENT_T));
  if (!tcpconn) {
    return NULL;
  }
  ip4addr_aton(TCP_SERVER_IP, &tcpconn->remote_addr);
  return tcpconn;
}

// PWM audio related
void pio_pwm_set_period(PIO pio, uint state_machine, uint32_t period) {
  pio_sm_set_enabled(pio, state_machine, false);
  pio_sm_put_blocking(pio, state_machine, period);
  pio_sm_exec(pio, state_machine, pio_encode_pull(false, false));
  pio_sm_exec(pio, state_machine, pio_encode_out(pio_isr, 32));
  pio_sm_set_enabled(pio, state_machine, true);
}

void pio_pwm_set_level(PIO pio, uint state_machine, uint32_t level) {
  pio_sm_put_blocking(pio, state_machine, level);
}

void pwm_interrupt_handler() {
  pwm_clear_irq(pwm_gpio_to_slice_num(audio_pin));
  if(wav_position < (tcpconn->buffer_len<<3)) {
    // allow the pwm value to repeat for 8 cycles this is >>3
    pwm_set_gpio_level(audio_pin, tcpconn->buffer[wav_position>>3] * (audio_volume <= 0.0 || audio_volume >= 1.0 ?0.5:audio_volume));
    wav_position++;
  } else {
    // reset to start
    wav_playing = false;
    wav_position = 0;
    servo_value = mouth_closed;
    pio_pwm_set_level(pio, state_machine, servo_value);
    irq_set_enabled(PWM_IRQ_WRAP, false);
  }
}

// Main
int main(void) {
  stdio_init_all();
  set_sys_clock_khz(176000, true); 
  gpio_set_function(audio_pin, GPIO_FUNC_PWM);

  int audio_pin_slice = pwm_gpio_to_slice_num(audio_pin);

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

  pwm_set_gpio_level(audio_pin, 0);

  // Servo from here
  uint offset = pio_add_program(pio, &servo_pio_program);

  float freq = 50.0f;
  uint clk_div = 64;

  servo_pio_program_init(pio, state_machine, offset, clk_div, servo_pin);

  uint cycles = clock_get_hz(clk_sys) / (freq * clk_div);
  uint32_t period = (cycles -3) / 3;  
  pio_pwm_set_period(pio, state_machine, period);

  // Wifi begin
  if (cyw43_arch_init()) {
    return 1;
  }
  cyw43_arch_enable_sta_mode();

  if(cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
    return 1;
  }

  tcpconn = tcp_client_init();
  if(!tcpconn) {
    return 0;
  }
  // Wifi end
  int state = 0;
  while(true) {
    printf("STATE: %d\n", state);
    if(state == 0) {
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
      if(gpio_get(pir_pin)) {
	if(!tcp_client_open(tcpconn)) {
	  tcp_result(tcpconn, -1);
	  return 0;
	}
	state = 1;
      }
    }
    if(state == 1) {
      if(tcpconn->complete) {
	cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
	tcpconn->complete = false;
	state = 2;
	wav_playing = true;
	servo_value = mouth_closed;
	irq_set_enabled(PWM_IRQ_WRAP, true);
      }
    }
    if(state == 2) {
      if(wav_playing) {
	// FIXME: Turn 2.6 factor into a variable calculated from sample data
	pio_pwm_set_level(pio, state_machine, mouth_closed - (servo_value * 2.6));
      } else {
	state = 0;
      }
    }
    // FIXME: Due to this sleep the mouth movement is always behind the sound. Find way to sample ahead 100 ms for the servo
    // I could add 100 ms samples to the beginning of the sample data. Then always calculate servo value from + 100 ms samples ahead
    // while the sound still plays from wav_position>>3
    if(wav_playing) {
      int sample_value = 0;
      int high_value = 0;
      int low_value = 255;
      // Read ahead 'read_ahead' samples for servo movement, but do it at steps of 8 as this is plenty to do an average
      for(int samples = 0; samples < read_ahead; samples += 8) {
	if(tcpconn->buffer[(wav_position>>3) + ((wav_position>>3) + samples + read_ahead >= tcpconn->buffer_len?tcpconn->buffer_len - (wav_position>>3):samples + read_ahead)] < 127) { // Inverse samples pointing 'downwards' (below 127)
	  sample_value = (127 - tcpconn->buffer[(wav_position>>3) + ((wav_position>>3) + samples + read_ahead >= tcpconn->buffer_len?tcpconn->buffer_len - (wav_position>>3):samples + read_ahead)]) + 127;
	} else {
	  sample_value = tcpconn->buffer[(wav_position>>3) + ((wav_position>>3) + samples + read_ahead >= tcpconn->buffer_len?tcpconn->buffer_len - (wav_position>>3):samples + read_ahead)];
	}
	sample_value -= 127;
	if(sample_value > high_value) {
	  high_value = sample_value;
	}
	if(sample_value < low_value) {
	  low_value = sample_value;
	}
      }
      servo_value = (low_value + high_value) / 2;
    }
    sleep_ms(100);
  }
  cyw43_arch_deinit();
  free(tcpconn);
  return 0;
}
