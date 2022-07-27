#include <stdio.h>
#include <string.h>
#include <time.h>
#include "pico/stdlib.h"
#include "pico/time.h"
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

#define TCP_PORT 4242
#define BUF_SIZE 64000 // 64 kb max

typedef struct TCP_CLIENT_T_ {
  struct tcp_pcb *tcp_pcb;
  ip_addr_t remote_addr;
  uint8_t buffer[BUF_SIZE];
  int buffer_len;
  bool complete;
} TCP_CLIENT_T;

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
  err_t err = tcp_connect(tcpconn->tcp_pcb, &tcpconn->remote_addr, TCP_PORT, tcp_client_connected);
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

TCP_CLIENT_T *tcpconn = NULL;
#define AUDIO_PIN 28 // GP28, Physical pin 34
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

/* Write `level` to TX FIFO. Tcpconn machine will copy this into X */
void pio_pwm_set_level(PIO pio, uint sm, uint32_t level) {
  pio_sm_put_blocking(pio, sm, level);
}

#define MOUTH_OPEN 1666
#define MOUTH_CLOSED 2000
int count = 0;
int span = 20;
int high_value = 0;
int low_value = 255;
bool wav_playing = false;
int wav_position = 0;
int servo_value = MOUTH_CLOSED;

void pwm_interrupt_handler() {
  pwm_clear_irq(pwm_gpio_to_slice_num(AUDIO_PIN));
  if(wav_position < (tcpconn->buffer_len<<3)) {
    // allow the pwm value to repeat for 8 cycles this is >>3
    if(wav_position % 8 == 0) {
      int sample_value = 0;
      if(tcpconn->buffer[wav_position>>3] < 127) { // Inverse samples pointing 'downwards' (below 127)
	sample_value = (127 - tcpconn->buffer[wav_position>>3]) + 127;
      } else {
	sample_value = tcpconn->buffer[wav_position>>3];
      }
      sample_value -= 127;
      if(sample_value > high_value) {
	high_value = sample_value;
      }
      if(sample_value < low_value) {
	low_value = sample_value;
      }
      count++;
      if(count == span) {
	servo_value = (low_value + high_value) / 2;
	count = 0;
	high_value = 0;
	low_value = 255;
      }
    }
    pwm_set_gpio_level(AUDIO_PIN, tcpconn->buffer[wav_position>>3]);
    wav_position++;
  } else {
    // reset to start
    wav_position = 0;
    servo_value = MOUTH_CLOSED;
    wav_playing = false;
    pio_pwm_set_level(pio, sm, servo_value);
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
    if(state == 0) {
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
      if(gpio_get(PIR_PIN)) {
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
	servo_value = MOUTH_CLOSED;
	irq_set_enabled(PWM_IRQ_WRAP, true);
      }
    }
    if(state == 2) {
      if(wav_playing) {
	// FIXME: Turn 2.6 factor into a variable calculated from sample data
	pio_pwm_set_level(pio, sm, MOUTH_CLOSED - (servo_value * 2.6));
      } else {
	state = 0;
      }
    }
    // FIXME: Due to this sleep the mouth movement is always behind the sound. Find way to sample ahead 100 ms for the servo
    // I could add 100 ms samples to the beginning of the sample data. Then always calculate servo value from + 100 ms samples ahead
    // while the sound still plays from wav_position>>3
    sleep_ms(100);
  }
  cyw43_arch_deinit();
  free(tcpconn);
  return 0;
}
