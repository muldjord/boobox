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
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
//#include "hardware/irq.h"
//#include "hardware/pwm.h"
//#include "hardware/sync.h"
#include "hardware/clocks.h"
#include "hardware/structs/clocks.h" // Needed by i2s
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "pico/audio_i2s.h"
#include "pico/binary_info.h"

bi_decl(bi_3pins_with_names(PICO_AUDIO_I2S_DATA_PIN, "I2S DIN", PICO_AUDIO_I2S_CLOCK_PIN_BASE, "I2S BCK", PICO_AUDIO_I2S_CLOCK_PIN_BASE+1, "I2S LRCK"));

#include "servo.pio.h"

//#define SINE_WAVE_TABLE_LEN 2048
#define SAMPLES_PER_BUFFER 256

//static int16_t sine_wave_table[SINE_WAVE_TABLE_LEN];

struct audio_buffer_pool *init_audio() {

  static audio_format_t audio_format = {
    .format = AUDIO_BUFFER_FORMAT_PCM_U8,
    //.sample_freq = 24000,
    .sample_freq = 11025,
    .channel_count = 1,
  };

  static struct audio_buffer_format producer_format = {
    .format = &audio_format,
    .sample_stride = 2
  };

  struct audio_buffer_pool *producer_pool = audio_new_producer_pool(&producer_format, 3, SAMPLES_PER_BUFFER); // todo correct size
  bool __unused ok;
  const struct audio_format *output_format;
  struct audio_i2s_config config = {
    .data_pin = PICO_AUDIO_I2S_DATA_PIN,
    .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
    .dma_channel = 0,
    .pio_sm = 0,
  };

  output_format = audio_i2s_setup(&audio_format, &config);
  if (!output_format) {
    panic("PicoAudio: Unable to open audio device.\n");
  }

  ok = audio_i2s_connect(producer_pool);
  assert(ok);
  audio_i2s_set_enabled(true);
  return producer_pool;
}


// NOTE!!! See pico-playground example for... well, an example

// Motion sensor related
const int pir_pin = 22; // GP22

// Servo related
const int servo_pin = 21; // GP26
const PIO pio = pio0;
const int state_machine = 0;
const int mouth_open = 1666; // Currently unused
const int mouth_closed = 2000;
const int read_ahead = 800; // Number of samples to read ahead of audio for better sync
int servo_value = mouth_closed;

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

// Audio related
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
  /*
  if(tcpconn->buffer_len > 200) {
    tcpconn->buffer_len -= 200;
  }
  */
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

// Main
int main(void) {
  stdio_init_all();

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
    printf("No wifi!\n");
    return 0;
  }
  // Wifi end

  // Audio i2s begin
  printf("Initializing i2s audio!\n");
  struct audio_buffer_pool *ap = init_audio();
  printf("Audio ok!\n");
  // Audio i2s end

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
      struct audio_buffer *buffer = take_audio_buffer(ap, true);
      for (uint i = 0; i < buffer->max_sample_count; i++) {
	if(wav_position < tcpconn->buffer_len) {
	  buffer->buffer->bytes[i] == tcpconn->buffer[i];
	} else {
	  buffer->buffer->bytes[i] == 127;
	}
      }
      buffer->sample_count = buffer->max_sample_count;
      wav_position += buffer->sample_count;
      give_audio_buffer(ap, buffer);
      if(wav_position > tcpconn->buffer_len - read_ahead) {
	wav_playing = false;
	wav_position = 0;
	servo_value = mouth_closed;
	pio_pwm_set_level(pio, state_machine, servo_value);
      }
      if(wav_playing) {
	int sample_value = 0;
	int high_value = 0;
	int low_value = 255;
	int span = 40;
	// Read ahead 'read_ahead' samples to get better audio sync with jaw movement
	for(int samples = read_ahead; samples < read_ahead + span; ++samples) {
	  if(tcpconn->buffer[(wav_position) + ((wav_position) + samples >= tcpconn->buffer_len?tcpconn->buffer_len - (wav_position):samples)] < 127) { // Inverse samples pointing 'downwards' (below 127)
	    sample_value = (127 - tcpconn->buffer[(wav_position) + ((wav_position) + samples >= tcpconn->buffer_len?tcpconn->buffer_len - (wav_position):samples)]) + 127;
	  } else {
	    sample_value = tcpconn->buffer[(wav_position) + ((wav_position) + samples >= tcpconn->buffer_len?tcpconn->buffer_len - (wav_position):samples)];
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
	// FIXME: Turn 2.6 factor into a variable calculated from sample data
	pio_pwm_set_level(pio, state_machine, mouth_closed - (servo_value * 2.6));
      } else {
	state = 0;
      }
    }
    sleep_ms(100);
  }
  cyw43_arch_deinit();
  free(tcpconn);
  return 0;
}
/*
int main(void) {
  stdio_init_all();
  for (int i = 0; i < SINE_WAVE_TABLE_LEN; i++) {
    sine_wave_table[i] = 32767 * cosf(i * 2 * (float) (M_PI / SINE_WAVE_TABLE_LEN));
  }
  struct audio_buffer_pool *ap = init_audio();
  uint32_t step = 0x200000;
  uint32_t pos = 0;
  uint32_t pos_max = 0x10000 * SINE_WAVE_TABLE_LEN;
  uint vol = 128;
  while (true) {
    int c = getchar_timeout_us(0);
    if (c >= 0) {
      if (c == '-' && vol) vol -= 4;
      if ((c == '=' || c == '+') && vol < 255) vol += 4;
      if (c == '[' && step > 0x10000) step -= 0x10000;
      if (c == ']' && step < (SINE_WAVE_TABLE_LEN / 16) * 0x20000) step += 0x10000;
      if (c == 'q') break;
      printf("vol = %d, step = %d      \r", vol, step >> 16);
    }
    struct audio_buffer *buffer = take_audio_buffer(ap, true);
    int16_t *samples = (int16_t *) buffer->buffer->bytes;
    for (uint i = 0; i < buffer->max_sample_count; i++) {
      samples[i] = (vol * sine_wave_table[pos >> 16u]) >> 8u;
      pos += step;
      if (pos >= pos_max) pos -= pos_max;
    }
    buffer->sample_count = buffer->max_sample_count;
    give_audio_buffer(ap, buffer);
  }
  puts("\n");
  return 0;
}
*/
