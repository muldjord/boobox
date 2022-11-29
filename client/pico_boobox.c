#include <math.h>
#include <stdio.h>
#include <string.h>
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "i2s.h"
#include "pico/stdlib.h"

// wifi
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

// TCP socket related
#define BUF_SIZE 192000

typedef struct TCP_CLIENT_T {
  struct tcp_pcb *tcp_pcb;
  ip_addr_t remote_addr;
  uint8_t buffer[BUF_SIZE];
  uint32_t buffer_len;
  uint32_t play_head;
  bool play;
  bool complete;
} TCP_CLIENT_T;

TCP_CLIENT_T *tcpconn = NULL;

static __attribute__((aligned(8))) pio_i2s i2s;

static void dma_i2s_in_handler(void) {
  if(tcpconn->play) {
    if(*(int32_t**)dma_hw->ch[i2s.dma_ch_out_ctrl].read_addr == i2s.output_buffer) {
      for(size_t i = 0; i < AUDIO_BUFFER_FRAMES * 2; i++) {
	i2s.output_buffer[i] = tcpconn->buffer[tcpconn->play_head + (i / 2)]<<23;
      }
    } else {
      for(size_t i = 0; i < AUDIO_BUFFER_FRAMES * 2; i++) {
	(&i2s.output_buffer[STEREO_BUFFER_SIZE])[i] = tcpconn->buffer[tcpconn->play_head + (i / 2)]<<23;
      }
    }
    tcpconn->play_head += AUDIO_BUFFER_FRAMES;
    if(tcpconn->play_head >= tcpconn->buffer_len - (AUDIO_BUFFER_FRAMES * 2)) {
      tcpconn->play = false;
    }
  } else {
    // No sound is playing so just send silence to the DMA buffers
    if(*(int32_t**)dma_hw->ch[i2s.dma_ch_out_ctrl].read_addr == i2s.output_buffer) {
      for(size_t i = 0; i < AUDIO_BUFFER_FRAMES * 2; i++) {
	i2s.output_buffer[i] = 127<<23;
      }
    } else {
      for(size_t i = 0; i < AUDIO_BUFFER_FRAMES * 2; i++) {
	(&i2s.output_buffer[STEREO_BUFFER_SIZE])[i] = 127<<23;
      }
    }
  }

  dma_hw->ints0 = 1u << i2s.dma_ch_out_data;  // clear the IRQ
}

static err_t tcp_client_close(void *arg) {
  TCP_CLIENT_T *tcpconn = (TCP_CLIENT_T*)arg;
  err_t err = ERR_OK;
  if (tcpconn->tcp_pcb != NULL) {
    tcp_arg(tcpconn->tcp_pcb, NULL);
    tcp_recv(tcpconn->tcp_pcb, NULL);
    err = tcp_close(tcpconn->tcp_pcb);
    if (err != ERR_OK) {
      tcp_abort(tcpconn->tcp_pcb);
      err = ERR_ABRT;
    }
    tcpconn->tcp_pcb = NULL;
  }
  tcpconn->complete = true;
  return err;
}

bool wav_init = false;

static err_t tcp_result(void *arg, int status) {
  if (status == 0) {
    // Success
  } else {
    // Failed
  }
  return tcp_client_close(arg);
}

static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
  if (err != ERR_OK) {
    return tcp_result(arg, err);
  }
  tcp_write(tpcb, "boobox\n", 7, TCP_WRITE_FLAG_COPY);
  return ERR_OK;
}

err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
  TCP_CLIENT_T *tcpconn = (TCP_CLIENT_T*)arg;
  if(p == NULL) { // pbuf is NULL if server closed the connection, meaning transfer was completed
    printf("Wav data recieved: %d bytes\n", tcpconn->buffer_len);
    return tcp_result(arg, err);
  }
  if(p->tot_len > 0) {
    const uint32_t buffer_left = BUF_SIZE - tcpconn->buffer_len;
    //printf("tcpconn->buffer_len: %d, buffer_left: %d, p->tot_len: %d\n", tcpconn->buffer_len, buffer_left, p->tot_len);
    tcpconn->buffer_len += pbuf_copy_partial(p,
					     tcpconn->buffer + tcpconn->buffer_len,
					     p->tot_len > buffer_left ? buffer_left : p->tot_len,
					     0);
    tcp_recved(tpcb, p->tot_len);
  }

  pbuf_free(p);

  return ERR_OK;
}

static bool tcp_client_open(void *arg) {
  TCP_CLIENT_T *tcpconn = (TCP_CLIENT_T*)arg;
  tcpconn->play = false;
  tcpconn->play_head = 0;
  tcpconn->buffer_len = 0;
  tcpconn->complete = false;

  tcpconn->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(&tcpconn->remote_addr));
  if (!tcpconn->tcp_pcb) {
    return false;
  }

  tcp_arg(tcpconn->tcp_pcb, tcpconn);
  //tcp_sent(tcpconn->tcp_pcb, tcp_client_sent);
  tcp_recv(tcpconn->tcp_pcb, tcp_client_recv);

  cyw43_arch_lwip_begin();
  err_t err = tcp_connect(tcpconn->tcp_pcb, &tcpconn->remote_addr, atoi(TCP_SERVER_PORT), tcp_client_connected);
  cyw43_arch_lwip_end();

  return err == ERR_OK;
}

const uint SERVO_PIN = 22;
float clockDiv = 64;
float wrap = 39062;

void setMillis(int servoPin, float millis)
{
  printf("Setting servo to: %f\n", millis);
  pwm_set_gpio_level(servoPin, (millis/20000.f)*wrap);
}

const uint PIR_PIN = 5;

int main() {
  set_sys_clock_khz(125000, true);
  stdio_init_all();

  // PIR motion sensor init
  gpio_init(PIR_PIN);
  gpio_set_dir(PIR_PIN, GPIO_IN);

  // Servo motor init
  gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
  
  pwm_config config = pwm_get_default_config();
  
  uint64_t clockspeed = clock_get_hz(5);
  clockDiv = 64;
  wrap = 39062;
  
  while (clockspeed/clockDiv/50 > 65535 && clockDiv < 256) clockDiv += 64; 
  wrap = clockspeed/clockDiv/50;
  
  pwm_config_set_clkdiv(&config, clockDiv);
  pwm_config_set_wrap(&config, wrap);
  
  pwm_init(slice_num, &config, true);
  
  // Wifi init
  cyw43_arch_init();
  cyw43_arch_enable_sta_mode();
  cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000);

  // i2s audio init
  i2s_program_start_synched(pio0, dma_i2s_in_handler, &i2s);

  int state = 0;
  tcpconn = calloc(1, sizeof(TCP_CLIENT_T));
  ip4addr_aton(TCP_SERVER_IP, &tcpconn->remote_addr);
  while(true) {
    //printf("STATE: %d\n", state);
    if(state == 0) {
      if(gpio_get(PIR_PIN)) {
	cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
	tcp_client_open(tcpconn);
	state = 1;
      }
    }
    if(state == 1) {
      if(tcpconn->complete) {
	cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
	tcpconn->play = true;
	state = 2;
      }
      cyw43_arch_poll();
    }
    if(state == 2) {
      uint32_t servoValue = 0;
      uint32_t servoZero = 1600;
      if(tcpconn->play == false) {
	printf("Closing jaws\n");
	setMillis(SERVO_PIN, servoZero);
	state = 0;
      } else {
	uint32_t lookAhead = 300;
	uint8_t valueSpan = 100;
	for(int i = tcpconn->play_head + lookAhead; i < tcpconn->play_head + lookAhead + valueSpan; ++i) {
	  if(i >= tcpconn->buffer_len) {
	    break;
	  }
	  int32_t curBufferValue = tcpconn->buffer[i];
	  uint32_t curValue = abs(curBufferValue - 128);
	  if(curValue > servoValue) {
	    servoValue = curValue;
	  }
	}
      }
      servoValue *= 2.0;
      if(servoValue > 250) {
	servoValue = 250;
      }
      printf("servoValue: %d\n", servoValue);
      setMillis(SERVO_PIN, servoZero - (servoValue));
    }
    sleep_ms(50);
  }
  free(tcpconn);
  cyw43_arch_deinit();
  return 0;
}
