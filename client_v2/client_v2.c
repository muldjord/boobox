#include <string.h>
#include <time.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#define BUF_SIZE 2048

#define POLL_TIME_S 5

#define DUMP_BYTES(A,B)

typedef struct TCP_CLIENT_T_ {
  struct tcp_pcb *tcp_pcb;
  ip_addr_t remote_addr;
  uint8_t buffer[BUF_SIZE];
  int buffer_len;
  int sent_len;
  bool complete;
  int run_count;
  bool connected;
} TCP_CLIENT_T;

static err_t tcp_client_close(void *arg) {
  TCP_CLIENT_T *tcpstate = (TCP_CLIENT_T*)arg;
  err_t err = ERR_OK;
  if(tcpstate->tcp_pcb != NULL) {
    tcp_arg(tcpstate->tcp_pcb, NULL);
    tcp_poll(tcpstate->tcp_pcb, NULL, 0);
    tcp_sent(tcpstate->tcp_pcb, NULL);
    tcp_recv(tcpstate->tcp_pcb, NULL);
    tcp_err(tcpstate->tcp_pcb, NULL);
    err = tcp_close(tcpstate->tcp_pcb);
    if(err != ERR_OK) {
      printf("close failed %d, calling abort\n", err);
      tcp_abort(tcpstate->tcp_pcb);
      err = ERR_ABRT;
    }
    tcpstate->tcp_pcb = NULL;
  }
  return err;
}

// Called with results of operation
static err_t tcp_result(void *arg, int status) {
  TCP_CLIENT_T *tcpstate = (TCP_CLIENT_T*)arg;
  if(status == 0) {
    printf("test success\n");
  } else {
    printf("test failed %d\n", status);
  }
  tcpstate->complete = true;
  return tcp_client_close(arg);
}

static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
  TCP_CLIENT_T *tcpstate = (TCP_CLIENT_T*)arg;
  printf("tcp_client_sent %u\n", len);
  tcpstate->sent_len += len;

  if(tcpstate->sent_len >= BUF_SIZE) {

    tcpstate->run_count++;
    if(tcpstate->run_count >= 5) {
      tcp_result(arg, 0);
      return ERR_OK;
    }

    // We should receive a new buffer from the server
    tcpstate->buffer_len = 0;
    tcpstate->sent_len = 0;
    printf("Waiting for buffer from server\n");
  }

  return ERR_OK;
}

static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
  TCP_CLIENT_T *tcpstate = (TCP_CLIENT_T*)arg;
  if(err != ERR_OK) {
    printf("connect failed %d\n", err);
    return tcp_result(arg, err);
  }
  tcpstate->connected = true;
  printf("Waiting for buffer from server\n");
  return ERR_OK;
}

static err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb) {
  printf("tcp_client_poll\n");
  return tcp_result(arg, -1); // no response is an error?
}

static void tcp_client_err(void *arg, err_t err) {
  if(err != ERR_ABRT) {
    printf("tcp_client_err %d\n", err);
    tcp_result(arg, err);
  }
}

err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
  TCP_CLIENT_T *tcpstate = (TCP_CLIENT_T*)arg;
  if(!p) {
    return tcp_result(arg, -1);
  }
  cyw43_arch_lwip_check();
  if(p->tot_len > 0) {
    printf("recv %d err %d\n", p->tot_len, err);
    for (struct pbuf *q = p; q != NULL; q = q->next) {
      DUMP_BYTES(q->payload, q->len);
    }
    // Receive the buffer
    const uint16_t buffer_left = BUF_SIZE - tcpstate->buffer_len;
    tcpstate->buffer_len += pbuf_copy_partial(p, tcpstate->buffer + tcpstate->buffer_len,
					   p->tot_len > buffer_left ? buffer_left : p->tot_len, 0);
    tcp_recved(tpcb, p->tot_len);
  }
  pbuf_free(p);

  // If we have received the whole buffer, send it back to the server
  /*
  if(tcpstate->buffer_len == BUF_SIZE) {
    printf("Writing %d bytes to server\n", tcpstate->buffer_len);
    err_t err = tcp_write(tpcb, tcpstate->buffer, tcpstate->buffer_len, TCP_WRITE_FLAG_COPY);
    if(err != ERR_OK) {
      printf("Failed to write data %d\n", err);
      return tcp_result(arg, -1);
    }
  }
  */
  return ERR_OK;
}

static bool tcp_client_open(void *arg) {
  TCP_CLIENT_T *tcpstate = (TCP_CLIENT_T*)arg;
  printf("Connecting to %s port %u\n", ip4addr_ntoa(&tcpstate->remote_addr), atoi(TCP_SERVER_PORT));
  tcpstate->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(&tcpstate->remote_addr));
  if(!tcpstate->tcp_pcb) {
    printf("failed to create pcb\n");
    return false;
  }

  tcp_arg(tcpstate->tcp_pcb, tcpstate);
  tcp_poll(tcpstate->tcp_pcb, tcp_client_poll, POLL_TIME_S * 2);
  tcp_sent(tcpstate->tcp_pcb, tcp_client_sent);
  tcp_recv(tcpstate->tcp_pcb, tcp_client_recv);
  tcp_err(tcpstate->tcp_pcb, tcp_client_err);

  tcpstate->buffer_len = 0;

  err_t err = tcp_connect(tcpstate->tcp_pcb, &tcpstate->remote_addr, atoi(TCP_SERVER_PORT), tcp_client_connected);

  return err == ERR_OK;
}

// Perform initialisation
static TCP_CLIENT_T* tcp_client_init(void) {
  TCP_CLIENT_T *tcpstate = calloc(1, sizeof(TCP_CLIENT_T));
  if(!tcpstate) {
    printf("failed to allocate tcpstate\n");
    return NULL;
  }
  ip4addr_aton(TCP_SERVER_IP, &tcpstate->remote_addr);
  return tcpstate;
}

int main() {
  stdio_init_all();

  if(cyw43_arch_init()) {
    printf("failed to initialise\n");
    return 1;
  }
  cyw43_arch_enable_sta_mode();

  printf("Connecting to WiFi...\n");
  if(cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
    printf("failed to connect.\n");
    return 1;
  } else {
    printf("Connected.\n");
  }

  int state = 0;

  TCP_CLIENT_T *tcpstate = NULL;

  while(true) {
    if(state == 0) {
      tcpstate = tcp_client_init();
      if(!tcpstate) {
	return 0;
      }
      if(!tcp_client_open(tcpstate)) {
	tcp_result(tcpstate, -1);
	return 0;
      }
      while(!tcpstate->complete) {
	cyw43_arch_poll();
	sleep_ms(1);
      }
      state = 1;
    } else if(state == 1) {
      // Play wav
      free(tcpstate);
      state = 0;
    }
  }

  cyw43_arch_deinit();
  return 0;
}
