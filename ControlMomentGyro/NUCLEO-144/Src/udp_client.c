#include "main.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>
#include "udp_client.h"
#include "gl_svg.h"

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

volatile uint8_t data[100];
volatile uint8_t datalen = 0;
volatile uint32_t message_count = 0;
struct udp_pcb *upcb = NULL;

volatile int megacounter = 0;


/**
  * @brief  Connect to UDP echo server
  * @param  None
  * @retval None
  */
void udp_client_connect(void) {
	ip_addr_t DestIPaddr;
	err_t err;

	/* Create a new UDP control block  */
	upcb = udp_new();

	if (upcb!=NULL) {
		/*assign destination IP address */
		IP4_ADDR( &DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3 );

		/* configure destination IP address and port */
		err = udp_connect(upcb, &DestIPaddr, UDP_SERVER_PORT);

		if (err == ERR_OK) {
			/* Set a receive callback for the upcb */
			udp_recv(upcb, udp_receive_callback, NULL);
		}
	}
}

/**
  * @brief This function is called when an UDP datagrm has been received on the port UDP_PORT.
  * @param arg user supplied argument (udp_pcb.recv_arg)
  * @param pcb the udp_pcb which received data
  * @param p the packet buffer that was received
  * @param addr the remote IP address from which the packet was received
  * @param port the remote port from which the packet was received
  * @retval None
  */
void udp_client_send(void) {
char msg[255] = {0};
float roll = GLVG_getRoll();
float yaw = GLVG_getYaw();
float pitch = GLVG_getPitch();

sprintf(msg,"%d %f %f %f\n", megacounter, roll, yaw, pitch);
uint8_t len = strlen(msg);
struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_POOL);
if (!p) return;
pbuf_take(p, (char *)msg, len);
udp_send(upcb, p);
pbuf_free(p);


	/*
	char hello[] = "hello";
	struct pbuf *p;

	if (!datalen) {
		datalen = strlen(hello);
		memcpy(data, hello, datalen);
	}

	p = pbuf_alloc(PBUF_TRANSPORT, datalen, PBUF_POOL);
	if (!p) return;
	pbuf_take(p, (char *)data, datalen);
	udp_send(upcb, p);
	pbuf_free(p);
*/
}

/**
  * @brief This function is called when an UDP datagram has been received on the port UDP_PORT.
  * @param arg user supplied argument (udp_pcb.recv_arg)
  * @param pcb the udp_pcb which received data
  * @param p the packet buffer that was received
  * @param addr the remote IP address from which the packet was received
  * @param port the remote port from which the packet was received
  * @retval None
  */
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
	// TODO: check if this routine is called from an interrput, replace the copy by a memcpy, check if we need a critical section
	for (uint8_t i=0; i<p->len; i++) {
		data[i] = pbuf_get_at(p, i);
	}
	datalen = p->len;
	message_count++;
	pbuf_free(p);
}

