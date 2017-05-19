/*
 * networkTaskFunction.h
 *
 *  Created on: Apr 18, 2017
 *      Author: Phanomphon Yotchon
 */

#include <stdio.h>
#include <string.h>
//
#include "lwip/sockets.h"
#include "netif/xadapter.h"
#include "lwipopts.h"
#include "xil_printf.h"
#include "FreeRTOS.h"
#include "task.h"

#ifndef SRC_NETWORKTASKFUNCTION_H_
#define SRC_NETWORKTASKFUNCTION_H_

#define THREAD_STACKSIZE 1024

u16_t server_port = 9000;
u16_t logs_server_port = 9001;

static struct netif server_netif;
struct netif *echo_netif;

// remote App
/* thread spawned for each connection */
void process_request(void *p)
{
	int sd = (int)p;
	int RECV_BUF_SIZE = 2048;
	char recv_buf[RECV_BUF_SIZE];
	int n;

	while (1) {
		/* read a max of RECV_BUF_SIZE bytes from socket */
		if ((n = read(sd, recv_buf, RECV_BUF_SIZE)) < 0) {
			xil_printf("%s: error reading from socket %d, closing socket\r\n", __FUNCTION__, sd);
			break;
		}
		/* break if client closed connection */
		if (n <= 0)
			break;

		float val;

		for(int i = 0; i < n - 1;){
			switch(recv_buf[i]){
			case 'y':
				val = *((float *) &recv_buf[i+1]);
				Hexapod.bodyRotTarget.y = val;
				i += 5;
				break;
			case 'p':
				val = *((float *) &recv_buf[i+1]);
				Hexapod.bodyRotTarget.p = val;
				i += 5;
				break;
			case 'r':
				val = *((float *) &recv_buf[i+1]);
				Hexapod.bodyRotTarget.r = val;
				i += 5;
				break;
			case 's':
				val = *((float *) &recv_buf[i+1]);
				Hexapod.stepTime = val;
				i += 5;
				break;
			}
		}
	}

	/* close connection */
	close(sd);
	vTaskDelete(NULL);
}

void remote_application_thread(void *p)
{
	int sock, new_sd;
	struct sockaddr_in address, remote;
	int size;

	if ((sock = lwip_socket(AF_INET, SOCK_STREAM, 0)) < 0)
		return;

	address.sin_family = AF_INET;
	address.sin_port = htons(server_port);
	address.sin_addr.s_addr = INADDR_ANY;

	if (lwip_bind(sock, (struct sockaddr *)&address, sizeof (address)) < 0)
		return;

	lwip_listen(sock, 0);

	size = sizeof(remote);

	while (1) {
		if ((new_sd = lwip_accept(sock, (struct sockaddr *)&remote, (socklen_t *)&size)) > 0) {
			sys_thread_new("ClientHandler", process_request,
				(void*)new_sd,
				THREAD_STACKSIZE,
				DEFAULT_THREAD_PRIO);
		}
	}
}

// logs Server
void process_logs_request(void *p)
{
	int sd = (int)p;
	char recv_buf[] = "Hello\r\n";
	int n = 7, nwrote;
	TickType_t st = pdMS_TO_TICKS( 2000 );

	while (1) {
		/* handle request */
		if ((nwrote = write(sd, recv_buf, n)) < 0) {
			xil_printf("%s: ERROR responding to client echo request. received = %d, written = %d\r\n",
					__FUNCTION__, n, nwrote);
			xil_printf("Closing socket %d\r\n", sd);
			break;
		}
		vTaskDelay( st );
	}

	/* close connection */
	close(sd);
	vTaskDelete(NULL);
}

void logs_application_thread(void *p)
{
	int sock, new_sd;
	struct sockaddr_in address, remote;
	int size;

	if ((sock = lwip_socket(AF_INET, SOCK_STREAM, 0)) < 0)
		return;

	address.sin_family = AF_INET;
	address.sin_port = htons(logs_server_port);
	address.sin_addr.s_addr = INADDR_ANY;

	if (lwip_bind(sock, (struct sockaddr *)&address, sizeof (address)) < 0)
		return;

	lwip_listen(sock, 0);

	size = sizeof(remote);

	while (1) {
		if ((new_sd = lwip_accept(sock, (struct sockaddr *)&remote, (socklen_t *)&size)) > 0) {
			sys_thread_new("LogsClientHandler", process_logs_request,
				(void*)new_sd,
				THREAD_STACKSIZE,
				DEFAULT_THREAD_PRIO);
		}
	}
}

void network_thread(void *p)
{
    struct netif *netif;
    struct ip_addr ipaddr, netmask, gw;

    /* the mac address of the board. this should be unique per board */
    unsigned char mac_ethernet_address[] = { 0x00, 0x0a, 0x35, 0x00, 0x01, 0x02 };

    netif = &server_netif;

    /* initliaze IP addresses to be used */
    IP4_ADDR(&ipaddr,  192, 168, 3, 100);
    IP4_ADDR(&netmask, 255, 255, 255,  0);
    IP4_ADDR(&gw,      192, 168, 3, 254);

    /* print out IP settings of the board */
    xil_printf("\r\n\r\n");
    xil_printf("-----lwIP Socket Mode Echo server Demo Application ------\r\n");

    /* Add network interface to the netif_list, and set it as default */
    if (!xemac_add(netif, &ipaddr, &netmask, &gw, mac_ethernet_address, XPAR_XEMACPS_0_BASEADDR)) {
        xil_printf("Error adding N/W interface\r\n");
        return;
    }
    netif_set_default(netif);

    /* specify that the network if is up */
    netif_set_up(netif);

    /* start packet receive thread - required for lwIP operation */
    sys_thread_new("xemacif_input_thread", (void(*)(void*))xemacif_input_thread, netif,
            THREAD_STACKSIZE,
            DEFAULT_THREAD_PRIO);

    sys_thread_new("RemoteApp", remote_application_thread, 0,
		THREAD_STACKSIZE,
		DEFAULT_THREAD_PRIO);

    sys_thread_new("LogsApp", logs_application_thread, 0,
    		THREAD_STACKSIZE,
    		DEFAULT_THREAD_PRIO);
    vTaskDelete(NULL);
    return;
}

#endif /* SRC_NETWORKTASKFUNCTION_H_ */
