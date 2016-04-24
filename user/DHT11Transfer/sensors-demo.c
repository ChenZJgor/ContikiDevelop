/*
 * Copyright (c) 2010, Loughborough University - Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

/**
 * \file
 *         Example to demonstrate-test cc2530 sensor functionality
 *
 *         B1 turns LED_GREEN on and off.
 *
 *         The node takes readings from the various sensors every x seconds and
 *         prints out the results.
 *
 *         We use floats here to translate the AD conversion results to
 *         meaningful values. However, our printf does not have %f support so
 *         we use an ugly hack to print out the value by extracting the integral
 *         part and then the fractional part. Don't try this at home.
 *
 *         Temperature:
 *           Math is correct, the sensor needs calibration per device.
 *           I currently use default values for the math which may result in
 *           very incorrect values in degrees C.
 *           See TI Design Note DN102 about the offset calibration.
 *
 *         Supply Voltage (VDD):
 *           For VDD, math is correct, conversion is correct.
 *           See DN101 for details.
 *
 *         Make sure you enable/disable things in project-conf.h
 *
 * \author
 *         George Oikonomou - <oikonomou@users.sourceforge.net>
 *         YoCiHou - < www.iotdev.net>
 */


#include "contiki.h"
#include "contiki-conf.h"
#include "dev/leds.h"

#include "dev/button-sensor.h"
#include "dev/adc-sensor.h"

#include "net/uip.h"
#include "net/uip-ds6.h"
#include "net/uip-udp-packet.h"
//#include "net/neighbor-info.h"
#include "net/rpl/rpl.h"
#include "dev/serial-line.h"
#include "dev/uart0.h"
#include "dev/uart1.h"
#include "collect-common.h"
#include "collect-view.h"
#include "dev/port.h"
#include "dev/dht11.h"

#include <stdio.h>
#include <string.h>

#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define MAX_PAYLOAD_LEN		40

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;
static unsigned char flag = 0;
extern unsigned char senddata[4];
//static int dec;
//static float frac;


#define DEBUG 1

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#include "net/uip-debug.h"
#else /* DEBUG */
/* We overwrite (read as annihilate) all output functions here (我们于此重写(读消灭)所有输出功能)*/

#define PRINTF(...)
#endif /* DEBUG */
/*---------------------------------------------------------------------------*/

#if BUTTON_SENSOR_ON
PROCESS(buttons_test_process, "Button Test Process");
AUTOSTART_PROCESSES(&buttons_test_process);
#else
AUTOSTART_PROCESSES(&sensors_test_process);
#endif

/*---------------------------------------------------------------------------*/
static void
send_packet(void *ptr)
{
  char buf[MAX_PAYLOAD_LEN];

  PRINTF("DATA send to %d \n",
         server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1]);
  sprintf(buf, "hello");
  uip_udp_packet_sendto(client_conn, buf, strlen(buf),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

void
collect_common_set_sink(void)
{
  /* A udp client can never become sink */
}
/*---------------------------------------------------------------------------*/
extern uip_ds6_route_t uip_ds6_routing_table[UIP_DS6_ROUTE_NB];

void
collect_common_net_print(void)
{
  rpl_dag_t *dag;
  uip_ds6_route_t *r;

  /* Let's suppose we have only one instance */
  dag = rpl_get_any_dag();
  if(dag->preferred_parent != NULL) {
    PRINTF("Preferred parent: ");
    //PRINT6ADDR(&dag->preferred_parent->addr);
    PRINT6ADDR(rpl_get_parent_ipaddr(dag->preferred_parent));
    PRINTF("\n");
  }
  PRINTF("Route entries:\n");
  for(r = uip_ds6_route_head();
      
      r != NULL;
      
      r = uip_ds6_route_next(r)) {
    
        PRINT6ADDR(&r->ipaddr);
  
      }
  
  PRINTF("---\n");
//  for(r = uip_ds6_route_list_head(); r != NULL; r = list_item_next(r)) {
//    PRINT6ADDR(&r->ipaddr);
//  }
//  PRINTF("---\n");
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  char *appdata;

  if(uip_newdata()) {
    appdata = (char *)uip_appdata;
    appdata[uip_datalen()] = 0;
    PRINTF("DATA recv '%s' from ", appdata);
    PRINTF("%d",
           UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1]);
    PRINTF("\n");
    if(strncmp(appdata, "GET", 3) == 0) {
      flag = 1;
      }
  }
}
/*---------------------------------------------------------------------------*/
void
collect_common_send(void)
{
  static uint8_t seqno;
  struct {
    uint8_t seqno;
    uint8_t for_alignment;
    struct collect_view_data_msg msg;
  } msg;
  /* struct collect_neighbor *n; */
  uint16_t parent_etx;
  uint16_t rtmetric;
  uint16_t num_neighbors;
  uint16_t beacon_interval;
  rpl_parent_t *preferred_parent;
  rimeaddr_t parent;
  rpl_dag_t *dag;

  if(client_conn == NULL) {
    /* Not setup yet */
    return;
  }
  memset(&msg, 0, sizeof(msg));
  seqno++;
  if(seqno == 0) {
    /* Wrap to 128 to identify restarts */
    seqno = 128;
  }
  msg.seqno = seqno;

  rimeaddr_copy(&parent, &rimeaddr_null);
  parent_etx = 0;

  /* Let's suppose we have only one instance */
  dag = rpl_get_any_dag();
  if(dag != NULL) {
    preferred_parent = dag->preferred_parent;
    if(preferred_parent != NULL) {
      uip_ds6_nbr_t *nbr;
      //nbr = uip_ds6_nbr_lookup(&preferred_parent->addr);
      nbr = uip_ds6_nbr_lookup(rpl_get_parent_ipaddr(preferred_parent));
      if(nbr != NULL) {
        /* Use parts of the IPv6 address as the parent address, in reversed byte order. */
        parent.u8[RIMEADDR_SIZE - 1] = nbr->ipaddr.u8[sizeof(uip_ipaddr_t) - 2];
        parent.u8[RIMEADDR_SIZE - 2] = nbr->ipaddr.u8[sizeof(uip_ipaddr_t) - 1];
        //parent_etx = neighbor_info_get_metric((rimeaddr_t *) &nbr->lladdr) / 2;
        //parent_etx = rpl_get_parent_rank((rimeaddr_t *) uip_ds6_nbr_get_ll(nbr)) / 2;
      }
    }
    rtmetric = dag->rank;
    //beacon_interval = (uint16_t) ((2L << dag->instance->dio_intcurrent) / 1000);
    //beacon_interval = (uint16_t) ((2L << dag->instance->dio_intcurrent) / 1000);
    num_neighbors = RPL_PARENT_COUNT(dag);
  } else {
    rtmetric = 0;
    beacon_interval = 0;
    num_neighbors = 0;
  }

  /* num_neighbors = collect_neighbor_list_num(&tc.neighbor_list); */
  collect_view_construct_message(&msg.msg, &parent,
                                 parent_etx, rtmetric,
                                 num_neighbors, beacon_interval);

  uip_udp_packet_sendto(client_conn, &msg, sizeof(msg),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
}
/*---------------------------------------------------------------------------*/
void
collect_common_recv(const rimeaddr_t *originator, uint8_t seqno, uint8_t hops,
                    uint8_t *payload, uint16_t payload_len)
{
  /*-- sender don't receive---*/
}
/*---------------------------------------------------------------------------*/
void
collect_common_net_init(void)
{
  collect_common_set_send_active(1);
  uart0_set_input(serial_line_input_byte);
  serial_line_init();
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
        uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  /* set server address */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);

}
/*---------------------------------------------------------------------------*/


#if BUTTON_SENSOR_ON
PROCESS_THREAD(buttons_test_process, ev, data)
{
  struct sensors_sensor *sensor;
  static unsigned char state = 0;
  char buf[MAX_PAYLOAD_LEN];
  unsigned char i;
  static struct etimer et;
//  static unsigned char STATE = 0;

  PROCESS_BEGIN();
  
  collect_common_net_init();
  
  PROCESS_PAUSE();

  set_global_address();

  PRINTF("UDP client process started\n");

  print_local_addresses();

  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));

  PRINTF("Created a connection with the server ");
  PRINT6ADDR(&client_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
        UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));
  etimer_set(&et,CLOCK_SECOND * 10);
          
  while (1) {
    PROCESS_WAIT_EVENT();
    if(etimer_expired(&et)){
/*      STATE++;
      sprintf(buf, "hello");
        uip_udp_packet_sendto(client_conn, &buf, sizeof(buf),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));*/
        state = GET_DHT11DATA();
        if(state){
          for(i = 0; i < 4; i++){
            buf[i] = senddata[i];
          }
          buf[4] = 0;
          uip_udp_packet_sendto(client_conn, buf, strlen(buf),
                      &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
          state = 0;
        }
      
      etimer_reset(&et);
    }
   
    if(ev == tcpip_event) {
/*      tcpip_handler();
     if(flag){
        state = GET_DHT11DATA();
        if(state){
          for(i = 0; i < 4; i++){
            buf[i] = senddata[i] + 0x30;
          }
          buf[4] = 0;
          uip_udp_packet_sendto(client_conn, buf, strlen(buf),
                      &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
          state = 0;
        }
      flag = 0;
      }*/
    }
    if(ev == sensors_event){
		sensor = (struct sensors_sensor *)data;
		if(sensor == &button_sensor){

		}
	}
    if(ev == serial_line_event_message) {
      char *line;
      line = (char *)data;
      if(strncmp(line, "RED", 3) == 0) {
        sprintf(buf, "RED");
        PRINTF("RED");
        uip_udp_packet_sendto(client_conn, buf, strlen(buf),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
      }else if(strncmp(line, "GREEN", 5) == 0){
        sprintf(buf, "GREEN");
        uip_udp_packet_sendto(client_conn, buf, strlen(buf),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
      }else if(strncmp(line, "BLINK", 5) == 0){
        sprintf(buf, "BLINK");
        uip_udp_packet_sendto(client_conn, buf, strlen(buf),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
      }else if(strncmp(line, "OFF", 3) == 0){
        sprintf(buf, "OFF");
        uip_udp_packet_sendto(client_conn, buf, strlen(buf),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
      }else {
        printf("unhandled command: %s\n", line);
      }
          /* Store rv temporarily in dec so we can use it for the battery */
      }
    }
    PROCESS_END();

}
/*---------------------------------------------------------------------------*/

#endif

