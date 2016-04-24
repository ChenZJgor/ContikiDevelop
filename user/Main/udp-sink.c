/*
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
 *
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/uip.h"
#include "net/rpl/rpl.h"
#include "net/rime/rimeaddr.h"
#include "dev/leds.h"

#include "net/netstack.h"
#include "dev/button-sensor.h"
#include "dev/serial-line.h"
#include "dev/leds.h"
#include "dev/uart0.h"
#include "dev/uart1.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "collect-common.h"
#include "collect-view.h"
#include "dev/12864.h"
#include "dev/dht11.h"

#define DEBUG DEBUG_PRINT 
#include "net/uip-debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688
#define MAX_PAYLOAD_LEN		40
static uip_ipaddr_t clientipaddr;

static struct uip_udp_conn *server_conn;
#define MT_PACKET_LEN  30  // max data length is 30
static uint8_t mt_packet[MT_PACKET_LEN];
static process_event_t blink_event;
static uint8_t sta = 0;
unsigned char func_index = 0;
unsigned char dht11data[4] = {0};

extern void (*current_operation_index)();

PROCESS(udp_server_process, "UDP server process");
AUTOSTART_PROCESSES(&udp_server_process);
/*---------------------------------------------------------------------------*/
void
collect_common_set_sink(void)
{
}
/*---------------------------------------------------------------------------*/
void
collect_common_net_print(void)
{
  printf("I am sink!\n");
}
/*---------------------------------------------------------------------------*/
void
collect_common_send(void)
{
  /* Server never sends */
}
/*---------------------------------------------------------------------------*/
/*--- payload is a pointer to struct collect_view_data_msg type (有效载荷是一个指向结构的指针collect_view_data_msg类型)----*/
extern rimeaddr_t rimeaddr_node_addr;
void
collect_common_recv(const rimeaddr_t *originator, uint8_t seqno, uint8_t hops,
                    uint8_t *payload, uint16_t payload_len)//originator 发起人; seqno 序号,序列号,成交回报的广播序号; hops 跳数; payload 有效载荷;
{
  int i;
  uint8_t fcs = 0;
  uint8_t data_len = 0;
  
  // output data to UART
  //0xA5+LEN(0,1)+SenderADDR(1,2)+TEMP(3,1)+VOLTAGE(4,1)+parentAddr(5,2) + FCS(7,1)
  mt_packet[0]=0xA5;
  mt_packet[1]=6; // 去掉 0xA5,LEN,FCS这几个字段长度
  data_len = mt_packet[1] + 3;
  mt_packet[2]=originator->u8[0]; //数据发送节点的地址
  mt_packet[3]=originator->u8[1];
  mt_packet[4]= *(payload+28); //msg->sensors[TEMP_SENSOR]
  mt_packet[5]= *(payload+24); //msg->sensors[BATTERY_VOLTAGE_SENSOR]
  mt_packet[6]= *(payload+14); // parent addr, Low byte
  mt_packet[7]= *(payload+15); // parent addr, high byte
  
  for(i=1;i<(data_len-1); i++) {
  fcs = fcs ^ mt_packet[i];
  }
  mt_packet[8]= fcs; 
    
  // put HEX data to uart0
  uart0_write(&mt_packet[0],data_len);  
}
/*---------------------------------------------------------------------------*/
void
collect_common_net_init(void)
{
  collect_common_set_send_active(1);
  uart1_set_input(serial_line_input_byte);
  serial_line_init();

  PRINTF("I am sink!\r\n");
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  char *appdata;
  unsigned char i;
  blink_event = process_alloc_event();

  if(uip_newdata()) {
    appdata = (char *)uip_appdata;
    appdata[uip_datalen()] = 0;
    for(i = 0; i < 4; i++){
      dht11data[i] = *(appdata + i);
    }
    PRINTF("DATA recv '%s' from ", appdata);
    PRINTF("%d",
           UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1]);
    PRINTF("\n");
    PRINTF("wendu is %c%c, shidu is %c%c \n", *appdata,*(appdata+1),*(appdata+2),*(appdata+3));
    
//    PRINTF("DATA sending reply\n");
//    uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
    /*
    if(strncmp(appdata, "RED", 3) == 0) {
      leds_on(LEDS_RED);
      leds_off(LEDS_GREEN);
      sta = 0;
      }
    if(strncmp(appdata, "GREEN", 5) == 0) {
      leds_on(LEDS_GREEN);
      leds_off(LEDS_RED);
      sta = 0;
      }
    if(strncmp(appdata,"BLINK",5) == 0) {
      process_post(&udp_server_process, blink_event, NULL);
      sta = 1;
    }
    if(strncmp(appdata,"OFF",3) == 0) {
      leds_off(LEDS_RED | LEDS_GREEN);
      sta = 0;
    }
#if SERVER_REPLY
    PRINTF("DATA sending reply\n");
    uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
    uip_udp_packet_send(server_conn, "Reply", sizeof("Reply"));
    uip_create_unspecified(&server_conn->ripaddr);
#endif
    */
  }
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Server IPv6 addresses: \r\n");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(state == ADDR_TENTATIVE || state == ADDR_PREFERRED) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\r\n");
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
        uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
  struct sensors_sensor *sensor;
  uip_ipaddr_t ipaddr;
  struct uip_ds6_addr *root_if;
  static struct etimer timer;
//  char buf[MAX_PAYLOAD_LEN];

  PROCESS_BEGIN();

  collect_common_net_init();
  
  PROCESS_PAUSE();

  SENSORS_ACTIVATE(button_sensor);

  PRINTF("UDP server started\r\n");

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);
  uip_ip6addr(&clientipaddr,0xaaaa,0,0,0,0x0212,0x4B00,0x051d,0x0d30);
  
  /* uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr); */
  uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL); //MANUAL 手动模式;说明书；
  root_if = uip_ds6_addr_lookup(&ipaddr);//look up 查找;
  if(root_if != NULL) {
    rpl_dag_t *dag;
    dag = rpl_set_root(RPL_DEFAULT_INSTANCE,(uip_ip6addr_t *)&ipaddr);
    uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
    rpl_set_prefix(dag, &ipaddr, 64); //prefix 前缀;
    PRINTF("created a new RPL dag.\r\n");
  } else {
    PRINTF("failed to create a new RPL DAG.\r\n");
  }

  print_local_addresses();

  /* The data sink runs with a 100% duty cycle in order to ensure high
     packet reception rates. */
  NETSTACK_RDC.off(1);

  server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
  udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));

  PRINTF("Created a server connection with remote address ");
  PRINT6ADDR(&server_conn->ripaddr);
  PRINTF("\r\n");
  PRINTF(" local/remote port %u/%u\r\n", UIP_HTONS(server_conn->lport),
         UIP_HTONS(server_conn->rport));
  initlcm();
  set_wenzi();

  while(1) {
    PROCESS_YIELD();
        if(ev == sensors_event){
	sensor = (struct sensors_sensor *)data;
	if(sensor == &button_sensor){
            func_index=table[func_index].up;
	}
        
        if(sensor == &button_2_sensor){
            func_index=table[func_index].down;
	}
        if(sensor == &button_3_sensor){
            func_index=table[func_index].enter;
	}
        
        current_operation_index=table[func_index].current_operation;
        (*current_operation_index)();//执行当前操作函数
        }
    if(ev == tcpip_event) {
      tcpip_handler();
    } else if (ev == blink_event) {
      etimer_set(&timer, CLOCK_CONF_SECOND);
    }
    if(ev == PROCESS_EVENT_TIMER && sta == 1) {
      if(data == &timer){
        leds_toggle(LEDS_GREEN | LEDS_RED);
      }
       etimer_reset(&timer);
    }
    if(ev == serial_line_event_message) {
      char *line;
      line = (char *)data;
      if(strncmp(line, "GET", 3) == 0) {
        PRINTF("GET");
    uip_ipaddr_copy(&server_conn->ripaddr, &clientipaddr);
    uip_udp_packet_send(server_conn, "GET", sizeof("GET"));
    uip_create_unspecified(&server_conn->ripaddr);
      }else {
        printf("unhandled command: %s\n", line);
      }
          /* Store rv temporarily in dec so we can use it for the battery */
    }     
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
