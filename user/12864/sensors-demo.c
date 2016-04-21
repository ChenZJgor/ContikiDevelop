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
#include "dev/12864.h"

#include <stdio.h>
#include <string.h>

#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define MAX_PAYLOAD_LEN		40

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;
static unsigned char flag = 0;
static struct timer delaytimer;
static struct timer delaytimer12864;
static unsigned char ucharFLAG,uchartemp;
static unsigned char ucharcomdata = 0;
static unsigned char ucharT_data_H,ucharT_data_L,ucharRH_data_H,ucharRH_data_L,ucharcheckdata;
static unsigned char ucharT_data_H_temp,ucharT_data_L_temp,ucharRH_data_H_temp,ucharRH_data_L_temp,ucharcheckdata_temp;
static unsigned char senddata[4];
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
void sendbyte(unsigned char bbyte) //发送一个字节
{
	PORT_FUNC_GPIO(SID_PORT, SID_PIN);
    PORT_DIR_OUTPUT(SID_PORT, SID_PIN);
	PORT_FUNC_GPIO(CLK_PORT, CLK_PIN);
    PORT_DIR_OUTPUT(CLK_PORT, CLK_PIN);
	
	unsigned char i;
	for(i=0;i<8;i++)
		{
			if(bbyte&0x80)//取出最高位
				PORT_SET(SID_PORT, SID_PIN); 
			else
				PORT_CLEAR(SID_PORT, SID_PIN);
	   PORT_SET(CLK_PORT, CLK_PIN); 
	   PORT_CLEAR(CLK_PORT, CLK_PIN);
	   bbyte <<= 1; //左移
	   }  
}

void write(unsigned char start, unsigned char ddata) //写指令或数据
{
	unsigned char start_data,Hdata,Ldata;
	if(start == COMMAND)
		start_data = 0xf8;  //写指令
	else      
		start_data = 0xfa;  //写数据

	Hdata = ddata&0xf0;    //取高四位
	Ldata = (ddata<<4)&0xf0;  //取低四位
	sendbyte(start_data);   //发送起始信号
	clock_delay_usec(100); //延时是必须的
	sendbyte(Hdata);       //发送高四位
	clock_delay_usec(100);  //延时是必须的
	sendbyte(Ldata);    //发送低四位
	clock_delay_usec(100);  //延时是必须的
}

void initlcm(void)
{
//	timer_set(&delaytimer12864, DELAY_100MS);
//	while(!(timer_expired(&delaytimer12864)));
	write(COMMAND, 0x30);//功能设置，一次送8位数据，基本指令集
	write(COMMAND, 0x0C);//0000,1100 整体显示，游标off，游标位置off
	write(COMMAND, 0x01);//0000,0001 清DDRAM
	write(COMMAND, 0x02);//0000,0010 DDRAM地址归位
	write(COMMAND, 0x80);//1000,0000 设定DDRAM 7位地址000，0000到地址计数器AC
}

void display(unsigned char x_add,unsigned char dat1,unsigned char dat2)
{
	write(COMMAND, x_add);//1xxx,xxxx 设定DDRAM 7位地址xxx,xxxx到地址计数器AC
	write(DATA, dat1);
	write(DATA, dat2);
}

void set_wenzi(void)
{
	write(COMMAND, 0x80);//1000,0001 设定DDRAM 7位地址000，0001到地址计数器AC.
	write(DATA, 0x48);//将ASCII码调出来,显示在液晶屏幕上.下同
	write(DATA, 0x65);
	write(DATA, 0x6c);
	write(DATA, 0x6c);
	write(DATA, 0x6f);
	write(DATA, 0x00);
	write(DATA, 0x4d);
	write(DATA, 0x72);
	write(DATA, 0x2e);
	write(DATA, 0x5a);
	write(DATA, 0x68);
	write(DATA, 0x6f);
	write(DATA, 0x75);

	write(COMMAND, 0x90); //设置液晶屏的显示地址,下同.
	write(DATA, 0x4d);
	write(DATA, 0x79);
	write(DATA, 0x20);
	write(DATA, 0x6e);
	write(DATA, 0x61);
	write(DATA, 0x6d);
	write(DATA, 0x65);
	write(DATA, 0x20);
	write(DATA, 0x69);
	write(DATA, 0x73);
	write(DATA, 0x20);
	write(DATA, 0x59);
	write(DATA, 0x75);
	write(COMMAND, 0x88);
	write(DATA, 0x32);
	write(DATA, 0x30);
	write(DATA, 0x3f);
	write(DATA, 0x3f);
	display(0x8a,0xc4,0xea); //将中文字调出来."年"的码值是:c4ea.
	write(DATA, 0x3f);
	write(DATA, 0x3f);
	display(0x8c,0xd4,0xc2);
	write(DATA, 0x3f);
	write(DATA, 0x3f);
	display(0x8e,0xc8,0xd5);
	write(COMMAND, 0x98);
	display(0x98,0xd0,0xc7);
	display(0x99,0xc6,0xda);
	write(DATA, 0x3f);
	write(DATA, 0x3f);
	write(DATA, 0x3f);
	write(DATA, 0x3f);
	write(DATA, 0x3a);
	write(DATA, 0x3f);
	write(DATA, 0x3f);
}
/*---------------------------------------------------------------------------*/
void COM(void)    // 温湿写入
{     
    unsigned char i;         
    for(i=0;i<8;i++)    
    {
        ucharFLAG=2; 
        while((!(PORT_READ(DHT11_DATA_PORT, DHT11_DATA_PIN)))&&ucharFLAG++);
        clock_delay_usec(30);
        uchartemp=0;
        if(PORT_READ(DHT11_DATA_PORT, DHT11_DATA_PIN))
          uchartemp=1;
        ucharFLAG=2;
        while((PORT_READ(DHT11_DATA_PORT, DHT11_DATA_PIN))&&ucharFLAG++);   
        if(ucharFLAG==1)
          break;    
        ucharcomdata<<=1;
        ucharcomdata|=uchartemp; 
    }    
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static unsigned char
GET_DHT11DATA(void)
{
    PORT_FUNC_GPIO(DHT11_DATA_PORT, DHT11_DATA_PIN);
    PORT_DIR_OUTPUT(DHT11_DATA_PORT, DHT11_DATA_PIN);
    PORT_CLEAR(DHT11_DATA_PORT, DHT11_DATA_PIN);
    //Delay_ms(19);
    timer_set(&delaytimer, INIT_TIME);
    while(!(timer_expired(&delaytimer)));
    PORT_SET(DHT11_DATA_PORT, DHT11_DATA_PIN);
    PORT_DIR_INPUT(DHT11_DATA_PORT, DHT11_DATA_PIN);
    clock_delay_usec(40);
    if(!(PORT_READ(DHT11_DATA_PORT, DHT11_DATA_PIN))){
      ucharFLAG = 2;
      while((!(PORT_READ(DHT11_DATA_PORT, DHT11_DATA_PIN)))&&ucharFLAG++);
      ucharFLAG=2;
      while((PORT_READ(DHT11_DATA_PORT, DHT11_DATA_PIN))&&ucharFLAG++);
      COM();
      ucharRH_data_H_temp=ucharcomdata;
      COM();
      ucharRH_data_L_temp=ucharcomdata;
      COM();
      ucharT_data_H_temp=ucharcomdata;
      COM();
      ucharT_data_L_temp=ucharcomdata;
      COM();
      ucharcheckdata_temp=ucharcomdata;
      uchartemp=(ucharT_data_H_temp+ucharT_data_L_temp+ucharRH_data_H_temp+ucharRH_data_L_temp);
      if(uchartemp==ucharcheckdata_temp)
      {
        ucharRH_data_H=ucharRH_data_H_temp;
        ucharRH_data_L=ucharRH_data_L_temp;
        ucharT_data_H=ucharT_data_H_temp;
        ucharT_data_L=ucharT_data_L_temp;
        ucharcheckdata=ucharcheckdata_temp;
      }
        senddata[0]=ucharT_data_H/10; 
        senddata[1]=ucharT_data_H%10;
        
        senddata[2]=ucharRH_data_H/10; 
        senddata[3]=ucharRH_data_H%10;
        printf("shidu is %d%d,wendu is %d%d\n",senddata[2],senddata[3],senddata[0],senddata[1]);
        return 1;
    }
    else //没用成功读取，返回0
    {
        senddata[0]=0; 
        senddata[1]=0;
        
        senddata[2]=0; 
        senddata[3]=0;  
        return 0;
    } 
    
}
/*---------------------------------------------------------------------------*/
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
  uart1_set_input(serial_line_input_byte);
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
  etimer_set(&et,CLOCK_SECOND);
  initlcm();
  set_wenzi();
          
  while (1) {
    PROCESS_WAIT_EVENT();
    if(etimer_expired(&et)){
/*      STATE++;
      sprintf(buf, "hello");
        uip_udp_packet_sendto(client_conn, &buf, sizeof(buf),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));*/
/*              state = GET_DHT11DATA();
        if(state){
          for(i = 0; i < 4; i++){
            buf[i] = senddata[i] + 0x30;
          }
          buf[4] = 0;
          uip_udp_packet_sendto(client_conn, buf, strlen(buf),
                      &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
          state = 0;
        }*/
      
      etimer_reset(&et);
    }
   
    if(ev == tcpip_event) {
      tcpip_handler();
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
      }
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

