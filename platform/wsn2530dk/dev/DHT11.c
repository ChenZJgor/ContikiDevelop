#include "contiki.h"
#include "dev/port.h"
#include "DHT11.h"

static struct timer delaytimer;

static unsigned char ucharFLAG, uchartemp;
static unsigned char ucharcomdata = 0;
static unsigned char ucharT_data_H,ucharT_data_L,ucharRH_data_H,ucharRH_data_L,ucharcheckdata;
static unsigned char ucharT_data_H_temp,ucharT_data_L_temp,ucharRH_data_H_temp,ucharRH_data_L_temp,ucharcheckdata_temp;
unsigned char senddata[4];

void COM(void)    // 温湿写入
{     
    unsigned char i;         
    for(i=0;i<8;i++)    
    {
        unsigned charFLAG=2; 
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
unsigned char GET_DHT11DATA(void)
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
        senddata[0]=ucharT_data_H/10 + 0x30; 
        senddata[1]=ucharT_data_H%10 + 0x30;
        
        senddata[2]=ucharRH_data_H/10 + 0x30; 
        senddata[3]=ucharRH_data_H%10 + 0x30;
        printf("shidu is %d%d,wendu is %d%d\n",senddata[2],senddata[3],senddata[0],senddata[1]);
        return 1;
    }
    else //没用成功读取，返�?
    {
        senddata[0]=0; 
        senddata[1]=0;
        
        senddata[2]=0; 
        senddata[3]=0;  
        return 0;
    } 
    
}
/*---------------------------------------------------------------------------*/