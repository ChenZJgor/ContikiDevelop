#include "contiki.h"
#include "dev/12864.h"
#include "dev/port.h"
#include "dev/dht11.h"

static struct timer delaytimer12864;

void (*current_operation_index)();

/*---------------------------------------------------------------------------*/
void fun1(void) //第一层，显示功能菜单、【传感器】、时间、设置
{
        clrram(); //清屏，下同
        display(0x82,0xb9,0xa6); 
        display(0x83,0xc4,0xdc); 
        display(0x84,0xb2,0xcb); 
        display(0x85,0xb5,0xa5); 
        
        write(COMMAND, 0x90); //设置液晶屏的显示地址,下同
        write(DATA, 0x31); 
        write(DATA, 0x2e); 
        display(0x91,0xa1,0xbe); 
        display(0x92,0xb4,0xab); 
        display(0x93,0xb8,0xd0); 
        display(0x94,0xc6,0xf7); 
        display(0x95,0xa1,0xbf); 
        
	write(COMMAND, 0x88); 
        write(DATA, 0x32); 
        write(DATA, 0x2e); 
        display(0x8a,0xca,0xb1);
        display(0x8b,0xbc,0xe4);
        
        write(COMMAND, 0x98);
        write(DATA, 0x33);
        write(DATA, 0x2e);
        display(0x9a,0xc9,0xe8);
        display(0x9b,0xd6,0xc3);
      
}

void fun2(void) //第一层，显示功能菜单、传感器、【时间】、设置
{
        clrram();
        display(0x82,0xb9,0xa6); 
        display(0x83,0xc4,0xdc); 
        display(0x84,0xb2,0xcb);
        display(0x85,0xb5,0xa5);
        
        write(COMMAND, 0x90);
        write(DATA, 0x31);
        write(DATA, 0x2e);
        display(0x92,0xb4,0xab);
        display(0x93,0xb8,0xd0);
        display(0x94,0xc6,0xf7);
        
	write(COMMAND, 0x88); //设置液晶屏的显示地址,下同.
        write(DATA, 0x32);
        write(DATA, 0x2e);
        display(0x89,0xa1,0xbe);
        display(0x8a,0xca,0xb1);
        display(0x8b,0xbc,0xe4);
        display(0x8c,0xa1,0xbf);
        
        write(COMMAND, 0x98);
        write(DATA, 0x33);
        write(DATA, 0x2e);
        display(0x9a,0xc9,0xe8);
        display(0x9b,0xd6,0xc3);
}

void fun3(void) //第一层，显示功能菜单、传感器、时间、【设置】
{
        clrram();
        display(0x82,0xb9,0xa6); 
        display(0x83,0xc4,0xdc); 
        display(0x84,0xb2,0xcb);
        display(0x85,0xb5,0xa5);
        
        write(COMMAND, 0x90);
        write(DATA, 0x31);
        write(DATA, 0x2e);
        display(0x92,0xb4,0xab);
        display(0x93,0xb8,0xd0);
        display(0x94,0xc6,0xf7);
        
	write(COMMAND, 0x88); //设置液晶屏的显示地址,下同.
        write(DATA, 0x32);
        write(DATA, 0x2e);
        display(0x8a,0xca,0xb1);
        display(0x8b,0xbc,0xe4);
        
        write(COMMAND, 0x98);
        write(DATA, 0x33);
        write(DATA, 0x2e);
        display(0x99,0xa1,0xbe);
        display(0x9a,0xc9,0xe8);
        display(0x9b,0xd6,0xc3);
        display(0x9c,0xa1,0xbf);
}

void fun4(void) //第二层，显示【温湿度传感器】、烟雾传感器、门磁传感器、返回
{
        clrram();
        display(0x80,0xa1,0xbe);
        display(0x81,0xce,0xc2); 
        display(0x82,0xca,0xaa); 
        display(0x83,0xb6,0xc8);
        display(0x84,0xb4,0xab);
        display(0x85,0xb8,0xd0);
        display(0x86,0xc6,0xf7);
        display(0x87,0xa1,0xbf);
        
        display(0x91,0xd1,0xcc);
        display(0x92,0xce,0xed);
        display(0x93,0xb4,0xab);
        display(0x94,0xb8,0xd0);
        display(0x95,0xc6,0xf7);
        
        display(0x89,0xc3,0xc5);
        display(0x8a,0xb4,0xc5);
        display(0x8b,0xb4,0xab);
        display(0x8c,0xb8,0xd0);
        display(0x8d,0xc6,0xf7);
        
        display(0x99,0xb7,0xb5);
        display(0x9a,0xbb,0xd8);
}

void fun5(void) //第二层，显示温湿度传感器、【烟雾传感器】、门磁传感器、返回
{
        clrram();

        display(0x81,0xce,0xc2); 
        display(0x82,0xca,0xaa); 
        display(0x83,0xb6,0xc8);
        display(0x84,0xb4,0xab);
        display(0x85,0xb8,0xd0);
        display(0x86,0xc6,0xf7);
 
        display(0x90,0xa1,0xbe);
        display(0x91,0xd1,0xcc);
        display(0x92,0xce,0xed);
        display(0x93,0xb4,0xab);
        display(0x94,0xb8,0xd0);
        display(0x95,0xc6,0xf7);
        display(0x96,0xa1,0xbf);
        
        display(0x89,0xc3,0xc5);
        display(0x8a,0xb4,0xc5);
        display(0x8b,0xb4,0xab);
        display(0x8c,0xb8,0xd0);
        display(0x8d,0xc6,0xf7);
        
        display(0x99,0xb7,0xb5);
        display(0x9a,0xbb,0xd8);
}

void fun6(void) //第二层，显示温湿度传感器、烟雾传感器、【门磁传感器】、返回
{
        clrram();

        display(0x81,0xce,0xc2); 
        display(0x82,0xca,0xaa); 
        display(0x83,0xb6,0xc8);
        display(0x84,0xb4,0xab);
        display(0x85,0xb8,0xd0);
        display(0x86,0xc6,0xf7); 

        display(0x91,0xd1,0xcc);
        display(0x92,0xce,0xed);
        display(0x93,0xb4,0xab);
        display(0x94,0xb8,0xd0);
        display(0x95,0xc6,0xf7);

        display(0x88,0xa1,0xbe);        
        display(0x89,0xc3,0xc5);
        display(0x8a,0xb4,0xc5);
        display(0x8b,0xb4,0xab);
        display(0x8c,0xb8,0xd0);
        display(0x8d,0xc6,0xf7);
        display(0x8e,0xa1,0xbf);
        
        display(0x99,0xb7,0xb5);
        display(0x9a,0xbb,0xd8);
}

void fun7(void) //第二层，显示温湿度传感器、烟雾传感器、门磁传感器、【返回】 
{
        clrram();

        display(0x81,0xce,0xc2); //温
        display(0x82,0xca,0xaa); //湿
        display(0x83,0xb6,0xc8); //度
        display(0x84,0xb4,0xab); //传
        display(0x85,0xb8,0xd0); //感
        display(0x86,0xc6,0xf7); //器

        display(0x91,0xd1,0xcc);
        display(0x92,0xce,0xed);
        display(0x93,0xb4,0xab);
        display(0x94,0xb8,0xd0);
        display(0x95,0xc6,0xf7);
       
        display(0x89,0xc3,0xc5);
        display(0x8a,0xb4,0xc5);
        display(0x8b,0xb4,0xab);
        display(0x8c,0xb8,0xd0);
        display(0x8d,0xc6,0xf7);


        display(0x98,0xa1,0xbe);         
        display(0x99,0xb7,0xb5);
        display(0x9a,0xbb,0xd8);        
        display(0x9b,0xa1,0xbf);        
}

void fun8(void) //第三层，显示现在的温度是、现在的湿度是、【重新获取】、返回
{
        clrram();

        display(0x80,0xcf,0xd6); 
        display(0x81,0xd4,0xda); 
        display(0x82,0xb5,0xc4);
        display(0x83,0xce,0xc2);
        display(0x84,0xb6,0xc8);
        display(0x85,0xca,0xc7);
        write(COMMAND, 0x86);
        write(DATA, 0x3a);
        write(DATA, 0x20);
        write(COMMAND, 0x87);
        write(DATA, senddata[0]);
        write(DATA, senddata[1]);

        display(0x90,0xcf,0xd6); 
        display(0x91,0xd4,0xda); 
        display(0x92,0xb5,0xc4);
        display(0x93,0xca,0xaa);
        display(0x94,0xb6,0xc8);
        display(0x95,0xca,0xc7);
        write(COMMAND, 0x96);
        write(DATA, 0x3a);
        write(DATA, 0x20);
        write(COMMAND, 0x97);
        write(DATA, senddata[2]);
        write(DATA, senddata[3]);
       
        display(0x88,0xa1,0xbe); 
        display(0x89,0xd6,0xd8);//重
        display(0x8a,0xd0,0xc2);//新
        display(0x8b,0xbb,0xf1);//获
        display(0x8c,0xc8,0xa1);//取
        display(0x8d,0xa1,0xbf);

        display(0x99,0xb7,0xb5);
        display(0x9a,0xbb,0xd8);   
}

void fun9(void) //第三层，显示现在的温度是、现在的湿度是、重新获取、【返回】
{
        clrram();

        display(0x80,0xcf,0xd6); 
        display(0x81,0xd4,0xda); 
        display(0x82,0xb5,0xc4);
        display(0x83,0xce,0xc2);
        display(0x84,0xb6,0xc8);
        display(0x85,0xca,0xc7);
        write(COMMAND, 0x86);
        write(DATA, 0x3a);
        write(DATA, 0x20);
        write(COMMAND, 0x87);
        write(DATA, senddata[0]);
        write(DATA, senddata[1]);        

        display(0x90,0xcf,0xd6); 
        display(0x91,0xd4,0xda); 
        display(0x92,0xb5,0xc4);
        display(0x93,0xca,0xaa);
        display(0x94,0xb6,0xc8);
        display(0x95,0xca,0xc7);
        write(COMMAND, 0x96);
        write(DATA, 0x3a);
        write(DATA, 0x20);
        write(DATA, senddata[2]);
        write(DATA, senddata[3]);        
       
        display(0x89,0xd6,0xd8);//重
        display(0x8a,0xd0,0xc2);//新
        display(0x8b,0xbb,0xf1);//获
        display(0x8c,0xc8,0xa1);//取


        display(0x98,0xa1,0xbe);         
        display(0x99,0xb7,0xb5);
        display(0x9a,0xbb,0xd8);
        display(0x9b,0xa1,0xbf);        
}

void fun10(void) //功能函数，获取温湿度 
{
        GET_DHT11DATA();
        
        func_index = 7;
        
        current_operation_index=table[func_index].current_operation;
        (*current_operation_index)();
}

/*---------------------------------------------------------------------------*/

key_table table[30]=
{
	
	{0,2,1,3,(*fun1)},//第一层，显示功能菜单、【传感器】、时间、设置

	{1,0,2,0,(*fun2)},//第一层，显示功能菜单、传感器、【时间】、设置

	{2,1,0,0,(*fun3)},//第一层，显示功能菜单、传感器、时间、【设置】                     

	{3,6,4,7,(*fun4)},//第二层，显示【温湿度传感器】、烟雾传感器、门磁传感器、返回

	{4,3,5,0,(*fun5)},//第二层，显示温湿度传感器、【烟雾传感器】、门磁传感器、返回                                                   

	{5,4,6,0,(*fun6)},//第二层，显示温湿度传感器、烟雾传感器、【门磁传感器】、返回      

	{6,5,3,0,(*fun7)}, //第二层，显示温湿度传感器、烟雾传感器、门磁传感器、【返回】                                                                             

	{7,8,8,9,(*fun8)},//第三层，显示现在的温度是、现在的湿度是、【重新获取】、返回      

	{8,7,7,3,(*fun9)},//第三层，显示现在的温度是、现在的湿度是、重新获取、【返回】                                                

	{9,9,9,9,(*fun10)},//功能函数，获取温湿度  
/*
	{10,9,11,21,(*fun11)}, //第二层，北京大学层下显示历史、政治、【简介】、返回                                                                                

	{11,10,8,1,(*fun12)},//第二层，北京大学层下显示历史、政治、简介、【返回】         

	{12,15,13,22,(*fun13)},//第二层，重庆三峡学院层下显示【简介】、最佳院系、最佳实验室、返回                                                        

	{13,12,14,23,(*fun14)}, //第二层，重庆三峡学院层下显示简介、【最佳院系】、最佳实验室、返回                                                               

	{14,13,15,24,(*fun15)}, //第二层，重庆三峡学院层下显示简介、最佳院系、【最佳实验室】、返回                                                               

	{15,14,12,2,(*fun16)}, //第二层，重庆三峡学院层下显示简介、最佳院系、最佳实验室、【返回】   

	{16,16,16,4,(*fun17)}, //第三层，清华大学地点层                                                                    

	{17,17,17,5,(*fun18)}, //第三层，清华大学创时间层                                                      

	{18,18,18,6,(*fun19)}, //第三层，清华大学简介层

	{19,19,19,8,(*fun20)}, //第三层，北京大学历史层                                                                    

	{20,20,20,9,(*fun21)}, //第三层，北京大学政治层                                                           

	{21,21,21,10,(*fun22)}, //第三层，北京大学简介层

	{22,22,22,12,(*fun23)}, //第三层，重庆三峡学院简介层                                                                  

	{23,23,23,13,(*fun24)}, //第三层，重庆三峡学院最佳院系层                                                        

	{24,24,24,14,(*fun25)}, //第三层，重庆三峡学院最佳实验室层        

	{25,25,25,0,(*fun26)}, //第0层      
	*/														
};
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

void clrram(void)
{
  write(COMMAND,0x30);
  write(COMMAND,0x01); //清除显示指令。
  clock_delay_usec(1000);
}

void initlcm(void)
{
	timer_set(&delaytimer12864, DELAY_100MS);
	while(!(timer_expired(&delaytimer12864)));
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