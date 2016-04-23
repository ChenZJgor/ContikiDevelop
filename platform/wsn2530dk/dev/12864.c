#include "contiki.h"
#include "dev/12864.h"
#include "dev/port.h"
#include "dev/dht11.h"

static struct timer delaytimer12864;

void (*current_operation_index)();

/*---------------------------------------------------------------------------*/
void fun1(void) //��һ�㣬��ʾ���ܲ˵���������������ʱ�䡢����
{
        clrram(); //��������ͬ
        display(0x82,0xb9,0xa6); 
        display(0x83,0xc4,0xdc); 
        display(0x84,0xb2,0xcb); 
        display(0x85,0xb5,0xa5); 
        
        write(COMMAND, 0x90); //����Һ��������ʾ��ַ,��ͬ
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

void fun2(void) //��һ�㣬��ʾ���ܲ˵�������������ʱ�䡿������
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
        
	write(COMMAND, 0x88); //����Һ��������ʾ��ַ,��ͬ.
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

void fun3(void) //��һ�㣬��ʾ���ܲ˵�����������ʱ�䡢�����á�
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
        
	write(COMMAND, 0x88); //����Һ��������ʾ��ַ,��ͬ.
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

void fun4(void) //�ڶ��㣬��ʾ����ʪ�ȴ��������������������ŴŴ�����������
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

void fun5(void) //�ڶ��㣬��ʾ��ʪ�ȴ����������������������ŴŴ�����������
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

void fun6(void) //�ڶ��㣬��ʾ��ʪ�ȴ��������������������ŴŴ�������������
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

void fun7(void) //�ڶ��㣬��ʾ��ʪ�ȴ������������������ŴŴ������������ء� 
{
        clrram();

        display(0x81,0xce,0xc2); //��
        display(0x82,0xca,0xaa); //ʪ
        display(0x83,0xb6,0xc8); //��
        display(0x84,0xb4,0xab); //��
        display(0x85,0xb8,0xd0); //��
        display(0x86,0xc6,0xf7); //��

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

void fun8(void) //�����㣬��ʾ���ڵ��¶��ǡ����ڵ�ʪ���ǡ������»�ȡ��������
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
        display(0x89,0xd6,0xd8);//��
        display(0x8a,0xd0,0xc2);//��
        display(0x8b,0xbb,0xf1);//��
        display(0x8c,0xc8,0xa1);//ȡ
        display(0x8d,0xa1,0xbf);

        display(0x99,0xb7,0xb5);
        display(0x9a,0xbb,0xd8);   
}

void fun9(void) //�����㣬��ʾ���ڵ��¶��ǡ����ڵ�ʪ���ǡ����»�ȡ�������ء�
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
       
        display(0x89,0xd6,0xd8);//��
        display(0x8a,0xd0,0xc2);//��
        display(0x8b,0xbb,0xf1);//��
        display(0x8c,0xc8,0xa1);//ȡ


        display(0x98,0xa1,0xbe);         
        display(0x99,0xb7,0xb5);
        display(0x9a,0xbb,0xd8);
        display(0x9b,0xa1,0xbf);        
}

void fun10(void) //���ܺ�������ȡ��ʪ�� 
{
        GET_DHT11DATA();
        
        func_index = 7;
        
        current_operation_index=table[func_index].current_operation;
        (*current_operation_index)();
}

/*---------------------------------------------------------------------------*/

key_table table[30]=
{
	
	{0,2,1,3,(*fun1)},//��һ�㣬��ʾ���ܲ˵���������������ʱ�䡢����

	{1,0,2,0,(*fun2)},//��һ�㣬��ʾ���ܲ˵�������������ʱ�䡿������

	{2,1,0,0,(*fun3)},//��һ�㣬��ʾ���ܲ˵�����������ʱ�䡢�����á�                     

	{3,6,4,7,(*fun4)},//�ڶ��㣬��ʾ����ʪ�ȴ��������������������ŴŴ�����������

	{4,3,5,0,(*fun5)},//�ڶ��㣬��ʾ��ʪ�ȴ����������������������ŴŴ�����������                                                   

	{5,4,6,0,(*fun6)},//�ڶ��㣬��ʾ��ʪ�ȴ��������������������ŴŴ�������������      

	{6,5,3,0,(*fun7)}, //�ڶ��㣬��ʾ��ʪ�ȴ������������������ŴŴ������������ء�                                                                             

	{7,8,8,9,(*fun8)},//�����㣬��ʾ���ڵ��¶��ǡ����ڵ�ʪ���ǡ������»�ȡ��������      

	{8,7,7,3,(*fun9)},//�����㣬��ʾ���ڵ��¶��ǡ����ڵ�ʪ���ǡ����»�ȡ�������ء�                                                

	{9,9,9,9,(*fun10)},//���ܺ�������ȡ��ʪ��  
/*
	{10,9,11,21,(*fun11)}, //�ڶ��㣬������ѧ������ʾ��ʷ�����Ρ�����顿������                                                                                

	{11,10,8,1,(*fun12)},//�ڶ��㣬������ѧ������ʾ��ʷ�����Ρ���顢�����ء�         

	{12,15,13,22,(*fun13)},//�ڶ��㣬������ϿѧԺ������ʾ����顿�����Ժϵ�����ʵ���ҡ�����                                                        

	{13,12,14,23,(*fun14)}, //�ڶ��㣬������ϿѧԺ������ʾ��顢�����Ժϵ�������ʵ���ҡ�����                                                               

	{14,13,15,24,(*fun15)}, //�ڶ��㣬������ϿѧԺ������ʾ��顢���Ժϵ�������ʵ���ҡ�������                                                               

	{15,14,12,2,(*fun16)}, //�ڶ��㣬������ϿѧԺ������ʾ��顢���Ժϵ�����ʵ���ҡ������ء�   

	{16,16,16,4,(*fun17)}, //�����㣬�廪��ѧ�ص��                                                                    

	{17,17,17,5,(*fun18)}, //�����㣬�廪��ѧ��ʱ���                                                      

	{18,18,18,6,(*fun19)}, //�����㣬�廪��ѧ����

	{19,19,19,8,(*fun20)}, //�����㣬������ѧ��ʷ��                                                                    

	{20,20,20,9,(*fun21)}, //�����㣬������ѧ���β�                                                           

	{21,21,21,10,(*fun22)}, //�����㣬������ѧ����

	{22,22,22,12,(*fun23)}, //�����㣬������ϿѧԺ����                                                                  

	{23,23,23,13,(*fun24)}, //�����㣬������ϿѧԺ���Ժϵ��                                                        

	{24,24,24,14,(*fun25)}, //�����㣬������ϿѧԺ���ʵ���Ҳ�        

	{25,25,25,0,(*fun26)}, //��0��      
	*/														
};
/*---------------------------------------------------------------------------*/
void sendbyte(unsigned char bbyte) //����һ���ֽ�
{
    PORT_FUNC_GPIO(SID_PORT, SID_PIN);
    PORT_DIR_OUTPUT(SID_PORT, SID_PIN);
    PORT_FUNC_GPIO(CLK_PORT, CLK_PIN);
    PORT_DIR_OUTPUT(CLK_PORT, CLK_PIN);
	
	unsigned char i;
	for(i=0;i<8;i++)
	{
		if(bbyte&0x80)//ȡ�����λ
			PORT_SET(SID_PORT, SID_PIN); 
		else
			PORT_CLEAR(SID_PORT, SID_PIN);
	 PORT_SET(CLK_PORT, CLK_PIN); 
	 PORT_CLEAR(CLK_PORT, CLK_PIN);
	 bbyte <<= 1; //����
	 }  
}

void write(unsigned char start, unsigned char ddata) //дָ�������
{
	unsigned char start_data,Hdata,Ldata;
	if(start == COMMAND)
		start_data = 0xf8;  //дָ��
	else      
		start_data = 0xfa;  //д����

	Hdata = ddata&0xf0;    //ȡ����λ
	Ldata = (ddata<<4)&0xf0;  //ȡ����λ
	sendbyte(start_data);   //������ʼ�ź�
	clock_delay_usec(100); //��ʱ�Ǳ����
	sendbyte(Hdata);       //���͸���λ
	clock_delay_usec(100);  //��ʱ�Ǳ����
	sendbyte(Ldata);    //���͵���λ
	clock_delay_usec(100);  //��ʱ�Ǳ����
}

void clrram(void)
{
  write(COMMAND,0x30);
  write(COMMAND,0x01); //�����ʾָ�
  clock_delay_usec(1000);
}

void initlcm(void)
{
	timer_set(&delaytimer12864, DELAY_100MS);
	while(!(timer_expired(&delaytimer12864)));
	write(COMMAND, 0x30);//�������ã�һ����8λ���ݣ�����ָ�
	write(COMMAND, 0x0C);//0000,1100 ������ʾ���α�off���α�λ��off
	write(COMMAND, 0x01);//0000,0001 ��DDRAM
	write(COMMAND, 0x02);//0000,0010 DDRAM��ַ��λ
	write(COMMAND, 0x80);//1000,0000 �趨DDRAM 7λ��ַ000��0000����ַ������AC
}

void display(unsigned char x_add,unsigned char dat1,unsigned char dat2)
{
	write(COMMAND, x_add);//1xxx,xxxx �趨DDRAM 7λ��ַxxx,xxxx����ַ������AC
	write(DATA, dat1);
	write(DATA, dat2);
}

void set_wenzi(void)
{
	write(COMMAND, 0x80);//1000,0001 �趨DDRAM 7λ��ַ000��0001����ַ������AC.
	write(DATA, 0x48);//��ASCII�������,��ʾ��Һ����Ļ��.��ͬ
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

	write(COMMAND, 0x90); //����Һ��������ʾ��ַ,��ͬ.
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
	display(0x8a,0xc4,0xea); //�������ֵ�����."��"����ֵ��:c4ea.
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