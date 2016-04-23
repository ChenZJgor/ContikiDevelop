#ifndef __12864_H__
#define __12864_H__

#define COMMAND 0
#define DATA 1

#define SID_PORT 0
#define SID_PIN 5

#define CLK_PORT 0
#define CLK_PIN 6

#define DELAY_100MS 6

typedef struct
{

	unsigned char current;
	unsigned char up;//向上翻索引号
	unsigned char down;//向下翻索引号
	unsigned char enter;//确认索引号
	void (*current_operation)();

}key_table;

extern key_table table[30];

extern unsigned char senddata[4];

extern unsigned char func_index;

void write(unsigned char start, unsigned char ddata);
void display(unsigned char x_add,unsigned char dat1,unsigned char dat2);
void clrram(void);

extern void set_wenzi(void);
extern void initlcm(void);

#endif /* 12864.h */

