#ifndef __12864_H__
#define __12864_H__

#define COMMAND 0
#define DATA 1

#define SID_PORT 0
#define SID_PIN 5

#define CLK_PORT 0
#define CLK_PIN 6

#define DELAY_100MS 6

extern void set_wenzi(void);
extern void initlcm(void);

#endif /* 12864.h */

