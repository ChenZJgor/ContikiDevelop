#ifndef __DHT11_H__
#define __DHT11_H__

#include "dev/port.h"
#include "contiki-conf.h"
#include "lib/sensors.h"

#define DHT11_DATA_PORT 1
#define DHT11_DATA_PIN 6
#define INIT_TIME 3
#define DHT11_DATA_PORT_HIGH 1

extern unsigned char senddata[4];

extern unsigned char GET_DHT11DATA(void);

#endif /* dht11.h */
