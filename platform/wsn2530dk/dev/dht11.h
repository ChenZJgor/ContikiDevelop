#ifndef __DHT11_H__
#define __DHT11_H__

#include "dev/port.h"
#include "contiki-conf.h"
#include "lib/sensors.h"

#define DHT11_DATA_PORT 0
#define DHT11_DATA_PIN 7
#define INIT_TIME 3
#define DHT11_DATA_PORT_HIGH 1

extern const struct sensors_sensor dht11_sensor;
static unsigned char DATA_HIGH=0,DATA_LOW=0;

#define DHT11_SENSOR "DHT"
#define DHT11ACTIVE() dht11_sensor.configure(SENSORS_ACTIVE, 1)

#endif /* dht11.h */
