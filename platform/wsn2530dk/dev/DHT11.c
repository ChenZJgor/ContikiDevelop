#include "DHT11.h"

static struct timer delaytimer;
static unsigned char DHT11_DATA[5] = {0};

extern const struct sensors_sensor dht11_sensor;

/*---------------------------------------------------------------------------*/
static int
value(int type)
{
    unsigned char byte, bit, tem;

    PORT_FUNC_GPIO(DHT11_DATA_PORT, DHT11_DATA_PIN);
    PORT_DIR_OUTPUT(DHT11_DATA_PORT, DHT11_DATA_PIN);
    PORT_CLEAR(DHT11_DATA_PORT, DHT11_DATA_PIN);
    timer_set(&delaytimer, INIT_TIME);
    while(!(timer_expired(&delaytimer)));
    PORT_SET(DHT11_DATA_PORT, DHT11_DATA_PIN);
    clock_delay(30);
    PORT_DIR_INPUT(DHT11_DATA_PORT, DHT11_DATA_PIN);
    while(!(PORT_READ(DHT11_DATA_PORT, DHT11_DATA_PIN) & DHT11_DATA_PORT_HIGH));
    while(PORT_READ(DHT11_DATA_PORT, DHT11_DATA_PIN) & DHT11_DATA_PORT_HIGH);

    for(byte=0;byte<5;byte++){
        for(bit=0;bit<8;bit++){
            while(!(PORT_READ(DHT11_DATA_PORT, DHT11_DATA_PIN) & DHT11_DATA_PORT_HIGH));
            clock_delay(30);
            tem = 0;
            if((PORT_READ(DHT11_DATA_PORT, DHT11_DATA_PIN)) && DHT11_DATA_PORT_HIGH){
                tem = 1;
            }
            *DATA <<= 1;
            if(tem){
                *DATA |= 0x01;
            }
        }
        DATA++;
    }

    if( (DHT11_DATA[0] + DHT11_DATA[1] + DHT11_DATA[2] + DHT11_DATA[3]) == DHT11_DATA[4]){
        return 1;
    }else return 0;
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int value)
{
    return 1;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  return 1;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(dht11_sensor, DHT11_SENSOR, value, configure, status)
