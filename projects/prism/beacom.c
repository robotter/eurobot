#include <avr/io.h>
#include <clock/clock.h>
#include <util/delay.h>
#include <uart/uart.h>
#include <string.h>
#include <util/crc16.h>
#include "battery_monitor.h"
#include "beacom.h"
#include "sensor.h"

#define DEBUG_PRINTF

#define BEACOM_MAGIC_START 'S'
#define BEACOM_MAGIC_END 'E'

typedef struct {
  uint8_t magic_start;
  uint8_t id;
  double period;
  uint8_t objnum;
  double dist[MAX_OBJECT];
  double angle[MAX_OBJECT];
  uint16_t crc;
  uint8_t magic_end;
} __attribute__((packed)) beacom_packet_t;

static int _id = 0;

void beacom_init(void) {
  PORTH.DIRCLR = (1 << 2) | (1 << 3) | (1 << 4);
  PORTH.PIN2CTRL = PORT_OPC_PULLUP_gc;
  PORTH.PIN3CTRL = PORT_OPC_PULLUP_gc;
  PORTH.PIN4CTRL = PORT_OPC_PULLUP_gc;

  PORTH.DIRSET = (1 << 1);
  PORTH.OUTCLR = (1 << 1);
  PORTE.DIRSET = (1 << 0);
  PORTE.OUTCLR = (1 << 0);

  _delay_ms(100); //soft init


  int tmp = PORTH.IN;

  if(!(tmp & (1 << 2))) {
    _id = 1;
  }

  if(!(tmp & (1 << 4))) {
    _id = 2;
  }
}

int beacom_get_id(void) {
  return _id;
}

static void beacom_run_slave(void) {
  while(1) {
    if(sensor_new_data_available()) {
      sensor_latch();
      if(uart_recv_nowait(uartF1) == -1) {
        PORTH.OUTCLR = (1 << 1);
      }
      else {
        PORTH.OUTSET = (1 << 1);
      }

      if(uart_recv_nowait(uartF0) == -1) {
        PORTE.OUTCLR = (1 << 0);
      }
      else {
        PORTE.OUTSET = (1 << 0);
      }
      
      beacom_packet_t pkt;
      pkt.magic_start = BEACOM_MAGIC_START;
      pkt.magic_end = BEACOM_MAGIC_END;
      pkt.id = beacom_get_id();
      pkt.period = sensor_get_period(SENSOR_TOP);
      pkt.objnum = sensor_get_object_number(SENSOR_TOP);
      for(int i = 0; i < MAX_OBJECT; i += 1) {
        pkt.dist[i] = sensor_get_object_distance(SENSOR_TOP, i);
        pkt.angle[i] = sensor_get_object_angle(SENSOR_TOP, i);
      }

      uint8_t * ptr;

      ptr = (uint8_t *)&pkt;
      uint16_t crc = 0xffff;
      for(unsigned int i = 0; i < sizeof(beacom_packet_t) - 3; i += 1, ptr += 1) {
        crc = _crc_ccitt_update(crc, *ptr); 
      }
      pkt.crc = crc;
      
      ptr = (uint8_t *)&pkt;
      for(unsigned int i = 0; i < sizeof(beacom_packet_t); i += 1, ptr += 1) {
        uart_send(uartF0, *ptr);
        uart_send(uartF1, *ptr);
      }
    }
    mirror_speed_correct();
  }
}

static int beacom_crc_is_valid(beacom_packet_t * pkt) {
  uint8_t * ptr = (uint8_t *)pkt;

  uint16_t crc = 0xffff;
  for(unsigned int i = 0; i < sizeof(beacom_packet_t) - 3; i += 1, ptr += 1) {
    crc = _crc_ccitt_update(crc, *ptr); 
  }

  return crc == pkt->crc;
}

static void beacom_run_master(void) {
  beacom_packet_t beacons_latched[2];
  beacom_packet_t * beacons_tmp[2];
  uint8_t beacons_tmp_buf[2][sizeof(beacom_packet_t)];

  memset(&beacons_latched[0], 0, sizeof(beacom_packet_t));
  memset(&beacons_latched[1], 0, sizeof(beacom_packet_t));
  memset(&beacons_tmp_buf[0], 0, sizeof(beacom_packet_t));
  memset(&beacons_tmp_buf[1], 0, sizeof(beacom_packet_t));
  beacons_tmp[0] = (beacom_packet_t *)&beacons_tmp_buf[0];
  beacons_tmp[1] = (beacom_packet_t *)&beacons_tmp_buf[1];



  while(1) {
    int recv;
    
    recv = uart_recv_nowait(uartF0);
    if(recv != -1) {
      for(unsigned int i = 0; i < sizeof(beacom_packet_t) - 1; i += 1) {
        beacons_tmp_buf[0][i] = beacons_tmp_buf[0][i + 1];
      }
      beacons_tmp_buf[0][sizeof(beacom_packet_t) - 1] = recv;
    }

    recv = uart_recv_nowait(uartF1);
    if(recv != -1) {
      for(unsigned int i = 0; i < sizeof(beacom_packet_t) - 1; i += 1) {
        beacons_tmp_buf[1][i] = beacons_tmp_buf[1][i + 1];
      }
      beacons_tmp_buf[1][sizeof(beacom_packet_t) - 1] = recv;
    }

    for(int i = 0; i < 2 ; i += 1) {
      if((beacons_tmp[i]->magic_start == BEACOM_MAGIC_START)) {
        if((beacons_tmp[i]->magic_end == BEACOM_MAGIC_END)) {
          if(beacom_crc_is_valid(beacons_tmp[i])) {
            if(i == 0) {
              PORTE.OUTSET = (1 << 0);
            }
            else {
              PORTH.OUTSET = (1 << 1);
            }
            memcpy(&beacons_latched[beacons_tmp[i]->id & 0x01], beacons_tmp[i], sizeof(beacom_packet_t));
          }
          memset(&beacons_tmp_buf[i], 0, sizeof(beacom_packet_t));
        }
      }
    }

    if(sensor_new_data_available()) {
      sensor_latch();
      PORTE.OUTCLR = (1 << 0);
      PORTH.OUTCLR = (1 << 1);
      uart_send(uartF0, 'P');
      uart_send(uartF1, 'P');

#ifdef DEBUG_PRINTF
      double per = sensor_get_period(SENSOR_TOP);
      int count = sensor_get_object_number(SENSOR_TOP);

      printf("\033[2J");
      printf("----------------\r\n");
      printf("Bat:\r\n");
      printf("\t - %s\r\n", BATTMON_IsBatteryDischarged() == BATTERY_DISCHARGED ? "KO" : "OK");
      printf("\t - %dmV\r\n", BATTMON_GetVoltage_mV());
      printf("B0:\r\n");
    printf("\t - per:%.2fms\r\n",per);
      printf("\t - obj:%d ",count);
      for(int i = 0; i < count; i += 1) {
        printf("{a:%.2fdeg d:%.2fdeg} ",sensor_get_object_angle(SENSOR_TOP,i), sensor_get_object_distance(SENSOR_TOP,i));
      }
      printf("\r\n");

      for(int i = 0; i < 2 ; i += 1) {
        printf("B%d:\r\n",i + 1);
        printf("\t - per:%.2fms\r\n",beacons_latched[i].period);
        printf("\t - obj:%d ",beacons_latched[i].objnum);
        for(int o = 0; o < beacons_latched[i].objnum; o += 1) {
          printf("{a:%.2fdeg d:%.2fdeg} ",beacons_latched[i].dist[o], beacons_latched[i].angle[o]);
        }
        printf("\r\n");
      }
#endif
    }
    
    mirror_speed_correct();
  }
}

void beacom_run(void) {
  switch(beacom_get_id()) {
    case 0:
      beacom_run_master();
      break;

    default:
      beacom_run_slave();
      break;
  }
}
