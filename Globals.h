// Central shared data declarations for cross-module use
#pragma once

#include <Arduino.h>
#include <Ethernet.h>
#include "Config.h"

// Exported data array (double in RAM, cast to float32 on wire)
extern double   send_arr[SEND_ARR_SIZE + 1];

// Accumulators for minute-average and std-dev
extern float    acc_sum[CH_COUNT];
extern float    acc_sq_sum[CH_COUNT];
extern uint16_t acc_count;
extern uint32_t send_arr_last_update;
extern uint32_t last_sec_tick;

// Scratch/increment helper
extern int      tmp_id_value;

// Decoded values from modules
extern double   meteo_dec[9];
extern float    radiation_uSvh;
extern float    sensors_dec[9];
extern float    service_t[2];

// Modbus TCP server handle
extern EthernetServer server;

