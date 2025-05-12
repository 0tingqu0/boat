/*
 * gps.h
 *
 *  Created on: May 3, 2025
 *      Author: zhang
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <usart.h>
#include <nrf24l01.h>
#include <stdbool.h> // For bool type

#define GPS_UART &huart1 // 假设使用USART1
#define GPS_BUFFER_SIZE 512

typedef struct {
    // 基础字段
    char protocol[6];       // 协议头（如 "GNRMC"）
    char utc_time[10];      // UTC时间（hhmmss.sss）
    char status;            // 状态（A=有效，V=无效）
    double latitude;        // 纬度（ddmm.mmmm）
    char lat_dir;           // 纬度方向（N/S）
    double longitude;       // 经度（dddmm.mmmm）
    char lon_dir;           // 经度方向（E/W）
    float speed_knots;      // 速度（节）
    float course_deg;       // 航向（度）
    char date[6];           // 日期（ddmmyy）
    float magnetic_var;     // 磁偏角
    char var_dir;           // 磁偏角方向（E/W）
    char mode;              // 模式（N=无定位，A=自主定位，D=差分定位，E=估算）*
    char checksum[3];       // 校验和（*hh）
    uint8_t updata;

} GPS_Data_GNRMC;

extern GPS_Data_GNRMC gps_data;             // 声明全局变量
extern uint8_t gps_buffer[GPS_BUFFER_SIZE]; // GPS数据缓冲区
extern uint8_t gps_index;            // 接收缓冲区索引
extern uint8_t GPS_timer[32];               // GPS定时器
extern char g_UartRxBuffer[100]; // 接收缓冲区

void GPS_Init(void);
void convert_to_cst(GPS_Data_GNRMC *data); // 东八区时间转换函数
bool parse_gnrmc(const char *nmea_sentence, GPS_Data_GNRMC *data);

#endif /* INC_GPS_H_ */
