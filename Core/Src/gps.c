/*
 * gps.c
 *
 *  Created on: May 3, 2025
 *      Author: zhang
 */
#include "gps.h"

typedef struct
{
    uint8_t message_id;       // 消息ID
    uint32_t utc_time;        // UTC时间
    float latitude;           // 纬度
    char latitude_direction;  // 纬度方向
    float longitude;          // 经度
    char longitude_direction; // 经度方向
    uint8_t target_state;     // 目标状态
    uint8_t satellite_count;  // 卫星数量
    char Level_Precision_Degradation_Factor; // 水平精度衰减因子
    float sea_level; // 海平面高度
    uint8_t units_height; // 高度单位

} GPS_Data;                   // GPS数据结构体

GPS_Data gps_data; // 初始化GPS数据结构体

DMA_HandleTypeDef hdma_usart1_rx; // 定义DMA句柄
DMA_HandleTypeDef hdma_usart1_tx; // 定义DMA句柄

uint8_t gps_buffer[512]; // GPS数据缓冲区
uint8_t gps_buffer_index = 0; // GPS数据缓冲区索引
uint8_t gps_data_ready = 0; // GPS数据准备好标志
uint8_t gps_data_parsed = 0; // GPS数据解析完成标志
uint8_t gps_data_received = 0; // GPS数据接收完成标志
uint8_t gps_data_error = 0; // GPS数据错误标志

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 接收数据完成，设置标志位
        gps_data_received = 1;
    }
}

