/*
 * gps.c
 *
 *  Created on: May 3, 2025
 *      Author: zhang
 */
#include "gps.h"

GPS_Data_GNRMC gps_data;             // 声明全局变量
uint8_t gps_buffer[GPS_BUFFER_SIZE]; // GPS数据缓冲区
uint8_t gps_index = 0;               // 接收缓冲区索引
uint8_t GPS_timer[32];               // GPS定时器

void GPS_Init(void)
{
    HAL_UART_Receive_DMA(&huart1, (uint8_t *)g_UartRxBuffer, 1); // 启动 DMA 接收
    memset(gps_buffer, 0, sizeof(gps_buffer));
    gps_index = 0;
}

// 东八区时间转换函数
void convert_to_cst(GPS_Data_GNRMC *data)
{
      char *t = data->utc_time;

    uint8_t hour = (t[0]-'0')*10 + (t[1]-'0');
    hour += 8;  // UTC+8
    
    // 处理跨日
    if (hour >= 24) {
        hour -= 24;
        // 如需日期变更可在此添加逻辑
    }

    t[0] = hour/10 + '0';  // 十位
    t[1] = hour%10 + '0';  // 个位

    char hh[3] = {gps_data.utc_time[0], gps_data.utc_time[1], '\0'};
    char mm[3] = {gps_data.utc_time[2], gps_data.utc_time[3], '\0'};
    char ss[7] = {gps_data.utc_time[4], gps_data.utc_time[5], '.',
                  gps_data.utc_time[7], gps_data.utc_time[8], gps_data.utc_time[9], '\0'};

    snprintf(GPS_timer, 32, "%s:%s:%s\r\n", hh, mm, ss); // 格式化为 hh:mm:ss.sss
}

/*
** @brief 解析GNRMC协议数据
** @param nmea_sentence 输入的NMEA句子
** @param data 解析后的数据结构体
** @return true 解析成功
** @return false 解析失败
** @note 解析GNRMC协议数据，返回解析后的数据结构体
** @note 解析规则：
** 1. UTC时间（hhmmss.sss）
** 2. 状态（A=有效，V=无效）
** 3. 纬度（ddmm.mmmm）
** 4. 纬度方向（N/S）
** 5. 经度（dddmm.mmmm）
** 6. 经度方向（E/W）
** 7. 速度（节）
** 8. 航向（度）
** 9. 日期（ddmmyy）
** 10. 磁偏角（可选字段）
** 11. 磁偏角方向（可选字段）
** 12. 模式（可选字段）
** 13. 校验和（*hh）
** @note 解析成功返回true，失败返回false
** @note 解析失败的原因可能是输入的NMEA句子格式不正确，或者字段数量不足
*/
bool parse_gnrmc(const char *nmea_sentence, GPS_Data_GNRMC *data)
{
    // 协议头检查
    if (strstr(nmea_sentence, "GNRMC") == NULL)
    {
        return false;
    }

    // 缓冲区安全拷贝
    char buffer[128];
    strncpy(buffer, nmea_sentence, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    // 字段解析
    char *token = strtok(buffer, ",");
    if (token == NULL)
        return false;

    // 协议头
    strncpy(data->protocol, token, sizeof(data->protocol) - 1);
    data->protocol[sizeof(data->protocol) - 1] = '\0';

    // 1. UTC时间（关键修改点）
    if ((token = strtok(NULL, ",")) == NULL)
        return false;
    strncpy(data->utc_time, token, sizeof(data->utc_time) - 1);
    data->utc_time[sizeof(data->utc_time) - 1] = '\0';
     convert_to_cst(data); // 转换为东八区时间
    // 2. 状态
    token = strtok(NULL, ",");
    if (token == NULL || (*token != 'A' && *token != 'V'))
        return false;
    data->status = *token;

    // 3. 纬度（格式：ddmm.mmmm）
    token = strtok(NULL, ",");
    if (token == NULL)
        return false;
    double lat_deg = atof(token) / 100.0; // 转换为度
    int lat_deg_int = (int)lat_deg;
    data->latitude = lat_deg_int + (lat_deg - lat_deg_int) * 100 / 60.0;

    // 4. 纬度方向
    token = strtok(NULL, ",");
    if (token == NULL || (*token != 'N' && *token != 'S'))
        return false;
    data->lat_dir = *token;

    // 5. 经度（格式：dddmm.mmmm）
    token = strtok(NULL, ",");
    if (token == NULL)
        return false;
    double lon_deg = atof(token) / 100.0;
    int lon_deg_int = (int)lon_deg;
    data->longitude = lon_deg_int + (lon_deg - lon_deg_int) * 100 / 60.0;

    // 6. 经度方向
    token = strtok(NULL, ",");
    if (token == NULL || (*token != 'E' && *token != 'W'))
        return false;
    data->lon_dir = *token;

    // 7. 速度（节）
    token = strtok(NULL, ",");
    if (token == NULL)
        return false;
    data->speed_knots = atof(token);

    // 8. 航向（度）
    token = strtok(NULL, ",");
    if (token == NULL)
        return false;
    data->course_deg = atof(token);

    // 9. 日期（ddmmyy）
    token = strtok(NULL, ",");
    if (token == NULL || strlen(token) != 6)
        return false;
    strncpy(data->date, token, sizeof(data->date) - 1);

    // 10. 磁偏角（可选字段）
    token = strtok(NULL, ",");
    if (token != NULL)
    {
        data->magnetic_var = atof(token);
        token = strtok(NULL, ","); // 可能是模式或校验和
        if (token != NULL)
        {
            // 检查是否为模式字段（单字符）
            if (strlen(token) == 1 && (*token == 'N' || *token == 'A' || *token == 'D' || *token == 'E'))
            {
                data->mode = *token;
                token = strtok(NULL, "*"); // 跳转到校验和
            }
            else
            {
                // 无模式字段，token可能是校验和
                data->mode = 'N'; // 默认无定位
                if (strstr(token, "*") != NULL)
                {
                    token = strtok(token, "*");
                }
            }
        }
    }

    // 11. 校验和（*hh）
    if (token != NULL && strstr(token, "*") == token)
    {
        strncpy(data->checksum, token + 1, sizeof(data->checksum) - 1);
    }
    else
    {
        data->checksum[0] = '\0';
    }
    gps_data.updata = 1;
    return true;
}
