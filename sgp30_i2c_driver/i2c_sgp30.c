#include <string.h>
#include <stdio.h>

#include "i2c_sgp30.h"

#define MSB_OF_UINT16_T(d) ( (uint8_t)(d >> 8) )
#define LSB_OF_UINT16_T(d) ( (uint8_t)(d & 0x00ff) )

static uint8_t s_sgp30_crc(uint8_t *p, int len)
{
  int i;
  unsigned char crc = 0xff;

  while(len--){
    crc ^= *p++;
    for (i = 0; i < 8; ++i){
      if (crc & 0x80){
        crc = (crc << 1) ^ 0x31;
      }else{
        crc <<= 1;
      }
    }
  }

  return crc;
}

static int s_i2c_sgp30_send_cmd(i2c_sgp30_handle_t handle, uint16_t cmd, uint32_t duration_ms){
    uint8_t arr_cmd[2];
    arr_cmd[0] = MSB_OF_UINT16_T(cmd);
    arr_cmd[1] = LSB_OF_UINT16_T(cmd);

    if (0 != handle->i2c_write(handle->i2c_addr, arr_cmd, sizeof(arr_cmd))){
        return SGP30_ERR;
    }

    handle->msleep(duration_ms);
    return SGP30_OK;
}

#define MAX_WRITE_DATA_LEN      128
static int s_i2c_sgp30_send_write_cmd(i2c_sgp30_handle_t handle, 
                    uint16_t cmd, uint8_t *data, size_t data_len, uint32_t duration_ms){
    if (data_len > MAX_WRITE_DATA_LEN - sizeof(cmd)){
        return SGP30_ERR;
    }
    int i = 0;
    uint8_t w_data[MAX_WRITE_DATA_LEN];
    w_data[i++] = MSB_OF_UINT16_T(cmd);
    w_data[i++] = LSB_OF_UINT16_T(cmd);

    for (int d_i = 0; d_i < data_len; ++d_i){
        w_data[i++] = data[d_i];
    }

    if (0 != handle->i2c_write(handle->i2c_addr, w_data, i)){
        return SGP30_ERR;
    }

    handle->msleep(duration_ms);
    return SGP30_OK;
}

//在“Init_air_quality”命令之后,必须定期发送“Measure_air_quality”命令,间隔为1秒,以确保动态基线补偿算法
//的正常运行。
int i2c_sgp30_init(i2c_sgp30_handle_t      handle){
    uint16_t cmd = CMD_Init_air_quality;
    uint32_t duration = CMD_Init_air_quality_MAX_DURATION_TIME_MS;

    if ( SGP30_OK != s_i2c_sgp30_send_cmd(handle, cmd, duration)){
        return SGP30_ERR;
    }

    return SGP30_OK;
}

int i2c_sgp30_measure_air_quality(i2c_sgp30_handle_t handle, uint16_t * co2eq, uint16_t * tvoc){
    uint16_t cmd = CMD_Measure_air_quality;
    uint32_t duration = CMD_Measure_air_quality_MAX_DURATION_TIME_MS;

    if ( SGP30_OK != s_i2c_sgp30_send_cmd(handle, cmd, duration)){
        return SGP30_ERR;
    }

    uint8_t out[6] = {0};
    handle->i2c_read(handle->i2c_addr, out, sizeof(out));

    // crc check
    uint8_t crc;
    crc = s_sgp30_crc(&out[0], 2);
    if (crc != out[2]){
        return SGP30_CRC_ERR;
    }
    crc = s_sgp30_crc(&out[3], 2);
    if (crc != out[5]){
        return SGP30_CRC_ERR;
    }

    *co2eq = (out[0] << 8 | out[1] );
    *tvoc = (out[3] << 8 | out[4] );
    return SGP30_OK;
}

int i2c_sgp30_get_baseline(i2c_sgp30_handle_t handle, uint16_t * baseline_co2eq, uint16_t * baseline_tvoc){
    uint16_t cmd = CMD_Get_baseline;
    uint32_t duration = CMD_Get_baseline_MAX_DURATION_TIME_MS;

    if ( SGP30_OK != s_i2c_sgp30_send_cmd(handle, cmd, duration)){
        return SGP30_ERR;
    }

    uint8_t out[6] = {0};
    handle->i2c_read(handle->i2c_addr, out, sizeof(out));

    // crc check
    uint8_t crc;
    crc = s_sgp30_crc(&out[0], 2);
    if (crc != out[2]){
        return SGP30_CRC_ERR;
    }
    crc = s_sgp30_crc(&out[3], 2);
    if (crc != out[5]){
        return SGP30_CRC_ERR;
    }

    *baseline_co2eq = (out[0] << 8 | out[1] );
    *baseline_tvoc = (out[3] << 8 | out[4] );

    return SGP30_OK;
}

// function not test!!
int i2c_sgp30_set_baseline(i2c_sgp30_handle_t handle, uint16_t baseline_co2eq, uint16_t baseline_tvoc){
    uint16_t cmd = CMD_Set_baseline;
    uint32_t duration = CMD_Set_baseline_MAX_DURATION_TIME_MS;

    uint8_t data[6] = {0};
    data[0] = MSB_OF_UINT16_T(baseline_co2eq);
    data[1] = LSB_OF_UINT16_T(baseline_co2eq);
    data[2] = s_sgp30_crc(&data[0], 2);

    data[3] = MSB_OF_UINT16_T(baseline_tvoc);
    data[4] = LSB_OF_UINT16_T(baseline_tvoc);
    data[5] = s_sgp30_crc(&data[3], 2);

    if (SGP30_OK != s_i2c_sgp30_send_write_cmd(handle, cmd, data, sizeof(data), duration)){
        return SGP30_ERR;
    }

    return SGP30_OK;
}

// 湿度补偿可以设置一下。要买一个湿度传感器。(this function not test!!)
int i2c_sgp30_set_humidity(i2c_sgp30_handle_t handle, uint16_t humidity){
    uint16_t cmd = CMD_Set_humidity;
    uint32_t duration = CMD_Set_humidity_MAX_DURATION_TIME_MS;

    uint8_t data[3] = {0};
    data[0] = MSB_OF_UINT16_T(humidity);
    data[1] = LSB_OF_UINT16_T(humidity);
    data[2] = s_sgp30_crc(&data[0], 2);

    if (SGP30_OK != s_i2c_sgp30_send_write_cmd(handle, cmd, data, sizeof(data), duration)){
        return SGP30_ERR;
    }

    return SGP30_OK;
}
// todo: 添加log打印到这里来，方便提示错误？？？？？？？
int i2c_sgp30_measure_test(i2c_sgp30_handle_t handle, uint16_t * data){
    uint16_t cmd = CMD_Measure_test;
    uint32_t duration = CMD_Measure_test_MAX_DURATION_TIME_MS;

    if ( SGP30_OK != s_i2c_sgp30_send_cmd(handle, cmd, duration)){
        return SGP30_ERR;
    }

    uint8_t out[3] = {0};
    handle->i2c_read(handle->i2c_addr, out, sizeof(out));

    // crc check
    uint8_t crc;
    crc = s_sgp30_crc(&out[0], 2);
    if (crc != out[2]){
        return SGP30_CRC_ERR;
    }

    *data = (out[0] << 8 | out[1] );
    return SGP30_OK;
}

int i2c_sgp30_get_feature_set_version(i2c_sgp30_handle_t handle, uint8_t * version){
    uint16_t cmd = CMD_Get_feature_set_version;
    uint32_t duration = CMD_Get_feature_set_version_MAX_DURATION_TIME_MS;

    if ( SGP30_OK != s_i2c_sgp30_send_cmd(handle, cmd, duration)){
        return SGP30_ERR;
    }

    uint8_t out[3] = {0};
    handle->i2c_read(handle->i2c_addr, out, sizeof(out));

    // check crc.
    uint8_t crc;
    crc = s_sgp30_crc(&out[0], 2);
    if (crc != out[2]){
        return SGP30_CRC_ERR;
    }

    version[0] = out[0];
    version[1] = out[1];
    return SGP30_OK;
}


int i2c_sgp30_measure_raw_signals(i2c_sgp30_handle_t handle, uint16_t * H2_signal , uint16_t * Ethanol_signal){
    uint16_t cmd = CMD_Measure_raw_signals;
    uint32_t duration = CMD_Measure_raw_signals_MAX_DURATION_TIME_MS;

    if ( SGP30_OK != s_i2c_sgp30_send_cmd(handle, cmd, duration)){
        return SGP30_ERR;
    }

    uint8_t out[6] = {0};
    handle->i2c_read(handle->i2c_addr, out, sizeof(out));

    // crc check
    uint8_t crc;
    crc = s_sgp30_crc(&out[0], 2);
    if (crc != out[2]){
        return SGP30_CRC_ERR;
    }
    crc = s_sgp30_crc(&out[3], 2);
    if (crc != out[5]){
        return SGP30_CRC_ERR;
    }

    *H2_signal = (out[0] << 8 | out[1] );
    *Ethanol_signal = (out[3] << 8 | out[4] );
    return SGP30_OK;
}

// todo： SGP30 动态基线补偿算法怎么实现的？

