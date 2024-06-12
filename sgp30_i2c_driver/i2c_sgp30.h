#ifndef __I2C_SGP30_DRIVER_H__
#define __I2C_SGP30_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#define SGP30_OK        (0)
#define SGP30_ERR       (1)
#define SGP30_CRC_ERR   (2)


#define CMD_Init_air_quality        0x2003
#define CMD_Measure_air_quality     0x2008
#define CMD_Get_baseline            0x2015
#define CMD_Set_baseline            0x201e
#define CMD_Set_humidity            0x2061
#define CMD_Measure_test            0x2032
#define CMD_Get_feature_set_version 0x202f
#define CMD_Measure_raw_signals     0x2050

#define CMD_Init_air_quality_MAX_DURATION_TIME_MS           (10*4)
#define CMD_Measure_air_quality_MAX_DURATION_TIME_MS        (12*4)
#define CMD_Get_baseline_MAX_DURATION_TIME_MS               (10*4)
#define CMD_Set_baseline_MAX_DURATION_TIME_MS               (10*4)
#define CMD_Set_humidity_MAX_DURATION_TIME_MS               (10*4)
#define CMD_Measure_test_MAX_DURATION_TIME_MS               (220*3)
#define CMD_Get_feature_set_version_MAX_DURATION_TIME_MS    (2*20)
#define CMD_Measure_raw_signals_MAX_DURATION_TIME_MS        (25*4)

typedef int (*i2C_write_ptr)(uint8_t addr, const uint8_t *data, size_t data_len);
typedef int (*i2C_read_ptr)(uint8_t addr, uint8_t * data, size_t buf_len);
typedef void (*sleep_msec_ptr)(uint32_t mseconds);

typedef struct i2c_sgp30_t{
    i2C_write_ptr i2c_write;
    i2C_read_ptr  i2c_read;
    sleep_msec_ptr msleep;
    uint8_t i2c_addr;
} *i2c_sgp30_handle_t;

int i2c_sgp30_init(i2c_sgp30_handle_t handle);
int i2c_sgp30_measure_air_quality(i2c_sgp30_handle_t handle, uint16_t * co2eq, uint16_t * tvoc);
int i2c_sgp30_get_baseline(i2c_sgp30_handle_t handle, uint16_t * baseline_co2eq, uint16_t * baseline_tvoc);
int i2c_sgp30_set_baseline(i2c_sgp30_handle_t handle, uint16_t baseline_co2eq, uint16_t baseline_tvoc);
int i2c_sgp30_set_humidity(i2c_sgp30_handle_t handle, uint16_t humidity);
int i2c_sgp30_measure_test(i2c_sgp30_handle_t handle, uint16_t * data);
int i2c_sgp30_get_feature_set_version(i2c_sgp30_handle_t handle, uint8_t * version);
int i2c_sgp30_measure_raw_signals(i2c_sgp30_handle_t handle, uint16_t * H2_signal , uint16_t * Ethanol_signal);

#ifdef __cplusplus
}
#endif


#endif
