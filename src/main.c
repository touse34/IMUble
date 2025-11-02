#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

void main(void)
{
    const struct device *sensor = DEVICE_DT_GET_ONE(st_lsm6ds3);
    
    if (!device_is_ready(sensor)) {
        printf("错误: IMU传感器未就绪\n");
        return;
    }
    
    printf("IMU传感器初始化成功\n");
    
    while (1) {
        struct sensor_value accel[3];
        
        if (sensor_sample_fetch(sensor) < 0) {
            printf("传感器数据获取失败\n");
            continue;
        }
        
        sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_X, &accel[0]);
        sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_Y, &accel[1]);
        sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_Z, &accel[2]);
        
        printf("加速度: X=%.2f, Y=%.2f, Z=%.2f m/s²\n",
               sensor_value_to_double(&accel[0]),
               sensor_value_to_double(&accel[1]),
               sensor_value_to_double(&accel[2]));
        
        k_sleep(K_MSEC(500));
    }
}