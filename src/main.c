#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

int main()
{
    const struct device *sensor = DEVICE_DT_GET_ONE(st_lsm6dsl);
    
    if (!device_is_ready(sensor)) {
        printf("错误: IMU传感器未就绪\n");
        return 0;
    }




    printf("IMU传感器初始化成功\n");

    /* --- 新增代码开始 --- */
    struct sensor_value odr_attr;

    /* 设置加速度计和陀螺仪的采样频率为 104 Hz */
    odr_attr.val1 = 104; // 104 Hz
    odr_attr.val2 = 0;   // 0 小数部分

    // 设置加速度计的采样频率
    if (sensor_attr_set(sensor, SENSOR_CHAN_ACCEL_XYZ,
                       SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        printf("无法设置加速度计的采样频率\n");
        return 0;
    }
    printf("加速度计采样频率设置为 104 Hz\n");

    // 设置陀螺仪的采样频率 (P26 项目也需要陀螺仪)
    if (sensor_attr_set(sensor, SENSOR_CHAN_GYRO_XYZ,
                       SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        printf("无法设置陀螺仪的采样频率\n");
        return 0;
    }
    printf("陀螺仪采样频率设置为 104 Hz\n");
    /* --- 新增代码结束 --- */
    
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

    return 0;
}