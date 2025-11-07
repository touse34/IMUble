#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>


// ========== 线程定义和全局变量 ==========
#define STACKSIZE 1024       // 新线程的堆栈大小
#define PRIORITY 7           // 新线程的优先级（数字越低，优先级越高）

// 定义堆栈空间：静态分配内存给新线程
K_THREAD_STACK_DEFINE(imu_sensor_stack, STACKSIZE);
// 声明线程结构体对象：内核用于管理此线程
struct k_thread imu_sensor_thread;

// 全局传感器设备指针：方便主线程和传感器线程共享访问
const struct device *sensor = DEVICE_DT_GET_ONE(st_lsm6dsl);




// ========== 新的传感器线程入口函数 ==========
// 负责周期性地获取 IMU 数据
void imu_sensor_entry(void *p1, void *p2, void *p3)
{
    // 传感器任务的主循环
    while (1) {
        struct sensor_value accel[3];
        struct sensor_value gyro[3];

        // 1. 从传感器硬件获取样本数据
        if (sensor_sample_fetch(sensor) < 0) {
            printf("[Task] 传感器数据获取失败\n");
            k_msleep(100);
            continue;
        }
        
        // 2. 从驱动程序获取加速度计数据
        sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_X, &accel[0]);
        sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_Y, &accel[1]);
        sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_Z, &accel[2]);

        // 3. 从驱动程序获取陀螺仪数据（计算角速度）
        sensor_channel_get(sensor, SENSOR_CHAN_GYRO_X, &gyro[0]);
        sensor_channel_get(sensor, SENSOR_CHAN_GYRO_Y, &gyro[1]);
        sensor_channel_get(sensor, SENSOR_CHAN_GYRO_Z, &gyro[2]);
        
        // 4. print
        printf("[Task] 加速度: X=%.2f, Y=%.2f, Z=%.2f m/s²\n",
               sensor_value_to_double(&accel[0]),
               sensor_value_to_double(&accel[1]),
               sensor_value_to_double(&accel[2]));

        printf("[Task] 陀螺仪: X=%.2f, Y=%.2f, Z=%.2f °/s\n",
               sensor_value_to_double(&gyro[0]),
               sensor_value_to_double(&gyro[1]),
               sensor_value_to_double(&gyro[2]));

        // 5. 让出 CPU：线程休眠 500 毫秒，等待下一次运行
        k_sleep(K_MSEC(500));
    }
}




// ========== 主线程函数 (main) ==========
int main()
{
    
    if (!device_is_ready(sensor)) {
        printf("[Main] 错误: IMU传感器未就绪\n");
        return 0;
    }



    printf("[Main] IMU传感器初始化成功\n");

    /* --- 新增代码开始 --- */
    struct sensor_value odr_attr;

    /* 设置加速度计和陀螺仪的采样频率为 104 Hz */
    odr_attr.val1 = 104; // 104 Hz
    odr_attr.val2 = 0;   // 0 小数部分

    // 设置加速度计的采样频率
    if (sensor_attr_set(sensor, SENSOR_CHAN_ACCEL_XYZ,
                       SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        printf("[Main] 无法设置加速度计的采样频率\n");
        return 0;
    }
    printf("[Main] 加速度计采样频率设置为 104 Hz\n");

    // 设置陀螺仪的采样频率 (P26 项目也需要陀螺仪)
    if (sensor_attr_set(sensor, SENSOR_CHAN_GYRO_XYZ,
                       SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        printf("[Main] 无法设置陀螺仪的采样频率\n");
        return 0;
    }
    printf("[Main] 陀螺仪采样频率设置为 104 Hz\n");
    /* --- 新增代码结束 --- */
    
    
    

    // 启动新的传感器线程
    k_thread_create(&imu_sensor_thread,      // 线程对象
                    imu_sensor_stack,        // 堆栈空间
                    STACKSIZE,               // 堆栈大小
                    imu_sensor_entry,        // 线程入口函数
                    NULL, NULL, NULL,        // 传递给线程函数的参数
                    PRIORITY,                // 优先级
                    0,                       // 选项 flags
                    K_NO_WAIT);              // 启动后立即运行
                    
    printf("[Main] 传感器线程已启动！\n");

    // 主线程进入低频空闲循环（可以被其他高优先级线程抢占）
    while (1) {
        printf("[Main] 主线程空闲...\n");
        k_sleep(K_MSEC(3000)); // 每 3 秒打印一次
    }

    return 0;
}