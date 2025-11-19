#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>
#include <zephyr/sys/slist.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>



/* 自定义 UUID */
#define BT_UUID_IMU_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x1234567890ab)

#define BT_UUID_IMU_CHAR_VAL \
    BT_UUID_128_ENCODE(0x12345679, 0x1234, 0x5678, 0x1234, 0x1234567890ab)

static struct bt_uuid_128 imu_svc_uuid = BT_UUID_INIT_128(BT_UUID_IMU_SERVICE_VAL);
static struct bt_uuid_128 imu_char_uuid = BT_UUID_INIT_128(BT_UUID_IMU_CHAR_VAL);

/* 用来临时放要通知出去的数据 */
static uint8_t imu_notify_data[12];

static void imu_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("IMU notify %s\n", notif_enabled ? "enabled" : "disabled");
}

/* 声明 GATT 服务：一个 Primary Service + 一个支持 notify 的特征 */
BT_GATT_SERVICE_DEFINE(imu_svc,
    BT_GATT_PRIMARY_SERVICE(&imu_svc_uuid),
    BT_GATT_CHARACTERISTIC(&imu_char_uuid.uuid,
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, imu_notify_data),
    BT_GATT_CCC(imu_ccc_cfg_changed,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);




// ========== 线程定义和全局变量 ==========
#define SAMPLE_FREQUENCY 104    // 传感器采样频率 (Hz)
#define CONSUMER_RATE_MS 50     // 消费者 (发送) 频率 (ms) -> 20 Hz


// 传感器数据结构体：用于存储单个样本，推入 k_fifo
struct imu_sample {
    sys_snode_t node; // k_fifo 需要的节点
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
};


// 全局传感器设备指针：方便主线程和传感器线程共享访问
const struct device *sensor = DEVICE_DT_GET_ONE(st_lsm6dsl);

// 消息队列：用于传感器 ISR (中断服务程序) 和工作项 (延迟发送) 之间传输数据
K_FIFO_DEFINE(imu_data_fifo);

void consumer_work_handler(struct k_work *work);

// 延迟工作项：用于周期性地处理队列并发送数据
static K_WORK_DELAYABLE_DEFINE(producer_work, consumer_work_handler); 




// 从 k_fifo 中取出所有样本，打包、发送
void consumer_work_handler(struct k_work *work)
{
    struct imu_sample *sample_ptr;
    int samples_count = 0;
    double ideal_samples = (double)SAMPLE_FREQUENCY * CONSUMER_RATE_MS / 1000.0;

    printf("\n[Consumer] === 开始批量传输 ===\n");

    // 从队列中取出所有积攒的样本
    while ((sample_ptr = k_fifo_get(&imu_data_fifo, K_NO_WAIT)) != NULL) {
        samples_count++;
        
        // 在这里进行数据打包/复制到发送缓冲区
        printf("[Consumer] 样本 %d: Accel X=%.2f m/s², Gyro Z=%.2f °/s\n", samples_count,
               sensor_value_to_double(&sample_ptr->accel[0]),
               sensor_value_to_double(&sample_ptr->gyro[2]));

               
               /* --- 新增：把 accel 三轴打包成 3 个 float（12 字节） --- */
        float ax = (float)sensor_value_to_double(&sample_ptr->accel[0]);
        float ay = (float)sensor_value_to_double(&sample_ptr->accel[1]);
        float az = (float)sensor_value_to_double(&sample_ptr->accel[2]);

        memcpy(&imu_notify_data[0], &ax, sizeof(float));
        memcpy(&imu_notify_data[4], &ay, sizeof(float));
        memcpy(&imu_notify_data[8], &az, sizeof(float));

        /* 通过 GATT Notify 发出去（属性索引 1 对应上面特征） */
        bt_gatt_notify(NULL, &imu_svc.attrs[1], imu_notify_data, sizeof(imu_notify_data));


        // 释放内存
        k_free(sample_ptr);
    }
    
    // 检查并报告
    if (samples_count > 0) {
        printf("[Consumer] 成功打包并释放 %d 个样本。理想样本数: %.1f\n", 
               samples_count, ideal_samples);
    } else {
        printf("[Consumer] 队列为空，没有样本可发送。\n");
    }

    // 重新启动延迟工作项 (20 Hz 周期性运行)
    k_work_reschedule(&producer_work, K_MSEC(CONSUMER_RATE_MS));
}


// 生产者：传感器触发器回调函数 (104 Hz)
static void sensor_trigger_handler(const struct device *dev,
                                 const struct sensor_trigger *trigger)
{
    struct imu_sample *new_sample;

    // 获取数据
    if (sensor_sample_fetch(dev) < 0) {
        printk("[Producer] 传感器数据获取失败\n");
        return;
    }

    // 分配内存给新的样本
    new_sample = k_malloc(sizeof(struct imu_sample));
    if (new_sample == NULL) {
        printk("[Producer] 内存分配失败，样本丢失\n");
        return;
    }

    // 从驱动程序获取数据
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &new_sample->accel[0]);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &new_sample->accel[1]);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &new_sample->accel[2]);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &new_sample->gyro[0]);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &new_sample->gyro[1]);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &new_sample->gyro[2]);

    //  将样本推入队列
    k_fifo_put(&imu_data_fifo, new_sample);

    printk("[Producer] 样本推入队列: Accel Z=%.2f\n", 
           sensor_value_to_double(&new_sample->accel[2]));
}


/* 蓝牙初始化完成后的回调 */
static void bt_ready(int err)
{
    if (err) {
        printk("[BT] Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("[BT] Bluetooth ready, start advertising\n");

    /* 开始广播，允许连接，并带上设备名 */
    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, NULL, 0, NULL, 0);
    if (err) {
        printk("[BT] Advertising failed to start (err %d)\n", err);  // 打印广告启动错误
    } else {
        printk("[BT] Advertising started successfully\n");
    }
}




// main函数
int main()
{
    int err;

    /* 1. 初始化蓝牙 */
    err = bt_enable(bt_ready);
    if (err) {
        printk("[BT] Bluetooth enable failed (err %d)\n", err);
    }


    if (!device_is_ready(sensor)) {
        printf("[Main] 错误: IMU传感器未就绪\n");
        return 0;
    }

    printf("[Main] IMU传感器初始化成功\n");

    /* --- 新增代码开始 --- */
    struct sensor_value odr_attr;

    /* 设置加速度计和陀螺仪的采样频率为 104 Hz */
    odr_attr.val1 = SAMPLE_FREQUENCY; // 104 Hz
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
    
    /* --- 配置和启用传感器触发器 (生产者) --- */
    struct sensor_trigger trig = {
        .type = SENSOR_TRIG_DATA_READY, // 数据就绪触发
        .chan = SENSOR_CHAN_ALL,
    };
    

   // 绑定触发器和回调函数
    if (sensor_trigger_set(sensor, &trig, sensor_trigger_handler) < 0) {
        printf("[Main] 无法设置传感器触发器\n");
        return 0;
    }  
    printf("[Main] 传感器线程已启动\n");


    // 启动延迟工作项
    // 立即启动第一次运行，之后它会在 consumer_work_handler 中自己每 50 ms 重新调度
    k_work_reschedule(&producer_work, K_MSEC(CONSUMER_RATE_MS));
    printf("[Main] 延迟工作项 (消费者, %d ms) 已启动。\n", CONSUMER_RATE_MS);


    // 主线程进入低频空闲循环
    while (1) {
        printf("[Main] 主线程空闲...\n");
        k_sleep(K_MSEC(3000)); 
    }

    return 0;
}
