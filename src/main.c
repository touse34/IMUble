#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>       
#include <zephyr/sys/atomic.h>  
#include <stdio.h>               
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/printk.h>


#define SAMPLE_FREQUENCY 104 // 传感器采样频率 (Hz)
#define CONSUMER_RATE_MS 50  // 消费者 (发送) 频率 (ms) -> 20 Hz

// 传感器数据结构体：用于存储单个样本，推入 k_fifo
struct imu_sample
{
        sys_snode_t node; // k_fifo 需要的节点
        struct sensor_value accel[3];
        struct sensor_value gyro[3];
};

// 消息队列：用于传感器 ISR (中断服务程序) 和工作项 (延迟发送) 之间传输数据
K_FIFO_DEFINE(imu_data_fifo);

// 全局传感器设备指针（结构体指针）
const struct device *sensor = DEVICE_DT_GET_ONE(st_lsm6dsl);

//一次发送5个包
#define BATCH_SIZE 6

// 定义通过蓝牙发送的数据包结构 (24 字节)
struct imu_batch_packet_t
{
        uint8_t count;
        struct {
        float accel[3];
        float gyro[3];
    } samples[BATCH_SIZE];    
} __attribute__((packed)); // 编译器指令，不让编译器填充多余字节（虽然float固定4字节，但是保险起见）

//消费者函数
void consumer_work_handler(struct k_work *work);
// 延迟工作项：用于周期性地处理队列并发送数据
static K_WORK_DELAYABLE_DEFINE(producer_work, consumer_work_handler); 

// 标记手机是否开启了通知
static bool imu_ntf_enabled;

static void imu_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

/* Service UUID: ac53c60f-179a-40f9-a1e1-abe320dc8e41 */

/* Characteristic UUID: 3631478e-0a3a-4ccc-8f37-76b12db44564 */

#define BT_UUID_IMU_SERVICE BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xac53c60f, 0x179a, 0x40f9, 0xa1e1, 0xabe320dc8e41))
#define BT_UUID_IMU_DATA_CHAR BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x3631478e, 0x0a3a, 0x4ccc, 0x8f37, 0x76b12db44564))

// 定义GATT服务
BT_GATT_SERVICE_DEFINE(imu_svc,

                       BT_GATT_PRIMARY_SERVICE(BT_UUID_IMU_SERVICE),

                       BT_GATT_CHARACTERISTIC(
                           BT_UUID_IMU_DATA_CHAR, // 特征的 UUID
                           BT_GATT_CHRC_NOTIFY,   // 特征的属性：通知
                           BT_GATT_PERM_NONE,     // 权限：无
                           NULL,                  // 读回调，不开read功能
                           NULL,                  // 写回调，不开write功能
                           NULL                   // 给上面两个函数传参，无
                           ),

                       BT_GATT_CUD("IMU Data", BT_GATT_PERM_READ),

                       BT_GATT_CCC( // CCC通知开关
                           imu_ccc_cfg_changed,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE // 手机 App 必须有权读写这个 CCC 描述符
                           ));

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  BT_UUID_128_ENCODE(0xac53c60f, 0x179a, 0x40f9, 0xa1e1, 0xabe320dc8e41))};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1)};

static ATOMIC_DEFINE(state, 2U);

#define STATE_CONNECTED 1U

#define STATE_DISCONNECTED 2U

static void connected(struct bt_conn *conn, uint8_t err)
{
        if (err)
        {

                // 如果有错误，打印连接失败信息，包括错误码和错误描述

                printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
        }
        else
        {

                // 没错误，打印连接成功！！

                printk("Connected\n");

                // 使用原子操作安全地设置 state 变量的 STATE_CONNECTED 位（第一位），设置值为1
                (void)atomic_set_bit(state, STATE_CONNECTED);
        

        // === 新增：请求交换 MTU ===
        // 这会告诉手机：“我支持大包（247字节），我们也用大包通信吧”
        // 放在这里或者在一个单独的 work 稍微延迟一点调用也可以
        //bt_gatt_exchange_mtu(conn, NULL);
        }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
        // 打印连接断开信息，包括断开原因代码和原因描述
        printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));

        // 使用原子操作安全地设置 state 变量的 STATE_DISCONNECTED 位（第二位），设置值为1
        (void)atomic_set_bit(state, STATE_DISCONNECTED);
}

//注册回调函数
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static void imu_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
        // 检查写入的值是启用通知 (0x0001) 还是禁用 (0x0000)
        imu_ntf_enabled = (value == BT_GATT_CCC_NOTIFY);

        printk("IMU notification status changed: %s\n", imu_ntf_enabled ? "enabled" : "disabled");
}

// 从 k_fifo 中取出所有样本，打包、发送
void consumer_work_handler(struct k_work *work)
{
        struct imu_sample *sample_ptr;
        struct imu_batch_packet_t packet;
        packet.count = 0;

        printf("\n[Consumer] === 开始批量传输 ===\n");

        // 从队列中取出所有积攒的样本
        while ((sample_ptr = k_fifo_get(&imu_data_fifo, K_NO_WAIT)) != NULL && packet.count < BATCH_SIZE)
        {
                int i = packet.count;
                packet.samples[i].accel[0] = (float)sensor_value_to_double(&sample_ptr->accel[0]);
                packet.samples[i].accel[1] = (float)sensor_value_to_double(&sample_ptr->accel[1]);
                packet.samples[i].accel[2] = (float)sensor_value_to_double(&sample_ptr->accel[2]);
        
                packet.samples[i].gyro[0]  = (float)sensor_value_to_double(&sample_ptr->gyro[0]);
                packet.samples[i].gyro[1]  = (float)sensor_value_to_double(&sample_ptr->gyro[1]);
                packet.samples[i].gyro[2]  = (float)sensor_value_to_double(&sample_ptr->gyro[2]);
                packet.count++;
                k_free(sample_ptr);
                printk("free batch of %d samples\n", packet.count);
        }

        if (imu_ntf_enabled && packet.count > 0) {
                size_t send_len = 1 + packet.count * 24;
                int err = bt_gatt_notify(NULL, &imu_svc.attrs[2], &packet, send_len);
                if (err) {
                printk("Batch notify failed: %d\n", err);
                } else {
                printk("Sent batch of %d samples\n", packet.count);
                }
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
        if (sensor_sample_fetch(dev) < 0)
        {
                printk("[Producer] 传感器数据获取失败\n");
                return;
        }

        // 分配内存给新的样本
        new_sample = k_malloc(sizeof(struct imu_sample));
        if (new_sample == NULL)
        {
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

#if defined(CONFIG_GPIO) // 检查kconfig中是否启用GPIO功能
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0) // 获取 "led0" 别名对应的设备树节点

#if DT_NODE_HAS_STATUS_OKAY(LED0_NODE)                                     // 判断Led0这个设备是否可用
#include <zephyr/drivers/gpio.h>                                           // 包含 GPIO 驱动头文件
#define HAS_LED 1                                                          // 定义一个宏，表示存在可用的 LED
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios); // 获取 led0 的具体硬件信息
#define BLINK_ONOFF K_MSEC(500)                                            // 定义闪烁间隔为 500 毫秒

static struct k_work_delayable blink_work; // 定义一个延时工作项变量
static bool led_is_on;                     // 追踪 LED 状态

static void blink_timeout(struct k_work *work) // 实际执行闪烁逻辑的回调函数 不断调度自己
{
        led_is_on = !led_is_on;
        gpio_pin_set(led.port, led.pin, (int)led_is_on); // 切换电平

        k_work_schedule(&blink_work, BLINK_ONOFF); // 请在 500 毫秒之后，把与 blink_work 相关联的那个函数 (blink_timeout) 交给一个后台工作线程去执行。
}

static int blink_setup(void)
{
        int err;

        // 检查设备是否就绪
        printk("Checking LED device...");
        if (!gpio_is_ready_dt(&led))
        {
                printk("failed.\n");
                return -EIO; // 打印I/O错误码
        }
        printk("done.\n");

        // 设置引脚
        printk("Configuring GPIO pin...");
        err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE); // 把led变量代表的引脚设为输出模式
        if (err)
        {
                printk("failed.\n");
                return -EIO;
        }
        printk("done.\n");

        k_work_init_delayable(&blink_work, blink_timeout); // 把待办事项和具体内容关联起来

        return 0;
}

static void blink_start(void)
{
        printk("Start blinking LED...\n");
        led_is_on = false;
        gpio_pin_set(led.port, led.pin, (int)led_is_on); // 初始电平设置为低，灯不亮
        k_work_schedule(&blink_work, BLINK_ONOFF);       // 请在 500 毫秒之后，把与 blink_work 相关联的那个函数 (blink_timeout) 交给一个后台工作线程去执行。
}

static void blink_stop(void)
{
        struct k_work_sync work_sync;

        printk("Stop blinking LED.\n");
        k_work_cancel_delayable_sync(&blink_work, &work_sync); // 取消尚未执行的调度

        /* Keep LED on */
        led_is_on = true;
        gpio_pin_set(led.port, led.pin, (int)led_is_on); // 停止后，保持 LED 为常亮状态
}
#endif /* LED0_NODE */
#endif /* CONFIG_GPIO */

int main(void)
{
        int err;
        err = bt_enable(NULL);
        if (err)
        {
                printk("Bluetooth init failed (err %d)\n", err); // 如果初始化失败打印错误信息
                return 0;                                        // 蓝牙无法启动，程序退出
        }

        printk("Bluetooth initialized\n");

        err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
        if (err)
        {
                printk("Advertising failed to start (err %d)\n", err);
                return 0;
        }
        printk("Advertising successfully started\n");

#if defined(HAS_LED) // 预处理器检查：仅当通过设备树找到了可用的 LED ("led0") 并且 GPIO 功能已启用时，才执行此代码块。（上文中两个#IF）

        // 调用 blink_setup() 函数，初始化 LED 相关的 GPIO 引脚，并设置好延时工作项
        err = blink_setup();
        if (err)
        {
                return 0;
        }

        // 调用 blink_start() 函数，开始使用工作队列进行非阻塞的 LED 闪烁，表示设备正在等待连接
        blink_start();

#endif /* HAS_LED */

        if (!device_is_ready(sensor))
        {
                printf("[Main] 错误: IMU传感器未就绪\n");
                return 0;
        }

        printf("[Main] IMU传感器初始化成功\n");

        struct sensor_value odr_attr;

        /* 设置加速度计和陀螺仪的采样频率为 104 Hz */
        odr_attr.val1 = SAMPLE_FREQUENCY; // 104 Hz
        odr_attr.val2 = 0;                // 0 小数部分

        // 设置加速度计的采样频率
        if (sensor_attr_set(sensor, SENSOR_CHAN_ACCEL_XYZ,
                            SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0)
        {
                printf("[Main] 无法设置加速度计的采样频率\n");
                return 0;
        }
        printf("[Main] 加速度计采样频率设置为 104 Hz\n");

        // 设置陀螺仪的采样频率
        if (sensor_attr_set(sensor, SENSOR_CHAN_GYRO_XYZ,
                            SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0)
        {
                printf("[Main] 无法设置陀螺仪的采样频率\n");
                return 0;
        }
        printf("[Main] 陀螺仪采样频率设置为 104 Hz\n");

        /* --- 配置和启用传感器触发器 (生产者) --- */
        struct sensor_trigger trig = {
            .type = SENSOR_TRIG_DATA_READY, // 数据就绪触发
            .chan = SENSOR_CHAN_ALL,
        };

        // 绑定触发器和回调函数
        if (sensor_trigger_set(sensor, &trig, sensor_trigger_handler) < 0)
        {
                printf("[Main] 无法设置传感器触发器\n");
                return 0;
        }
        printf("[Main] 传感器线程已启动\n");

        // 启动延迟工作项
        // 立即启动第一次运行，之后它会在 consumer_work_handler 中自己每 50 ms 重新调度
        k_work_reschedule(&producer_work, K_MSEC(CONSUMER_RATE_MS));
        printf("[Main] 延迟工作项 (消费者, %d ms) 已启动。\n", CONSUMER_RATE_MS);

        while (1)
        {
                k_sleep(K_MSEC(500)); // 暂停 500 毫秒，给其他任务运行时间

                if (atomic_test_and_clear_bit(state, STATE_CONNECTED))
                {

                        /* Connected callback executed */

                        blink_stop(); // 停止闪灯，改为常亮
                }
                else if (atomic_test_and_clear_bit(state, STATE_DISCONNECTED))
                { // 每秒轮询检查连接是否断开	// 重新启动传统广播
                        printk("Starting Legacy Advertising (connectable and scannable)\n");
                        err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd,
                                              ARRAY_SIZE(sd));
                        if (err)
                        {
                                printk("Advertising failed to start (err %d)\n", err);
                                return 0;
                        }

                        blink_start();
                }
        }
        return 0;
}