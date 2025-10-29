/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
// 定义一个名为 SLEEP_TIME_MS 的宏常量，其值为 1000。注释说明了这是 1000 毫秒，即 1 秒。这个常量用于控制 LED 闪烁的频率（亮多久，灭多久）。

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
// 定义一个名为 LED0_NODE 的宏。DT_ALIAS(led0) 是 Zephyr 提供的一个特殊宏，用于从“设备树 (Devicetree)”中查找名为 "led0" 的别名 (alias)。设备树是一种描述硬件连接方式的文件（.dts, .overlay）。开发板的设备树文件会定义好哪个物理 LED 被赋予了 "led0" 这个别名。这个宏使得代码可以用统一的逻辑名称 ("led0") 来引用硬件，而无需硬编码具体的引脚号，提高了代码的可移植性。

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
// 这一行是获取 LED 硬件信息的关键步骤。
// static const struct gpio_dt_spec led: 声明一个名为 led 的变量。
//   - static: 表示这个变量只在当前文件内可见。
//   - const: 表示这个变量的值在初始化后不能被修改。
//   - struct gpio_dt_spec: 这是 Zephyr 定义的一个结构体类型，专门用来存储从设备树获取到的 GPIO 引脚的完整信息（包括它属于哪个 GPIO 控制器、具体的引脚编号、引脚的特性标志等）。
// GPIO_DT_SPEC_GET(LED0_NODE, gpios): 这是另一个 Zephyr 提供的宏，用于根据设备树节点信息来初始化 gpio_dt_spec 结构体。
//   - LED0_NODE: 我们之前定义的宏，指向设备树中 "led0" 别名对应的节点。
//   - gpios: 指定我们要获取的是该节点下的 "gpios" 属性（这个属性包含了引脚号、标志等信息）。
// 整个语句的作用就是：根据设备树中 "led0" 的定义，创建一个名为 led 的、包含了控制该 LED 所需全部硬件信息的结构体变量。
// 注释警告：如果你的开发板的设备树文件里没有定义 "led0" 这个别名，或者定义不正确，编译时就会在这行报错。
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);



int main(void)
{
	int ret; // 定义一个整型变量 ret (return code)，用于存储 Zephyr API 函数的返回值。按照惯例，返回 0 通常表示成功，返回负数表示发生了错误。
	bool led_state = true; // 定义一个布尔型变量 led_state，用于在逻辑上跟踪 LED 的状态（亮或灭）。这里初始化为 true，假设初始状态为亮。

	// 调用 gpio_is_ready_dt 函数，检查与 led 变量关联的 GPIO 设备是否已经初始化并且可以使用。
    // 需要传入 led 变量的地址 (&led)
	if (!gpio_is_ready_dt(&led)) {
		return 0;	//如果没准备好，返回0并退出
	}

	// 调用 gpio_pin_configure_dt 函数，将 led 变量所代表的 GPIO 引脚配置为输出模式。
    // GPIO_OUTPUT_ACTIVE 是一个标志，表示配置为输出，并且当引脚设置为“活动”(Active) 电平时 LED 会亮（具体是高电平还是低电平取决于设备树中的配置）。
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {	// 检查返回值。如果配置失败，函数会返回一个负数错误码。
		return 0;	// 如果配置失败，也退出程序。
	}

	// 进入一个无限循环。嵌入式程序的核心逻辑通常在一个不会退出的循环中运行，以持续执行任务。
	while (1) {
		// 调用 gpio_pin_toggle_dt 函数，切换 led 引脚的当前电平状态。
        // 如果引脚当前是高电平，则变为低电平；如果是低电平，则变为高电平。从而实现 LED 的亮灭切换。
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) { //检查切换是否成功
			return 0;
		}

		// 更新逻辑状态变量的值（取反）。
        // 调用 printf 函数，将 LED 的当前逻辑状态 ("ON" 或 "OFF") 打印到控制台。
        // led_state ? "ON" : "OFF" 是一个三元运算符，如果 led_state 为 true，则输出 "ON"，否则输出 "OFF"。
        // \n 表示换行。
		led_state = !led_state;
		printf("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);
	}
	// 因为上面的 while(1) 是一个无限循环，所以程序理论上永远不会执行到这一行。
	return 0;
}

