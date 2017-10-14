/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Author: LeyiZhang@ligowave.com
 */

/**
 * @brief Driver for Independent Watchdog (IWDG) for STM32 MCUs
 *
 * Based on reference manual:
 *   STM32F4xx
 *   advanced ARM ® -based 32-bit MCUs
 *
 * Chapter 19: Window watchdog (WWDG)
 *
 */

#include <watchdog.h>
#include <soc.h>
#include <errno.h>
#include <zephyr.h>
#include <device.h>
#include <misc/printk.h>
#include <soc.h>
#include <clock_control.h>
#include <drivers/clock_control/stm32_clock_control.h>
#include "wwdg_stm32.h"

#define AS_WWDG(__base_addr) \
    (struct wwdg_stm32 *)(__base_addr)

static void (*user_cb)(struct device *dev);
static u32_t timeout = 0x40;
static int wwdg_stm32_init(struct device *dev);

static void wwdg_stm32_enable(struct device *dev)
{
    struct wwdg_stm32 *wwdg = AS_WWDG(WWDG_BASE);
    printk(">>>> WWDG STM32 before enabled, wwdg->cr=%d, wwdg->cfr=%d, wwdg->sr=%d\n",
    wwdg->cr.val, wwdg->cfr.val, wwdg->sr.val);
    ARG_UNUSED(dev);
    /* 设置CR的计数器 */
    wwdg->cr.bit.t = 0x7f;
    /* 设置CRF的窗口值 */
    wwdg->cfr.bit.w = timeout;
    wwdg->cfr.bit.wdgtb = 3;
    /* 只能由硬件复位，置1表示使能 */
    //wwdg->cr.bit.wdga = 1;
    wwdg->sr.bit.ewif = 0;
    if (user_cb){
	printk("User defined a callback\n");
        wwdg->cfr.bit.ewi = 1;
	//user_cb(dev);
	wwdg->cr.bit.wdga = 1;
    }
    //wwdg->cr.val = 0x7f | 1 << 7;
    //wwdg->cfr.val = 0x5f | 1<<7 | 1<< 8 | 1<<9;
    printk(">>>>> Enable WWDG STM32\n");
    printk(">>>>> WWDG STM32 enabled, wwdg->cr=%d, wwdg->cfr=%d, wwdg->sr=%d\n",
    wwdg->cr.val, wwdg->cfr.val, wwdg->sr.val);
}

static void wwdg_stm32_disable(struct device *dev)
{
    /* watchdog cannot be stopped once started */
    ARG_UNUSED(dev);
}

static void wwdg_stm32_reload(struct device *dev)
{
    volatile struct wwdg_stm32 *wwdg = AS_WWDG(WWDG_BASE);
    wwdg->cr.bit.t = 0x7f;
    wwdg->sr.bit.ewif = 0;
    //printk(">>>>> WWDG STM32 reload start, wwdg->cr=%d, wwdg->cfr=%d, wwdg->sr=%d\n",
    //    wwdg->cr.val, wwdg->cfr.val, wwdg->sr.val);
    ARG_UNUSED(dev);
    wwdg->sr.bit.ewif = 0;
    wwdg->cr.bit.t = 0x7f;
    //printk(">>>>> WWDG STM32 reload done, wwdg->cr=%d, wwdg->cfr=%d, wwdg->sr=%d\n",
    //    wwdg->cr.val, wwdg->cfr.val, wwdg->sr.val);   
}

static int wwdg_stm32_set_config(struct device *dev,
    struct wdt_config *config)
{
    ARG_UNUSED(dev);
    user_cb = config->interrupt_fn;
    timeout = config->timeout;
    
    return -ENOTSUP;
}

static void wwdg_stm32_get_config(struct device *dev,
    struct wdt_config *config)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(config);
}

static const struct wdt_driver_api wwdg_stm32_api = {
	.enable = wwdg_stm32_enable,
	.disable = wwdg_stm32_disable,
	.get_config = wwdg_stm32_get_config,
	.set_config = wwdg_stm32_set_config,
	.reload = wwdg_stm32_reload,
};

static void wwdg_stm32_isr(void *arg){
	struct device *dev = arg;
	// struct uart_stm32_data *data = DEV_DATA(dev);

	if (user_cb) {
		user_cb(dev);
	}
}

DEVICE_AND_API_INIT(wwdg_stm32, "WWDG_STM32", wwdg_stm32_init,
    NULL, NULL,
    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
    &wwdg_stm32_api);

static int wwdg_stm32_init(struct device *dev)
{
    printk("Init WWDG on stm32f4xx\n");
    // 使能时钟
    struct device *clk = device_get_binding(STM32_CLOCK_CONTROL_NAME);
#if 0
	struct stm32_pclken pclken = {
		.bus = STM32_CLOCK_BUS_APB1,
		.enr = LL_APB1_GRP1_PERIPH_WWDG;
	};
#endif
    struct stm32_pclken pclken;
    pclken.bus = STM32_CLOCK_BUS_APB1;
    pclken.enr = LL_APB1_GRP1_PERIPH_WWDG;

	/* Enable SYSCFG clock */
    clock_control_on(clk, (clock_control_subsys_t *) &pclken);
    // // 注册ISR
     IRQ_CONNECT(WWDG_IRQn, 0,
         wwdg_stm32_isr, DEVICE_GET(wwdg_stm32), 0);
     irq_enable(WWDG_IRQn);
    ARG_UNUSED(dev);
    return 0;

}
