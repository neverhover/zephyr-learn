/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <gpio.h>
#include <device.h>
#include <watchdog.h>
#include "board.h"
#include <misc/printk.h>
#include <soc.h>
#include <clock_control.h>
#include <drivers/clock_control/stm32_clock_control.h>

#define SW0_PORT "GPIOE"
#define SW0_PIN 4
#define SW1_PORT "GPIOE"
#define SW1_PIN 3
#define KEY_UP_PORT "GPIOA"
#define KEY_UP_PIN 0

#define EDGE (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW)

#define PORT "GPIOA"
/* PB5 */
#define LED1_PIN 6
/* PB8 */
#define LED2_PIN 7

#define SLEEP_TIME 300

struct device *gpioe;
struct device *gpioa;

//used for tim2 -- tim5
struct my_timer{
	__IO u32_t CR1;
	__IO u32_t CR2;
	__IO u32_t SMCR;
	__IO u32_t DIER;
	__IO u32_t SR;
	__IO u32_t EGR;
	__IO u32_t CCMR1;
	__IO u32_t CCMR2;
	__IO u32_t CCER;
	__IO u32_t CNT;
	__IO u32_t PSC;
	__IO u32_t ARR;
	//any else reg
};

#define AS_TIMER(__base_addr) \
    (struct my_timer *)(__base_addr)

struct my_timer *tim3 = AS_TIMER(TIM3_BASE);

void button_0_pressed(struct device *gpio, struct gpio_callback *cb,
	u32_t pins);

void button_0_pressed(struct device *gpio, struct gpio_callback *cb,
					  u32_t pins)
{
	int val = 0;
	// k_sleep(SLEEP_TIME);
	if (pins & BIT(SW0_PIN))
	{
		printk("SW0 PUSHED\n");
		gpio_pin_read(gpioa, LED1_PIN, &val);
		gpio_pin_write(gpioa, LED1_PIN, !val);
	}
	else if (pins & BIT(SW1_PIN))
	{
		printk("SW1 PUSHED\n");
		gpio_pin_read(gpioa, LED2_PIN, &val);
		gpio_pin_write(gpioa, LED2_PIN, !val);
	}else if (pins & BIT(KEY_UP_PIN)){
		printk("KEY_UP pushed and reload WWDG\n");
		gpio_pin_read(gpioa, LED1_PIN, &val);
		gpio_pin_write(gpioa, LED1_PIN, !val);
		gpio_pin_write(gpioa, LED2_PIN, val);
	}
}

void init_led(){
	static struct gpio_callback gpio_btn0_cb;
	gpioe = device_get_binding(SW0_PORT);
	if (!gpioe)
	{
		printk("error\n");
		return;
	}
	gpioa = device_get_binding(PORT);

	gpio_pin_configure(gpioe, SW0_PIN,
					   GPIO_DIR_IN | GPIO_INT | GPIO_PUD_PULL_UP | EDGE);
	gpio_pin_configure(gpioe, SW1_PIN,
					   GPIO_DIR_IN | GPIO_INT | GPIO_PUD_PULL_UP | EDGE);
	gpio_pin_configure(gpioa, KEY_UP_PIN,
					   GPIO_DIR_IN | GPIO_INT | GPIO_PUD_PULL_DOWN | EDGE);

	gpio_init_callback(&gpio_btn0_cb, button_0_pressed, BIT(SW0_PIN) | BIT(SW1_PIN) | BIT(KEY_UP_PIN) );

	gpio_add_callback(gpioe, &gpio_btn0_cb);
	gpio_add_callback(gpioa, &gpio_btn0_cb);
	gpio_pin_enable_callback(gpioe, SW0_PIN);
	gpio_pin_enable_callback(gpioe, SW1_PIN);
	gpio_pin_enable_callback(gpioa, KEY_UP_PIN);

	//Init LED
	gpio_pin_configure(gpioa, LED1_PIN, GPIO_DIR_OUT);
	gpio_pin_configure(gpioa, LED2_PIN, GPIO_DIR_OUT);
	gpio_pin_write(gpioa, LED1_PIN, 1);
	gpio_pin_write(gpioa, LED2_PIN, 0);

}

void print_device_info(){
	uint32_t uid_low = LL_GetUID_Word0();
	uint32_t uid_medium = LL_GetUID_Word1();
	uint32_t uid_high = LL_GetUID_Word2();
	uint16_t flash_size = LL_GetFlashSize();

	printk("======= Device UID : %d-%d-%d\n", uid_high, uid_medium, uid_low);
	printk("======= Device Flash Size : %d KB\n", flash_size);
	printk("======= Kernel uptime : %d ms\n", k_uptime_get_32());
	printk("======= Kernel verion : %d \n", sys_kernel_version_get());
}


void timer_irq_handler(void *unused){
	ARG_UNUSED(unused);
	int val=0;
	if(tim3->SR & 0x0001){
	  	gpio_pin_read(gpioa, LED1_PIN, &val);
                gpio_pin_write(gpioa, LED1_PIN, !val);
                gpio_pin_write(gpioa, LED2_PIN, val);
	}
	tim3->SR &= ~(1<<0);
}

void init_timer(){

	//Enable SYS CLK
	struct device *clk = device_get_binding(STM32_CLOCK_CONTROL_NAME);
	struct stm32_pclken pclken;
	pclken.bus = STM32_CLOCK_BUS_APB1,
	pclken.enr = LL_APB1_GRP1_PERIPH_TIM3;
	


	/* Enable SYSCFG clock */
   	 clock_control_on(clk, (clock_control_subsys_t *) &pclken);

        if (!clk)
        {
                printk("Cant get system clock\n");
                return;
        }else{
                u32_t rate=0;
                clock_control_get_rate( clk, (clock_control_subsys_t *) &pclken, &rate);
		printk("======= %s APB1 Clock :%d\n", STM32_CLOCK_CONTROL_NAME, rate);
        }

	//Enable IRQ
        IRQ_CONNECT(TIM3_IRQn, 0, timer_irq_handler, NULL, 0);
        irq_enable(TIM3_IRQn);
   

	//Init presclar
	//arr=1000-1
	//psc=8400-1
	tim3->ARR = 4000-1;
	tim3->PSC = 8400-1;
	tim3->DIER |= 1<<0;
	tim3->CR1 |= 0X01;
}


void main(void)
{


	print_device_info();
	init_led();
	init_timer();
}
