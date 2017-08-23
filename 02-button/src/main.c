/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <board.h>
#include <device.h>
#include <gpio.h>
#include <misc/util.h>
#include <misc/printk.h>

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

void button_0_pressed(struct device *gpioe, struct gpio_callback *cb,
					  u32_t pins)
{
	int val = 0;
	// k_sleep(SLEEP_TIME);
	if (pins & BIT(SW0_PIN))
	{
		gpio_pin_read(gpioa, LED1_PIN, &val);
		gpio_pin_write(gpioa, LED1_PIN, !val);
	}
	else if (pins & BIT(SW1_PIN))
	{
		gpio_pin_read(gpioa, LED2_PIN, &val);
		gpio_pin_write(gpioa, LED2_PIN, !val);
	}else if (pins & BIT(KEY_UP_PIN)){
		gpio_pin_read(gpioa, LED1_PIN, &val);
		gpio_pin_write(gpioa, LED1_PIN, !val);
		gpio_pin_write(gpioa, LED2_PIN, val);
	}
}

// void button_1_pressed(struct device *gpioe, struct gpio_callback *cb,
// 					  u32_t pins)
// {
// 	gpio_pin_write(gpioa, LED2_PIN, val);
// 	gpio_pin_write(gpioa, LED1_PIN, !val);
// 	val = !val;
// }

void main(void)
{
	static struct gpio_callback gpio_btn0_cb;
	// static struct gpio_callback gpio_btn1_cb;

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
	gpio_pin_configure(gpioe, KEY_UP_PIN,
					   GPIO_DIR_IN | GPIO_INT | GPIO_PUD_PULL_DOWN | EDGE);

	gpio_init_callback(&gpio_btn0_cb, button_0_pressed, BIT(SW0_PIN) | BIT(SW1_PIN) | BIT(KEY_UP_PIN) );

	gpio_add_callback(gpioe, &gpio_btn0_cb);
	gpio_pin_enable_callback(gpioe, SW0_PIN);
	gpio_pin_enable_callback(gpioe, SW1_PIN);
	gpio_pin_enable_callback(gpioe, KEY_UP_PIN);

	//Init LED
	gpio_pin_configure(gpioa, LED1_PIN, GPIO_DIR_OUT);
	gpio_pin_configure(gpioa, LED2_PIN, GPIO_DIR_OUT);
	gpio_pin_write(gpioa, LED1_PIN, 0);
	gpio_pin_write(gpioa, LED2_PIN, 0);
}
