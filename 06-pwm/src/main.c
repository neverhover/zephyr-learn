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
#include <dt-bindings/pinctrl/stm32-pinctrl.h>

#include <pinmux.h>
#include <sys_io.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_tim.h>
#include <stm32f4xx_ll_utils.h>


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
int count=0;
int dir=1;

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
	__IO u32_t RCR;
	__IO u32_t CCR1;
	__IO u32_t CCR2;
	__IO u32_t CCR3;
	__IO u32_t CCR4;
	__IO u32_t BDTR;
	__IO u32_t DCR;
	__IO u32_t DMAR;
};

struct my_GPIO{
	u32_t MODER;
};

#define AS_TIMER(__base_addr) \
    (struct my_timer *)(__base_addr)

struct my_timer *tim3 = AS_TIMER(TIM3_BASE);
struct my_timer *tim14 = AS_TIMER(TIM14_BASE);
struct my_GPIO *g7 = (struct my_GPIO *) GPIOA_BASE;

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
	// gpio_pin_configure(gpioa, LED2_PIN, GPIO_DIR_OUT);
	gpio_pin_write(gpioa, LED1_PIN, 1);
	// gpio_pin_write(gpioa, LED2_PIN, 0);

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

void init_timer14(){
	//用于输出PWM
    //先使能timer始终
    struct device *clk = device_get_binding(STM32_CLOCK_CONTROL_NAME);
    struct stm32_pclken pclken;
    pclken.bus = STM32_CLOCK_BUS_APB1,
    pclken.enr = LL_APB1_GRP1_PERIPH_TIM14;

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

	//pinmux GPIOA 7
#if 0
	static const struct pin_config pinconf[] = {
		{STM32_PIN_PA7, STM32F4_PINMUX_FUNC_PA7_ETH}
	}
	stm32_setup_pins(pinconf, ARRAY_SIZE(pinconf));
#endif
	//psc=8400-1
#if 0
	//需要LED7上复用，funciton为AF7,使能TIM14
	//修改方法为drivers
    tim14->ARR = 500-1;
    tim14->PSC = 84-1;
    tim14->DIER |= 1<<0;
    //计数频率为84M/84 = 1Mhz
    //PWM频率为1M/500= 2Khz

	tim14->CCMR1 |=( 6<<4 | 1<< 3);
	tim14->CCER |= (1<<0 | 1 << 1);
	
	//使能ARPE
	tim14->CR1 |= 1<<7;
	//使能TIM14
	tim14->CR1 |= 1<<0;
#else
	LL_GPIO_InitTypeDef GPIO_InitStruct;
	LL_GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_9;
	LL_GPIO_Init((GPIO_TypeDef *) GPIOA_BASE, &GPIO_InitStruct);

	TIM_TypeDef *tim14 = (TIM_TypeDef *)TIM14_BASE;

	LL_TIM_InitTypeDef TIM_InitStruct;
	LL_TIM_StructInit(&TIM_InitStruct);
	TIM_InitStruct.Prescaler = 84-1;
	TIM_InitStruct.Autoreload = 500-1;
	LL_TIM_Init(tim14, &TIM_InitStruct);

	LL_TIM_OC_InitTypeDef TIM14_struct;
	TIM14_struct.OCMode = LL_TIM_OCMODE_PWM1;
	TIM14_struct.OCState = LL_TIM_OCSTATE_ENABLE;
	TIM14_struct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
	LL_TIM_OC_Init(tim14, LL_TIM_CHANNEL_CH1, &TIM14_struct);
	
	tim14->CR1 |= 1<<0;
#endif
}

void main(void)
{


	print_device_info();
	init_led();
	init_timer14();
	printk("GPIO A7 : %u\n", g7->MODER);
#if 1
	while(1){
		if(dir == 1){
			// printk("Plus tim14 CCR1\n");
			tim14->CCR1 = tim14->CCR1 + 1;
			count +=1;
		}else{
			tim14->CCR1 = tim14->CCR1 -1;
			// printk("Div tim14 CCR1\n");
			count -=1;
		}
		if(count> 250)
			dir = 0;
		else if(count == 0)
			dir = 1;
		
		k_sleep(2);
	}
#endif
}
