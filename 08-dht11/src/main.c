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
#include <stm32f4xx_ll_rtc.h>
#include <stm32f4xx_ll_rcc.h>
#include <stm32f4xx_ll_pwr.h>

/* SESOR header */
#include <sensor.h>

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

#define AS_TIMER(__base_addr) \
    (struct my_timer *)(__base_addr)

void button_0_pressed(struct device *gpio, struct gpio_callback *cb,
	u32_t pins);

void rtc_get_time_now();

void button_0_pressed(struct device *gpio, struct gpio_callback *cb,
					  u32_t pins)
{
	int val = 0;
	u8_t tmp_val = 0;
	u8_t status = 0;

	RTC_TypeDef *rtc = (RTC_TypeDef *)RTC_BASE;

	LL_RTC_TimeTypeDef RTC_TimeStruct;
	LL_RTC_TIME_StructInit(&RTC_TimeStruct);
	
	
	if (pins & BIT(SW0_PIN))
	{
		printk("SW0 PUSHED and hour updated\n");
		gpio_pin_read(gpioa, LED1_PIN, &val);
		gpio_pin_write(gpioa, LED1_PIN, !val);
		//修改小时
		tmp_val = LL_RTC_TIME_GetHour(rtc);
		if(tmp_val == 23){
			tmp_val = 0;
		}else{
			tmp_val +=1;
		}
		RTC_TimeStruct.Hours      = tmp_val;
		RTC_TimeStruct.Minutes      = LL_RTC_TIME_GetMinute(rtc);
		RTC_TimeStruct.Seconds      = LL_RTC_TIME_GetSecond(rtc);
		
		status = LL_RTC_TIME_Init(rtc, LL_RTC_FORMAT_BCD, &RTC_TimeStruct);
		printk("======= RTC Time init ...%u\n", status);
		WRITE_REG(rtc->BKP0R, 0x6666U);
		rtc_get_time_now();

	}
	else if (pins & BIT(SW1_PIN))
	{
		printk("SW1 PUSHED and minute update\n");
		gpio_pin_read(gpioa, LED2_PIN, &val);
		gpio_pin_write(gpioa, LED2_PIN, !val);

		//修改分钟
		tmp_val = LL_RTC_TIME_GetMinute(rtc);
		if(tmp_val == 59){
			tmp_val = 0;
		}else{
			tmp_val +=1;
		}
		RTC_TimeStruct.Minutes      = tmp_val;
		RTC_TimeStruct.Hours      = LL_RTC_TIME_GetHour(rtc);
		RTC_TimeStruct.Seconds      = LL_RTC_TIME_GetSecond(rtc);
		
		status = LL_RTC_TIME_Init(rtc, LL_RTC_FORMAT_BCD, &RTC_TimeStruct);
		printk("======= RTC Time init ...%u\n", status);
		WRITE_REG(rtc->BKP0R, 0x6666U);
		rtc_get_time_now();

	}else if (pins & BIT(KEY_UP_PIN)){
		printk("KEY_UP pushed and read time now\n");
		gpio_pin_read(gpioa, LED1_PIN, &val);
		gpio_pin_write(gpioa, LED1_PIN, !val);
		gpio_pin_write(gpioa, LED2_PIN, val);
		rtc_get_time_now();
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

void init_timer14(){
	//用于输出PWM
    //先使能timer始终
    struct device *clk = device_get_binding(STM32_CLOCK_CONTROL_NAME);
    struct stm32_pclken pclken;
    pclken.bus = STM32_CLOCK_BUS_APB1,
    pclken.enr = LL_APB1_GRP1_PERIPH_TIM14;

    /* Enable SYSCFG clock */
    clock_control_on(clk, (clock_control_subsys_t *) &pclken);

    if (!clk){
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


ErrorStatus LL_RTC_GetTime(RTC_TypeDef *RTCx, uint32_t RTC_Format, LL_RTC_TimeTypeDef *RTC_TimeStruct)
{

  /* Check the parameters */
  assert_param(IS_RTC_ALL_INSTANCE(RTCx));
  assert_param(IS_LL_RTC_FORMAT(RTC_Format));
  
  if (LL_RTC_IsShadowRegBypassEnabled(RTCx) == 0U){
	while(LL_RTC_IsActiveFlag_RS(RTCx) == ERROR){}
  }
  
  /* Fill the structure fields with the read parameters */
  RTC_TimeStruct->Hours = (uint8_t)(LL_RTC_TIME_GetHour(RTCx));
  RTC_TimeStruct->Minutes = (uint8_t)(LL_RTC_TIME_GetMinute(RTCx));
  RTC_TimeStruct->Seconds = (uint8_t)(LL_RTC_TIME_GetSecond(RTCx));
  
  /* Check the input parameters format */
  if(RTC_Format == LL_RTC_FORMAT_BIN)
  {
    /* Convert the time structure parameters to Binary format */
    RTC_TimeStruct->Hours = (uint8_t)__LL_RTC_CONVERT_BCD2BIN(RTC_TimeStruct->Hours);
    RTC_TimeStruct->Minutes = (uint8_t)__LL_RTC_CONVERT_BCD2BIN(RTC_TimeStruct->Minutes);
    RTC_TimeStruct->Seconds = (uint8_t)__LL_RTC_CONVERT_BCD2BIN(RTC_TimeStruct->Seconds);  
  }
  
  return SUCCESS;
}

ErrorStatus LL_RTC_GetDate(RTC_TypeDef *RTCx, uint32_t RTC_Format, LL_RTC_DateTypeDef *RTC_DateStruct)
{

  /* Check the parameters */
  assert_param(IS_RTC_ALL_INSTANCE(RTCx));
  assert_param(IS_LL_RTC_FORMAT(RTC_Format));
  
  if (LL_RTC_IsShadowRegBypassEnabled(RTCx) == 0U){
	while(LL_RTC_IsActiveFlag_RS(RTCx) == ERROR){}
  }

  /* Fill the structure fields with the read parameters */
  RTC_DateStruct->Year = (uint8_t)(LL_RTC_DATE_GetYear(RTCx));
  RTC_DateStruct->Month = (uint8_t)(LL_RTC_DATE_GetMonth(RTCx));
  RTC_DateStruct->Day = (uint8_t)(LL_RTC_DATE_GetDay(RTCx));
  RTC_DateStruct->WeekDay = (uint8_t)(LL_RTC_DATE_GetWeekDay(RTCx)); 

  /* Check the input parameters format */
  if(RTC_Format == LL_RTC_FORMAT_BIN)
  {    
    /* Convert the date structure parameters to Binary format */
    RTC_DateStruct->Year = __LL_RTC_CONVERT_BCD2BIN(RTC_DateStruct->Year);
    RTC_DateStruct->Month = __LL_RTC_CONVERT_BCD2BIN(RTC_DateStruct->Month);
    RTC_DateStruct->Day = __LL_RTC_CONVERT_BCD2BIN(RTC_DateStruct->Day);  
  }
  return SUCCESS;
}

void rtc_get_time_now(){
	//RTC指针
	RTC_TypeDef *rtc = (RTC_TypeDef *)RTC_BASE;
	
	LL_RTC_DateTypeDef RTC_DateStruct;
	LL_RTC_TimeTypeDef RTC_TimeStruct;
	LL_RTC_GetDate(rtc, LL_RTC_FORMAT_BIN, &RTC_DateStruct);
	LL_RTC_GetTime(rtc, LL_RTC_FORMAT_BIN, &RTC_TimeStruct);
	printk("======= Get RTC time: %u-%u-%u(weekday:%u) %02u:%02u:%02u\n",
					RTC_DateStruct.Year+2000, RTC_DateStruct.Month, RTC_DateStruct.Day, RTC_DateStruct.WeekDay, 
					RTC_TimeStruct.Hours, RTC_TimeStruct.Minutes, RTC_TimeStruct.Seconds);
}

void init_rtc(){
	printk("======= Init RTC\n");
	//时钟使能
	/* Enable SYSCFG clock */
	struct device *clk = device_get_binding(STM32_CLOCK_CONTROL_NAME);
    struct stm32_pclken pclken;
    pclken.bus = STM32_CLOCK_BUS_APB1,
	pclken.enr = LL_APB1_GRP1_PERIPH_PWR;
	clock_control_on(clk, (clock_control_subsys_t *) &pclken);
	LL_PWR_EnableBkUpAccess();
	printk("=======    Init APB1 PWR ...OK\n");
	LL_RCC_LSE_Enable();
	LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
    LL_RCC_EnableRTC();
	
	//RTC指针
	RTC_TypeDef *rtc = (RTC_TypeDef *)RTC_BASE;
	u32_t bkp0r = READ_REG(rtc->BKP0R);
	if(bkp0r == 0x6666U){
		//已经初始化过，不再初始化
		printk("=======     RTC had already initialized\n");
		rtc_get_time_now();
		return;
	}else{
		printk("=======     RTC has not initialized before\n");
	}
	//开启LSE时钟，32.768KHZ
	u16_t retry = 0x1FFF;
	u16_t status = ERROR;
	// LL_RCC_LSE_Enable();
	//LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
	//LL_RCC_EnableRTC();
	while(retry && LL_RCC_LSE_IsReady() == 0){
		retry--;
		k_sleep(5);
	}
	if(retry == 0){
		printk("=======    RCC LSE initialized ...Error (spent %u ms)\n", 5 * (0x1FFF));
		return ;
	}else{
		printk("=======    RCC LSE initialized ...OK (spent %u ms)\n", 5*(0x1FFF - retry));
	}
	
	//初始化结构
	LL_RTC_InitTypeDef RTC_InitStruct;
	LL_RTC_StructInit(&RTC_InitStruct);
	LL_RTC_Init(rtc, &RTC_InitStruct);
	
	//初始化日期
	LL_RTC_DateTypeDef RTC_DateStruct;
	LL_RTC_DATE_StructInit(&RTC_DateStruct);
	RTC_DateStruct.Year = 17;
	RTC_DateStruct.Month = LL_RTC_MONTH_SEPTEMBER;
	RTC_DateStruct.Day = 18;
	RTC_DateStruct.WeekDay = LL_RTC_WEEKDAY_FRIDAY;
	status = LL_RTC_DATE_Init(rtc, LL_RTC_FORMAT_BIN, &RTC_DateStruct);
	printk("======= RTC Date init ...%s\n", status == SUCCESS ? "OK":"Error");

	//初始化时间
	LL_RTC_TimeTypeDef RTC_TimeStruct;
	LL_RTC_TIME_StructInit(&RTC_TimeStruct);
	RTC_TimeStruct.Hours      = 2;
	RTC_TimeStruct.Minutes    = 55;
	RTC_TimeStruct.Seconds    = 0;
	status = LL_RTC_TIME_Init(rtc, LL_RTC_FORMAT_BIN, &RTC_TimeStruct);
	printk("======= RTC Time init ...%s\n", status == SUCCESS ? "OK":"Error");
	WRITE_REG(rtc->BKP0R, 0x6666U);
	printk("======== Init RTC time: %u-%u-%u(weekday:%u) %02u:%02u:%02u\n",
		RTC_DateStruct.Year+2000, RTC_DateStruct.Month, RTC_DateStruct.Day, RTC_DateStruct.WeekDay, 
		RTC_TimeStruct.Hours, RTC_TimeStruct.Minutes, RTC_TimeStruct.Seconds);

	//测试是否正确
	rtc_get_time_now();
}

void read_dht1x(){
	int ret;
	struct sensor_value temp_value;
	struct device *dev;
	dev = device_get_binding("DHT11");
	const struct sensor_driver_api *api=NULL;
	
	if(!dev){
		printk("Can't get DHT11(DHT12) device\n");
		return;
	}
	api = dev->driver_api;
	while (1) {
		ret = sensor_sample_fetch(dev);
		#if 0
		if (ret) {
			printk("sensor_sample_fetch failed ret %d\n", ret);
			return;
		}
		#endif
		if(ret == 0){
		ret = sensor_channel_get(dev, SENSOR_CHAN_TEMP, &temp_value);
		if (ret) {
			printk("sensor_channel_get failed ret %d\n", ret);
			return;
		}
		printk("temp is %d.%d\n", temp_value.val1,
                       temp_value.val2);
           
		ret = sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &temp_value);
                if (ret) {
                        printk("sensor_channel_get failed ret %d\n", ret);
                        return;
                }

		printk("humidity is %d.%d\n", temp_value.val1, temp_value.val2);
		}
		k_sleep(1000);
	}
}

void main(void){

	print_device_info();
	//init_led();
	init_rtc();
	read_dht1x();
}
