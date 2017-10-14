/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _STM32_WWDG_H_
#define _STM32_WWDG_H_

#include <zephyr/types.h>

/**
 * @brief Driver for Independent Watchdog (WWDG) for STM32 MCUs
 *
 * Based on reference manual:
 *   STM32F101xx, STM32F102xx, STM32F103xx, STM32F105xx and STM32F107xx
 *   advanced ARM ® -based 32-bit MCUs
 *
 * Chapter 19: Independent watchdog (WWDG)
 *
 */


/* counter reload trigger */
#define STM32_WWDG_KR_RELOAD 0xaaaa
/* magic value for unlocking write access to PR and RLR */
#define STM32_WWDG_KR_UNLOCK 0x5555
/* watchdog start */
#define STM32_WWDG_SR_EWIF_CLEAN  0



/* 19.4.2 WWDG_CR */
union __wwdg_cr {
	u32_t val;
	struct {
        u32_t t :7 __packed;
        u32_t wdga : 1 __packed;
		u32_t rsvd__8_31 :24 __packed;
	} bit;
};

/* 19.4.3 WWDG_CFR */
union __wwdg_cfr {
	u32_t val;
	struct {
        u32_t w :7 __packed;
        u32_t wdgtb : 2 __packed;
        u32_t ewi : 1 __packed;
		u32_t rsvd__10_31 :22 __packed;
	} bit;
};

/* 19.6.3 WWDG_SR */
union __wwdg_sr {
	u32_t val;
	struct {
		u32_t ewif :1 __packed;
		u32_t rsvd__1_31 :31 __packed;
	} bit;
};

/* 19.4.5 WWDG register map */
struct wwdg_stm32 {
	__IO union __wwdg_cr cr;
	__IO union __wwdg_cfr cfr;
	__IO union __wwdg_sr sr;
};

#endif	/* _STM32_WWDG_H_ */
