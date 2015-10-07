/*
 * arch/arm/mach-tegra/board-keenhi_t40.h
 *
 * Copyright (c) 2013, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _MACH_TEGRA_BOARD_keenhi_t40_H
#define _MACH_TEGRA_BOARD_keenhi_t40_H

#include <mach/gpio.h>
#include <mach/irqs.h>
#include "gpio-names.h"
#include <linux/thermal.h>
#include <linux/platform_data/thermal_sensors.h>
#include "tegra11_soctherm.h"

#define PMC_WAKE_STATUS 0x14
#define PMC_WAKE2_STATUS 0x168

/* External peripheral act as gpio */
#define PALMAS_TEGRA_GPIO_BASE	TEGRA_NR_GPIOS

/* Audio-related GPIOs */
#define TEGRA_GPIO_CDC_IRQ		TEGRA_GPIO_PW3
#define TEGRA_GPIO_LDO1_EN		TEGRA_GPIO_PX6//TEGRA_GPIO_PV3
#define TEGRA_GPIO_CODEC1_EN	TEGRA_GPIO_PP3
#define TEGRA_GPIO_CODEC2_EN	TEGRA_GPIO_PP1
#define TEGRA_GPIO_CODEC3_EN	TEGRA_GPIO_PV0
#if defined(CONFIG_MACH_T8400N) || defined(CONFIG_MACH_TB610N) || defined(CONFIG_MACH_KEENHI01N)
#define TEGRA_GPIO_SPKR_EN		TEGRA_GPIO_PX4
#else
#define TEGRA_GPIO_SPKR_EN		-1
#endif
#define TEGRA_GPIO_HP_DET		TEGRA_GPIO_PR7
#if defined(CONFIG_MACH_T8400N) || defined(CONFIG_MACH_KEENHI01N)
#define TEGRA_GPIO_INT_MIC_EN		TEGRA_GPIO_PR5
#elif defined(CONFIG_MACH_TB610N)
#define TEGRA_GPIO_INT_MIC_EN		TEGRA_GPIO_PN0
#else
#define TEGRA_GPIO_EXT_MIC_EN		-1
#endif
#define TEGRA_GPIO_RST 		TEGRA_GPIO_PX6
#define TEGRA_SOC_OC_IRQ_BASE		TEGRA_NR_IRQS
#define TEGRA_SOC_OC_NUM_IRQ		TEGRA_SOC_OC_IRQ_MAX

#define KEENHI_LIGHTSENSOR_I2C_BUS 0

/* External peripheral act as interrupt controller */
#define PALMAS_TEGRA_IRQ_BASE	(TEGRA_SOC_OC_IRQ_BASE + TEGRA_SOC_OC_NUM_IRQ)
#define PALMAS_TEGRA_IRQ_END	(PALMAS_TEGRA_IRQ_BASE + PALMAS_NUM_IRQ)

/* I2C related GPIOs */
#define TEGRA_GPIO_I2C1_SCL		TEGRA_GPIO_PC4
#define TEGRA_GPIO_I2C1_SDA             TEGRA_GPIO_PC5
#define TEGRA_GPIO_I2C2_SCL             TEGRA_GPIO_PT5
#define TEGRA_GPIO_I2C2_SDA             TEGRA_GPIO_PT6
#define TEGRA_GPIO_I2C3_SCL             TEGRA_GPIO_PBB1
#define TEGRA_GPIO_I2C3_SDA             TEGRA_GPIO_PBB2
#define TEGRA_GPIO_I2C4_SCL             TEGRA_GPIO_PV4
#define TEGRA_GPIO_I2C4_SDA             TEGRA_GPIO_PV5
#define TEGRA_GPIO_I2C5_SCL             TEGRA_GPIO_PZ6
#define TEGRA_GPIO_I2C5_SDA             TEGRA_GPIO_PZ7

/* Camera related GPIOs */
//#define CAM_RSTN			TEGRA_GPIO_PBB5
#define CAM_FLASH_STROBE		TEGRA_GPIO_PBB4
#define CAM1_POWER_DWN_GPIO		TEGRA_GPIO_PBB5
#define CAM2_POWER_DWN_GPIO		TEGRA_GPIO_PBB6
#define CAM_AF_PWDN			TEGRA_GPIO_PBB4
#define CAM_GPIO1			TEGRA_GPIO_PCC1
#define CAM_GPIO2			TEGRA_GPIO_PCC2
#define FLASH_ENABLE_PIN		TEGRA_GPIO_PCC1
/* Touchscreen definitions */
#define TOUCH_GPIO_IRQ_RAYDIUM_SPI      TEGRA_GPIO_PK2
#define TOUCH_GPIO_RST_RAYDIUM_SPI      TEGRA_GPIO_PK4

/*Touch releated GPIO*/
#define TEGRA_GPIO_TOUCH_DETECT   	 TEGRA_GPIO_PEE3
#define TEGRA_GPIO_TOUCH_DETECT_0   TEGRA_GPIO_PN7
#define TEGRA_GPIO_TOUCH_DETECT_1   TEGRA_GPIO_PK6
#define TEGRA_GPIO_TOUCH_DETECT_2      TEGRA_GPIO_PK5

/* hardware version control*/
#define TEGRA_HW_VER0       TEGRA_GPIO_PQ3
#define TEGRA_HW_VER1       TEGRA_GPIO_PS0
#define TEGRA_HW_VER2       TEGRA_GPIO_PS2

/*serial port enable*/
#define TEGRA_GPIO_SERIAL_ENABLE  TEGRA_GPIO_PR0
/* Invensense MPU Definitions */
#define MPU_GYRO_NAME           "mpu3050"
#define MPU_GYRO_IRQ_GPIO       TEGRA_GPIO_PR3
#define MPU_GYRO_ADDR           0x68
#define MPU_GYRO_BUS_NUM        0

#define MPU3050_ACCEL_NAME		"kxtf9"
#define MPU3050_ACCEL_IRQ_GPIO	TEGRA_GPIO_PO5//0 /* DISABLE ACCELIRQ:  TEGRA_GPIO_PL1 */
#if defined(CONFIG_MACH_TB610N)
#define MPU3050_ACCEL_ADDR		0x0E
#else
#define MPU3050_ACCEL_ADDR		0x0F
#endif
#define MPU3050_ACCEL_BUS_NUM	0

#define MPU_COMPASS_NAME        "ak8963"
#define MPU_COMPASS_IRQ_GPIO    0
#define MPU_COMPASS_ADDR        0x0D
#define MPU_COMPASS_BUS_NUM     0

#if defined(CONFIG_MACH_TERRA10)
#define MPU_GYRO_ORIENTATION	{ 0, -1, 0, -1, 0, 0, 0, 0, -1 }
#define MPU3050_ACCEL_ORIENTATION	{ 0,1, 0, 1, 0, 0, 0, 0, -1 }
#define MPU_COMPASS_ORIENTATION { 0,1, 0, 1, 0, 0, 0, 0, -1 }//{ 1, 0, 0, 0, -1, 0, 0, 0, -1 }
#elif defined(CONFIG_MACH_T8400N) || defined(CONFIG_MACH_KEENHI01N)
#define MPU_GYRO_ORIENTATION	{ 1, 0, 0, 0, -1, 0, 0, 0, -1 }
#define MPU3050_ACCEL_ORIENTATION	{ 0,-1, 0, 1, 0, 0, 0, 0, 1 }
#define MPU_COMPASS_ORIENTATION { -1, 0, 0, 0, 1, 0, 0, 0, -1 }
#elif defined(CONFIG_MACH_TB610N)
#define MPU_GYRO_ORIENTATION	{ 1, 0, 0, 0, -1, 0, 0, 0, -1 }
#define MPU3050_ACCEL_ORIENTATION	{ 0,-1, 0, -1, 0, 0, 0, 0, -1 }
#define MPU_COMPASS_ORIENTATION { -1, 0, 0, 0, 1, 0, 0, 0, -1 }
#else
#define MPU_GYRO_ORIENTATION	{ 1, 0, 0, 0, -1, 0, 0, 0, -1 }
#define MPU3050_ACCEL_ORIENTATION	{ -1,0, 0, 0, 1, 0, 0, 0, -1 }
#define MPU_COMPASS_ORIENTATION { -1, 0, 0, 0, 1, 0, 0, 0, -1 }
#endif

/* Modem related GPIOs */
#define MODEM_EN		TEGRA_GPIO_PP2
#define MDM_RST			TEGRA_GPIO_PP0
#define MDM_COLDBOOT		TEGRA_GPIO_PO5

int keenhi_t40_regulator_init(void);
int keenhi_t40_suspend_init(void);
int keenhi_t40_sdhci_init(void);
int keenhi_t40_pinmux_init(void);
int keenhi_t40_sensors_init(void);
int keenhi_t40_emc_init(void);
int keenhi_t40_edp_init(void);
int keenhi_t40_panel_init(void);
int roth_panel_init(void);
int keenhi_t40_kbc_init(void);
int keenhi_t40_pmon_init(void);
int keenhi_t40_soctherm_init(void);
void keenhi_t40_sysedp_init(void);
void keenhi_t40_sysedp_core_init(void);
void keenhi_t40_sysedp_psydepl_init(void);

/* UART port which is used by bluetooth*/
#define BLUETOOTH_UART_DEV_NAME "/dev/ttyHS2"

/* Baseband IDs */
enum tegra_bb_type {
	TEGRA_BB_NEMO = 1,
};

#define UTMI1_PORT_OWNER_XUSB	0x1
#define UTMI2_PORT_OWNER_XUSB	0x2
#define HSIC1_PORT_OWNER_XUSB	0x4

#endif
