/*
 * Copyright (C) 2016 Ingenic Semiconductor
 * SunWenZhong(Fighter) <wenzhong.sun@ingenic.com, wanmyqawdr@126.com>
 *
 * Release under GPLv2
 *
 */

#ifndef PMU_RICOH619_AW808_H
#define PMU_RICOH619_AW808_H

#define PMU_I2C_BUSNUM          0

/* ****************************PMU DCDC MODE at POWER_ON STATE*************** */
#define DCDC_AUTO_MODE          0
#define DCDC_PWM_MODE           1
#define DCDC_PSM_MODE           2
/* ****************************PMU DCDC MODE at POWER_ON STATE END*********** */

/* ****************************PMU DC/LDO NAME******************************* */
#define DC1_NAME                "CPU core"
#define DC2_NAME                "LPDDR2, CPU MEM, SENSOR HUB"
#define DC3_NAME                "NO USE-P0"
#define DC4_NAME                "EMMC, LPDDR2, CPUIO, PLL, SENSOR, SENSOR HUB"
#define DC5_NAME                "LDO3, LDO4, LDO6, LDO9, LDO10, Speaker, Motor"
#define LDO1_NAME               "BT-VDDIO"
#define LDO2_NAME               "EMMC, CPU_ADC, CPU_CODEC, CPU_USB"
#define LDO3_NAME               "NO USE-P1"
#define LDO4_NAME               "Amoled-P0"
#define LDO5_NAME               "CPU_USB, CPU_MIPI, if is SENSOR 9-Axis"
#define LDO6_NAME               "Amoled-P1"
#define LDO7_NAME               "NJRC heart rate-P0"
#define LDO8_NAME               "NJRC heart rate-P1"
#define LDO9_NAME               "TP-P0"
#define LDO10_NAME              "TP-P1"
#define LDORTC1_NAME            "rtc_1v8"
#define LDORTC2_NAME            "rtc_1v1"

/* ****************************PMU DC/LDO NAME END*************************** */

/* ****************************PMU DC/LDO DEFAULT V************************** */
#define DC1_INIT_UV             1150    /* CPU core */
#define DC2_INIT_UV             1150    /* LPDDR2, CPU MEM, SENSOR HUB*/
#define DC3_INIT_UV             0       /* NO USE */
#define DC4_INIT_UV             1725    /* EMMC, LPDDR2, CPUIO, PLL, SENSOR, SENSOR HUB */
#define DC5_INIT_UV             3000    /* LDO3, LDO4, LDO6, LDO9, LDO10, Speaker, Motor */
#define LDO1_INIT_UV            1725    /* BT-VDDIO */
#define LDO2_INIT_UV            2850    /* EMMC, CPU_ADC, CPU_CODEC, CPU_USB */
#define LDO3_INIT_UV            0       /* NO USE */
#define LDO4_INIT_UV            1725    /* Amoled */
#define LDO5_INIT_UV            2500    /* CPU_USB, CPU_MIPI, if is SENSOR 9-Axis */
#define LDO6_INIT_UV            2800    /* Amoled */
#define LDO7_INIT_UV            3000    /* NJRC heart rate */
#define LDO8_INIT_UV            1725    /* NJRC heart rate */
#define LDO9_INIT_UV            1725    /* TP */
#define LDO10_INIT_UV           2800    /* TP */

#define LDORTC1_INIT_UV         1800
#define LDORTC2_INIT_UV         1100
/* ****************************PMU DC/LDO DEFAULT V END********************** */

/* ****************************PMU DC/LDO ALWAYS ON************************** */
#define DC1_ALWAYS_ON           1
#define DC2_ALWAYS_ON           1
#define DC3_ALWAYS_ON           0
#define DC4_ALWAYS_ON           1
#define DC5_ALWAYS_ON           0
#define LDO1_ALWAYS_ON          1
#define LDO2_ALWAYS_ON          1
#define LDO3_ALWAYS_ON          0
#define LDO4_ALWAYS_ON          1
#define LDO5_ALWAYS_ON          1
#define LDO6_ALWAYS_ON          1
#define LDO7_ALWAYS_ON          0
#define LDO8_ALWAYS_ON          0
#define LDO9_ALWAYS_ON          0
#define LDO10_ALWAYS_ON         0
#define LDORTC1_ALWAYS_ON       1
#define LDORTC2_ALWAYS_ON       1
/* ****************************PMU DC/LDO ALWAYS ON END********************** */

/* ****************************PMU DC/LDO BOOT ON**************************** */
#define DC1_BOOT_ON             1
#define DC2_BOOT_ON             1
#define DC3_BOOT_ON             0
#define DC4_BOOT_ON             1
#define DC5_BOOT_ON             1
#define LDO1_BOOT_ON            1
#define LDO2_BOOT_ON            1
#define LDO3_BOOT_ON            0
#define LDO4_BOOT_ON            1
#define LDO5_BOOT_ON            1
#define LDO6_BOOT_ON            1
#define LDO7_BOOT_ON            0
#define LDO8_BOOT_ON            0
#define LDO9_BOOT_ON            0
#define LDO10_BOOT_ON           0
#define LDORTC1_BOOT_ON         1
#define LDORTC2_BOOT_ON         1
/* ****************************PMU DC/LDO BOOT ON END************************ */

/* ****************************PMU DC/LDO INIT ENABLE************************ */
#define DC1_INIT_ENABLE         DC1_BOOT_ON
#define DC2_INIT_ENABLE         DC2_BOOT_ON
#define DC3_INIT_ENABLE         DC3_BOOT_ON
#define DC4_INIT_ENABLE         DC4_BOOT_ON
#define DC5_INIT_ENABLE         DC5_BOOT_ON
#define LDO1_INIT_ENABLE        LDO1_BOOT_ON
#define LDO2_INIT_ENABLE        LDO2_BOOT_ON
#define LDO3_INIT_ENABLE        LDO3_BOOT_ON
#define LDO4_INIT_ENABLE        LDO4_BOOT_ON
#define LDO5_INIT_ENABLE        LDO5_BOOT_ON
#define LDO6_INIT_ENABLE        LDO6_BOOT_ON
#define LDO7_INIT_ENABLE        LDO7_BOOT_ON
#define LDO8_INIT_ENABLE        LDO8_BOOT_ON
#define LDO9_INIT_ENABLE        LDO9_BOOT_ON
#define LDO10_INIT_ENABLE       LDO10_BOOT_ON
#define LDORTC1_INIT_ENABLE     LDORTC1_BOOT_ON
#define LDORTC2_INIT_ENABLE     LDORTC2_BOOT_ON
/* ****************************PMU DC/LDO INIT ENABLE END******************** */

/* ****************************PMU DCDC5 SUPPLY LDOS ********************* */
/*
 * the pmu supply is in group
 *----------------------------------
 * VINL1 : ldo1 ldo2 rtc1.8 rtc1.1
 * VINL2 : ldo5 ldo7 ldo8
 * VINL3 : ldo3 ldo4
 * VINL4 : ldo6 ldo9 ldo10
 *----------------------------------
 */
#define DC5_SUPPLY_LDO_5_7_8    0
#define DC5_SUPPLY_LDO_3_4      1
#define DC5_SUPPLY_LDO_6_9_10   1
/* ****************************PMU DCDC5 SUPPLY LDOS END ********************* */

/* ****************************PMU LDO1-6 ECO SLEEP MODE********************* */

/**
 * LOD7 ~ LDO10 DO NOT SUPPORT ECO MODE
 */
#define LDO1_ECO_SLEEP          1
#define LDO2_ECO_SLEEP          1
#define LDO3_ECO_SLEEP          1
#define LDO4_ECO_SLEEP          1
#define LDO5_ECO_SLEEP          1
#define LDO6_ECO_SLEEP          1
/* ****************************PMU LDO1~6 ECO SLEEP MODE END***************** */

/* ****************************PMU DCDC POWER_ON MODE************************ */
#define DCDC1_MODE              DCDC_AUTO_MODE
#define DCDC2_MODE              DCDC_AUTO_MODE
#define DCDC3_MODE              DCDC_PSM_MODE
#define DCDC4_MODE              DCDC_AUTO_MODE
#define DCDC5_MODE              DCDC_PSM_MODE
/* ****************************PMU DCDC POWER_ON MODE END******************** */

#endif
