#ifndef __BCM_PM_CORE__
#define __BCM_PM_CORE__

struct bcm_power_platform_data {
    char    *bcm_power_name;
    int     bcm_pwr_en;
    void (*clk_enable)(void);
    void (*clk_disable)(void);
};

#endif /*__BCM_PM_CORE__*/
