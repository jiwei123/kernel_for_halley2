#ifndef __BCM4774_PM__
#define __BCM4774_PM__

enum {
	BCM4774_POWER_ON,
	BCM4774_POWER_OFF,
	BCM4774_POWER_MAX,
};

struct bcm4774_platform_data {
        char *regulator_name;
        unsigned int nSTANDBY_enable;
        int (*enable)(struct bcm4774_platform_data *, int);
        void (*clk_enable)(void);
        void (*clk_disable)(void);
};

#endif /*__BCM4774_PM__*/
