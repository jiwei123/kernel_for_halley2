#include <linux/kernel.h>
extern void frizz_init();
#ifdef CONFIG_FRIZZ_VERIFY
const int config_verify = 1;
#else
const int config_verify = 0;
#endif

#ifdef CONFIG_SLPT
unsigned int slpt_onoff = 1;
#else
unsigned int slpt_onoff = 0;
#endif
#ifdef CONFIG_WAKEUP_BY_RAISE
const unsigned int config_wakeup_by_raise = 1;
#else
const unsigned int config_wakeup_by_raise = 0;
#endif
void __init fake ()
{
	frizz_init();
}
device_initcall(fake);

