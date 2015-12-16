#include <linux/kernel.h>
#include <linux/switch.h>

static struct switch_dev switch_usb = {
	.name = "usb_cable",
};
static int switch_state = 0;

void usb_switch_set_state(unsigned int state)
{
	if(switch_state < 0) {
		pr_err("Failed to register usb switch, please check!, [%s,%d]\n"
				,__func__,__LINE__);
		return;
	}
	switch_set_state(&switch_usb, state);
	return;
}
EXPORT_SYMBOL(usb_switch_set_state);

static int __init usb_switch_init(void)
{
	switch_state = switch_dev_register(&switch_usb);

	if (switch_state < 0)
		pr_err("Failed to register usb switch. %d\n", switch_state);

	return 0;
}
arch_initcall(usb_switch_init);

