#ifndef EM30719_H_
#define EM30719_H_

#include <linux/ioctl.h>

struct em30719_platform_data {
    int gpio_int;
    int (*board_init)(struct device *dev);
    int (*board_exit)(struct device *dev);
    int (*power_on)(void);
    int (*power_off)(void);
};

/*------------ REGISTER-----------*/
#define ALP_PS_PID_REG              0x00
#define ALP_PS_CONFIG_REG           0x01
#define ALP_PS_INTERRUPT_REG        0x02
#define ALP_PS_PROX_LT_REG          0x03
#define ALP_PS_PROX_HT_REG          0x04
#define ALP_PS_ALSIR_TH1_REG        0x05
#define ALP_PS_ALSIR_TH2_REG        0x06
#define ALP_PS_ALSIR_TH3_REG        0x07
#define ALP_PS_PROX_DATA_REG        0x08
#define ALP_PS_ALSIR_DT1_REG        0x09
#define ALP_PS_ALSIR_DT2_REG        0x0A
#define ALP_PS_RESET_REG            0x0E
#define ALP_PS_OFFSET_REG           0x0F

#define EM30719_PID                 0X31

#endif /* EM30719_H_ */
