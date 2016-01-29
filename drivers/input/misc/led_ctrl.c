/* LED control settings */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include "led_ctrl.h"
extern unsigned char ofn_write_reg(unsigned char addr, unsigned char data);
extern unsigned char ofn_read_reg(unsigned char addr, unsigned char *data);
#define __write_reg(a, b) ofn_write_reg(a, b)
#define __read_reg(a, b) ofn_read_reg(a, b)

static uint8_t _led_step = DEFAULT_LED_STEP;
static uint8_t _state = 0, _state_count = 0;
static uint8_t _led_current_change_flag = 0;
static uint8_t _sleepflag = 1 ;

uint8_t get_led_current_change_flag()
{
    return _led_current_change_flag;
}

void led_ctrl(uint8_t touch)
{
    if (touch == 0x80) {
        uint8_t data;
        uint16_t EP_L, EP_H, Exposure_Line; //leon_140213
        uint8_t _touch_threshold;

        __write_reg(0x7f, 0x00); //for bank0
        __write_reg(0x05, 0x98);

        __read_reg(0x33, &data);
        EP_H = data & 0x03; //Read Exposure Time high byte
        __read_reg(0x32, &data);
        EP_L = data; //Read Exposure Time low byte
        Exposure_Line = (EP_H << 8) + EP_L;

        __write_reg(0x7f, 0x01); //for bank1

        if (_sleepflag == 1) {
            __write_reg(0x38, (0xE0|DEFAULT_LED_STEP));
            _sleepflag = 0;
        }

        if (_state_count <= STATE_COUNT_TH) {
            _state_count++;
            _led_current_change_flag = 0;
        } else {
            _state_count = 0;

            if (_state == 0) {
                if ((Exposure_Line >= LED_CTRL_EXPO_TIME_HI_BOUND)
                        || (Exposure_Line <= LED_CTRL_EXPO_TIME_LOW_BOUND)) {
                    //__write_reg(0x7f,0x01);     //for bank1
                    __read_reg(0x38, &data);
                    _led_step = data & 0x1f;

                    if (Exposure_Line >= LED_CTRL_EXPO_TIME_HI_BOUND
                            && (_led_step < LED_CURRENT_HI)) {
                        _state = 1;
                        _led_step = _led_step + LED_INC_DEC_STEP;
                        if (_led_step > LED_CURRENT_HI)
                            _led_step = LED_CURRENT_HI;
                        __write_reg(0x38, (_led_step|0xE0));
                        _led_current_change_flag = 1;
                    } else if ((Exposure_Line <= LED_CTRL_EXPO_TIME_LOW_BOUND)
                            && (_led_step > LED_CURRENT_LOW)) {
                        _state = 2;
                        if (_led_step <= (LED_CURRENT_LOW + LED_INC_DEC_STEP)) {
                            _led_step = LED_CURRENT_LOW;
                        } else {
                            _led_step = _led_step - LED_INC_DEC_STEP;
                        }
                        __write_reg(0x38, (_led_step|0xE0));
                        _led_current_change_flag = 1;
                    } else {
                        _state = 0;
                        _led_current_change_flag = 0;
                    }

                    //__write_reg(0x38, (_led_step|0xE0));
                } else {
                    _led_current_change_flag = 0;
                }
            } else if (_state == 1) {
                if (Exposure_Line > LED_CTRL_EXPO_TIME_HI) {
                    _state = 1;
                    _led_step = _led_step + LED_INC_DEC_STEP;

                    if (_led_step >= LED_CURRENT_HI) {
                        _state = 0;
                        _led_step = LED_CURRENT_HI;
                    }
                    __write_reg(0x38, (_led_step|0xE0));
                    _led_current_change_flag = 1;
                } else {
                    _state = 0;
                    _led_current_change_flag = 0;
                }
            } else {
                if (Exposure_Line < LED_CTRL_EXPO_TIME_LOW) {
                    _state = 2;
                    if (_led_step <= (LED_CURRENT_LOW + LED_INC_DEC_STEP)) {
                        _state = 0;
                        _led_step = LED_CURRENT_LOW;
                    } else {
                        _led_step = _led_step - LED_INC_DEC_STEP;
                    }
                    __write_reg(0x38, (_led_step|0xE0));
                    _led_current_change_flag = 1;
                } else {
                    _state = 0;
                    _led_current_change_flag = 0;
                }
            }
        }
//        printk(KERN_DEBUG "led_step=====%d\n", _led_step);
    } else {
        __write_reg(0x7f, 0x00);
        //for bank0
        __write_reg(0x5, 0xB8);
        __write_reg(0x7F, 0x01);
        //__write_reg(0x42,0xA0);
        _led_step = DEFAULT_LED_STEP;
        __write_reg(0x38, 0xFF);
        _sleepflag = 1;
        _led_current_change_flag = 0;
    }

    //LED_Current_Change_Flag_D1  = LED_Current_Change_Flag;
}

