
#ifndef __LINUX_ROOTLIST_H
#define __LINUX_ROOTLIST_H

#include <linux/ioctl.h>

enum root_e {
    nBLUETOOTH = _IO('R', 0),
    nLCD,
    nSYSTEM,
    nSENSOR,
    nMMI,
};

enum attr_e {
    tTYPE = _IO('A', 0),
    tCHIP,
    tNAME,
    tVERSION,
    tDPI,
    tPPI,
    tEXTERIOR,
    tPATH,
    tSIZE,
    tPORT,
    tITEM,
};

typedef enum root_e root_key;
typedef enum attr_e attr_key;

#ifdef CONFIG_ROOTLIST
extern int hwlist_create_attr(root_key root, attr_key attr, const char *value);
#else
#define hwlist_create_attr(a,b,c) 0
#endif

/*   bluetooth    */
static inline int hwlist_bluetooth_chip(const char *value)
{
    return hwlist_create_attr(nBLUETOOTH, tCHIP, value);
}

static inline int hwlist_bluetooth_port(const char *value)
{
    return hwlist_create_attr(nBLUETOOTH, tPORT, value);
}

/*   lcd   */
static inline int hwlist_lcd_chip(const char *value)
{
    return hwlist_create_attr(nLCD, tCHIP, value);
}

static inline int hwlist_lcd_name(const char *value)
{
    return hwlist_create_attr(nLCD, tNAME, value);
}

static inline int hwlist_lcd_size(const char *value)
{
    return hwlist_create_attr(nLCD, tSIZE, value);
}

static inline int hwlist_lcd_dpi(const char *value)
{
    return hwlist_create_attr(nLCD, tDPI, value);
}

static inline int hwlist_lcd_ppi(const char *value)
{
    return hwlist_create_attr(nLCD, tPPI, value);
}

static inline int hwlist_lcd_exterior(const char *value)
{
    return hwlist_create_attr(nLCD, tEXTERIOR, value);
}

/*   sensor   */
static inline int hwlist_sensor_chip(const char *value)
{
    return hwlist_create_attr(nSENSOR, tCHIP, value);
}

static inline int hwlist_sensor_type(const char *value)
{
    return hwlist_create_attr(nSENSOR, tTYPE, value);
}

/*   mmi   */
static inline int hwlist_mmi_item(const char *value)
{
    return hwlist_create_attr(nMMI, tITEM, value);
}
#endif

