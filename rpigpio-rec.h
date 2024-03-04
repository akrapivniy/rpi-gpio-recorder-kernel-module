#ifndef __RPIGREC_GPIO_H__
#define __RPIGREC_GPIO_H__

#include <linux/ioctl.h>

#define RPIGREC_MAGIC_IOCTL    'M'
#define RPIGREC_GET_GPIO_VALUE _IOR(RPIGREC_MAGIC_IOCTL, 1, uint32_t)
#define RPIGREC_BUFFER_RESET   _IO(RPIGREC_MAGIC_IOCTL, 2)

#endif // __RPIGREC_GPIO_H__
