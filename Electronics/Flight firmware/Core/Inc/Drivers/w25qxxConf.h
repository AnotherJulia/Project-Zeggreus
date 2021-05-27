#include "main.h"

#ifndef _W25QXXCONFIG_H
#define _W25QXXCONFIG_H

#define _W25QXX_SPI                   hspi1
#define _W25QXX_CS_GPIO               FLASH_NSS_GPIO_Port
#define _W25QXX_CS_PIN                FLASH_NSS_Pin
#define _W25QXX_USE_FREERTOS          1
#define _W25QXX_DEBUG                 0

#endif
