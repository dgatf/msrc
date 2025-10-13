#ifndef CONFIG_H
#define CONFIG_H

#include "common.h"

#define CONFIG_FORZE_WRITE false
#define CONFIG_VERSION 2
#define CONFIG_FLASH_TARGET_OFFSET (768 * 1024)  // 768kB after start of flash. Program (uf2) max size 768kB

extern context_t context;

config_t *config_read();
void config_write(config_t *config);
void config_forze_write();
void config_get(config_t *config);

#endif