#ifndef CONFIG_H
#define CONFIG_H

#include "common.h"

#define CONFIG_FORZE_WRITE false
#define CONFIG_VERSION 1

config_t *config_read();
void config_write(config_t *config);
void config_forze_write();
void config_get(config_t *config);

#endif