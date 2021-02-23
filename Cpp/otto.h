#pragma once

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void OTTO_preinit();
void OTTO_main_loop();
void OTTO_systick();

#ifdef __cplusplus
}
#endif
