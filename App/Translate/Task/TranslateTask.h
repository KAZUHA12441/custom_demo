#pragma once

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

void translateTask(void const * argument);

#ifdef __cplusplus
}
#endif
