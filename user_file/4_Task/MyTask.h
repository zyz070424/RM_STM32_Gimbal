#ifndef __TASK_H__
#define __TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

void Task_Init(void);
void Task_loop(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_H__ */
