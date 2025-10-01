#ifndef F407_RM_TMPLATE_HAL_TASKLIST_H
#define F407_RM_TMPLATE_HAL_TASKLIST_H

#include "main_cpp.h"
#include "Debug.h"

#ifdef __cplusplus
extern "C" {
#endif


void TestTask(void const * argument);
void DebugTask(void const * argument);
void INS_Task(void const * argument);

#ifdef __cplusplus
}
#endif



#endif //F407_RM_TMPLATE_HAL_TASKLIST_H
