
/**
  ******************************************************************************
  * @file    mc_tasks.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implementes tasks definition.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup MCTasks
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MCTASKS_H
#define MCTASKS_H

/* Includes ------------------------------------------------------------------*/
#include "mc_parameters.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MCTasks Motor Control Tasks
  *
  * @brief Motor Control subsystem configuration and operation routines.
  *
  * @{
  */

#define STOPPERMANENCY_MS              ((uint16_t)400)
#define STOPPERMANENCY_MS2             ((uint16_t)400)
#define STOPPERMANENCY_TICKS           (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)  / ((uint16_t)1000))
#define STOPPERMANENCY_TICKS2          (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2) / ((uint16_t)1000))

/* Initializes the Motor subsystem core according to user defined parameters */
void MCboot(MCI_Handle_t *pMCIList[NBR_OF_MOTORS]);

/* Runs all the Tasks of the Motor Control cockpit */
void MC_RunMotorControlTasks(void);

/* Executes the Medium Frequency Task functions for each drive instance */
void MC_Scheduler(void);

/* Executes safety checks (e.g. bus voltage and temperature) for all drive instances */
void TSK_SafetyTask(void);

/* */
void SIX_STEP_Init(void);

/* */
void SIX_STEP_Clear(uint8_t bMotor);

/* */
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);

/* */
bool TSK_ChargeBootCapDelayHasElapsedM1(void);

/* */
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);

/* */
bool TSK_StopPermanencyTimeHasElapsedM1(void);

/* */
void TSK_SetStopPermanencyTimeM2(uint16_t SysTickCount);

/* */
void TSK_SetChargeBootCapDelayM2(uint16_t hTickCount);

/* */
bool TSK_StopPermanencyTimeHasElapsedM2(void);

/* */
bool TSK_ChargeBootCapDelayHasElapsedM2(void);

void UI_HandleStartStopButton_cb(void);

/* Puts the Motor Control subsystem in in safety conditions on a Hard Fault */
void TSK_HardwareFaultTask(void);

 /* Locks GPIO pins used for Motor Control to prevent accidental reconfiguration */
void mc_lock_pins(void);

 /* Execute the speed timer configuration */
void TSK_SpeedTIM_task(void);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* MCTASKS_H */
/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
