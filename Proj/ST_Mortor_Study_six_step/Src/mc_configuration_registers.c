
/*******************************************************************************
  * @file    mc_configuration_registers.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides project configuration information registers.
  *
  *
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
  */

#include "mc_type.h"
#include "mc_configuration_registers.h"
#include "register_interface.h"
#include "parameters_conversion.h"

#define FIRMWARE_NAME_STR "ST MC SDK\tVer.6.4.0"

const char_t CTL_BOARD[] = "NUCLEO-G431RB";
static const char_t M1_PWR_BOARD[] = "X-NUCLEO-IHM16M1";
const char_t FIRMWARE_NAME [] = FIRMWARE_NAME_STR;

const GlobalConfig_reg_t globalConfig_reg =
{
  .SDKVersion     = SDK_VERSION,
  .MotorNumber    =  1 ,
  .MCP_Flag       = FLAG_MCP_OVER_STLINK + FLAG_MCP_OVER_UARTA + FLAG_MCP_OVER_UARTB,
  .MCPA_UARTA_LOG = 0, /* Used by Motor Pilot to detect data log enabled. */
  .MCPA_UARTB_LOG = 0,
  .MCPA_STLNK_LOG = 0,
};

static const ApplicationConfig_reg_t M1_ApplicationConfig_reg =
{
  .maxMechanicalSpeed = 1572,
  .maxReadableCurrent = M1_MAX_READABLE_CURRENT,
  .nominalCurrent     = 2.160,
  .nominalVoltage     = 13,
  .driveType          = DRIVE_TYPE_M1,
};

//cstat !MISRAC2012-Rule-9.2
static const MotorConfig_reg_t M1_MotorConfig_reg =
{
  .polePairs  = 7,
  .ratedFlux  = 5.0,
  .rs         = 5.29,
  .ls         = 0.001058*1.000,
  .ld         = 0.001058,
  .maxCurrent = 2.160,
  .name = "GimBal GBM2804H-100T"
};

static const SixStepFwConfig_reg_t M1_SixStepConfig_reg =
{
  .primarySensor      = (uint8_t)PRIM_SENSOR_M1,
  .topology           = (uint8_t)TOPOLOGY_M1,
  .PWMFrequency       = (uint32_t)PWM_FREQ_M1,
  .MediumFrequency    = (uint16_t)MEDIUM_FREQUENCY_TASK_RATE,
  .configurationFlag1 = (uint16_t)configurationFlag1_M1, //cstat !MISRAC2012-Rule-10.1_R6
  .configurationFlag2 = (uint16_t)configurationFlag2_M1, //cstat !MISRAC2012-Rule-10.1_R6
  .driveMode          = (uint8_t)DEFAULT_DRIVE_MODE
};

const char_t * PWR_BOARD_NAME[NBR_OF_MOTORS] = {M1_PWR_BOARD};
const SixStepFwConfig_reg_t* SixStepConfig_reg[NBR_OF_MOTORS] = {&M1_SixStepConfig_reg};
const MotorConfig_reg_t* MotorConfig_reg[NBR_OF_MOTORS] = {&M1_MotorConfig_reg};
const ApplicationConfig_reg_t* ApplicationConfig_reg[NBR_OF_MOTORS] = {&M1_ApplicationConfig_reg};

/************************ (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
