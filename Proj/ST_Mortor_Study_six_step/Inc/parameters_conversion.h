
/**
  ******************************************************************************
  * @file    parameters_conversion.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file includes the proper Parameter conversion on the base
  *          of stdlib for the first drive
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PARAMETERS_CONVERSION_H
#define PARAMETERS_CONVERSION_H

#include "mc_math.h"
#include "parameters_conversion_g4xx.h"
#include "pmsm_motor_parameters.h"
#include "drive_parameters.h"
#include "power_stage_parameters.h"

#define CURRENT_CONV_FACTOR                 ((RSHUNT * AMPLIFICATION_GAIN) / ADC_REFERENCE_VOLTAGE)
#define CURRENT_CONV_FACTOR_INV             (1.0 / ((RSHUNT * AMPLIFICATION_GAIN) / ADC_REFERENCE_VOLTAGE))

#define NOMINAL_CURRENT                     NOMINAL_CURRENT_A
#define ADC_REFERENCE_VOLTAGE               3.3
#define M1_MAX_READABLE_CURRENT             (ADC_REFERENCE_VOLTAGE / ( RSHUNT * AMPLIFICATION_GAIN))

/************************* CONTROL FREQUENCIES & DELAIES **********************/

#define DPP_CONV_FACTOR                     65536

#define SYS_TICK_FREQUENCY                  (uint16_t)2000
#define UI_TASK_FREQUENCY_HZ                10U

#define MEDIUM_FREQUENCY_TASK_RATE          (uint16_t)SPEED_LOOP_FREQUENCY_HZ
#define MF_TASK_OCCURENCE_TICKS             (SYS_TICK_FREQUENCY / SPEED_LOOP_FREQUENCY_HZ) - 1u
#define UI_TASK_OCCURENCE_TICKS             (SYS_TICK_FREQUENCY / UI_TASK_FREQUENCY_HZ) - 1u
#define SERIALCOM_TIMEOUT_OCCURENCE_TICKS   (SYS_TICK_FREQUENCY / SERIAL_COM_TIMEOUT_INVERSE) - 1u
#define SERIALCOM_ATR_TIME_TICKS            (uint16_t)(((SYS_TICK_FREQUENCY * SERIAL_COM_ATR_TIME_MS) / 1000u) - 1u)

#define MAX_APPLICATION_SPEED_UNIT          ((MAX_APPLICATION_SPEED_RPM * SPEED_UNIT) / U_RPM)
#define MIN_APPLICATION_SPEED_UNIT          ((MIN_APPLICATION_SPEED_RPM * SPEED_UNIT) / U_RPM)
#define MAX_APPLICATION_SPEED_UNIT2         ((MAX_APPLICATION_SPEED_RPM2 * SPEED_UNIT) / U_RPM)
#define MIN_APPLICATION_SPEED_UNIT2         ((MIN_APPLICATION_SPEED_RPM2 * SPEED_UNIT) / U_RPM)

#define PERCENTAGE_FACTOR                   (uint16_t)(VARIANCE_THRESHOLD * 128u)

#define SPEED_TIMER_CONV_FACTOR             (uint32_t) ((10 * APB1TIM_FREQ / (POLE_PAIR_NUM * (LF_TIMER_PSC + 1))) * SPEED_UNIT/ U_RPM)

/************************* BEMF OBSERVER PARAMETERS **************************/
#define OBS_MINIMUM_SPEED_UNIT              (uint16_t)((OBS_MINIMUM_SPEED_RPM * SPEED_UNIT) / U_RPM)
#define BEMF_ERRORS_SCORE                   (uint16_t)(10 * M1_MAX_CONSECUTIVE_BEMF_ERRORS)
/*!< Conversion factor from bus voltage digits to bemf threshold digits */
#define BEMF_BUS2THRES_FACTOR               (uint16_t)(1000 * VBUS_PARTITIONING_FACTOR * BEMF_ON_SENSING_DIVIDER)
/*!< Correction factor for bemf divider diode voltage drop */
#define BEMF_CORRECT_FACTOR                 (uint16_t) (65536 * BEMF_DIVIDER_DIODE_V / (BEMF_ON_SENSING_DIVIDER * ADC_REFERENCE_VOLTAGE))

#define BEMF_ADC_TRIG_TIME                  (uint16_t)(PWM_PERIOD_CYCLES * BEMF_ADC_TRIG_TIME_DPP / 1024)

#define ZCD_RISING_TO_COMM_9BIT            (uint16_t) (ZCD_RISING_TO_COMM * 512 / 60) /*!< Zero Crossing detection to commutation delay in 60/512 degrees */
#define ZCD_FALLING_TO_COMM_9BIT           (uint16_t) (ZCD_FALLING_TO_COMM * 512 / 60) /*!< Zero Crossing detection to commutation delay in 60/512 degrees */

#define BEMF_ADC_TRIG_TIME_ON               (uint16_t)(PWM_PERIOD_CYCLES * BEMF_ADC_TRIG_TIME_ON_DPP / 1024)
#define BEMF_PWM_ON_ENABLE_THRES            (uint16_t)(PWM_PERIOD_CYCLES * BEMF_PWM_ON_ENABLE_THRES_DPP / 1024)
#define BEMF_PWM_ON_DISABLE_THRES           (uint16_t)(PWM_PERIOD_CYCLES * (BEMF_PWM_ON_ENABLE_THRES_DPP\
                                            - BEMF_PWM_ON_ENABLE_HYSTERESIS_DPP) / 1024)

#define PHASE1_VOLTAGE_DPP                  ((PWM_PERIOD_CYCLES * PHASE1_VOLTAGE_RMS * PHASE1_VOLTAGE_RMS)\
                                            / (NOMINAL_BUS_VOLTAGE_V * NOMINAL_BUS_VOLTAGE_V))
#define PHASE2_VOLTAGE_DPP                  ((PWM_PERIOD_CYCLES * PHASE2_VOLTAGE_RMS * PHASE2_VOLTAGE_RMS)\
                                            / (NOMINAL_BUS_VOLTAGE_V * NOMINAL_BUS_VOLTAGE_V))
#define PHASE3_VOLTAGE_DPP                  ((PWM_PERIOD_CYCLES * PHASE3_VOLTAGE_RMS * PHASE3_VOLTAGE_RMS)\
                                            / (NOMINAL_BUS_VOLTAGE_V * NOMINAL_BUS_VOLTAGE_V))
#define PHASE4_VOLTAGE_DPP                  ((PWM_PERIOD_CYCLES * PHASE4_VOLTAGE_RMS * PHASE4_VOLTAGE_RMS)\
                                            / (NOMINAL_BUS_VOLTAGE_V * NOMINAL_BUS_VOLTAGE_V))
#define PHASE5_VOLTAGE_DPP                  ((PWM_PERIOD_CYCLES * PHASE5_VOLTAGE_RMS * PHASE5_VOLTAGE_RMS)\
                                            / (NOMINAL_BUS_VOLTAGE_V * NOMINAL_BUS_VOLTAGE_V))

#define DEMAG_MINIMUM_SPEED                    SPEED_THRESHOLD_DEMAG * SPEED_UNIT / U_RPM
#define DEMAG_REVUP_CONV_FACTOR                (uint32_t)((((DEMAG_REVUP_STEP_RATIO * PWM_FREQUENCY * SPEED_UNIT)\
                                               / (600 * POLE_PAIR_NUM )) * LFTIM_PERIOD_CYCLES * TIM_CLOCK_DIVIDER ) / (LF_TIMER_PSC + 1))
#define DEMAG_RUN_CONV_FACTOR                  (uint32_t)((((DEMAG_RUN_STEP_RATIO * PWM_FREQUENCY * SPEED_UNIT)\
                                               / (600 * POLE_PAIR_NUM )) * LFTIM_PERIOD_CYCLES * TIM_CLOCK_DIVIDER )  / (LF_TIMER_PSC + 1))
#define MIN_DEMAG_COUNTER_TIME                 (uint32_t) ((MIN_DEMAG_TIME * LFTIM_PERIOD_CYCLES * TIM_CLOCK_DIVIDER ) / (LF_TIMER_PSC + 1))

/**************************   VOLTAGE CONVERSIONS  Motor 1 *************************/
#define OVERVOLTAGE_THRESHOLD_d             (uint16_t)(OV_VOLTAGE_THRESHOLD_V * 65535 /\
                                            (ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR))
#define OVERVOLTAGE_THRESHOLD_LOW_d         (uint16_t)(OV_VOLTAGE_THRESHOLD_V * 65535 /\
                                            (ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR))
#define UNDERVOLTAGE_THRESHOLD_d            (uint16_t)((UD_VOLTAGE_THRESHOLD_V * 65535) /\
                                            ((uint16_t)(ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR)))
#define INT_SUPPLY_VOLTAGE                  (uint16_t)(65536 / ADC_REFERENCE_VOLTAGE)
#define DELTA_TEMP_THRESHOLD                (OV_TEMPERATURE_THRESHOLD_C - T0_C)
#define DELTA_V_THRESHOLD                   (dV_dT * DELTA_TEMP_THRESHOLD)
#define OV_TEMPERATURE_THRESHOLD_d          ((V0_V + DELTA_V_THRESHOLD) * INT_SUPPLY_VOLTAGE)
#define DELTA_TEMP_HYSTERESIS               (OV_TEMPERATURE_HYSTERESIS_C)
#define DELTA_V_HYSTERESIS                  (dV_dT * DELTA_TEMP_HYSTERESIS)
#define OV_TEMPERATURE_HYSTERESIS_d         (DELTA_V_HYSTERESIS * INT_SUPPLY_VOLTAGE)

/*************** Timer for PWM generation  ******/
#define PWM_PERIOD_CYCLES                    (uint16_t)(((uint32_t)ADV_TIM_CLK_MHz * (uint32_t)1000000u\
                                             / ((uint32_t)(PWM_FREQUENCY))) & (uint16_t)0xFFFE)
#define DEADTIME_NS                          HW_DEAD_TIME_NS
#define DEAD_TIME_ADV_TIM_CLK_MHz           (ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER)
#define DEAD_TIME_COUNTS_1                  (DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS / 500uL)
#if (DEAD_TIME_COUNTS_1 <= 255)
#define DEAD_TIME_COUNTS                    (uint16_t)DEAD_TIME_COUNTS_1
#elif (DEAD_TIME_COUNTS_1 <= 508)
#define DEAD_TIME_COUNTS                    (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/2) /1000uL) + 128)
#elif (DEAD_TIME_COUNTS_1 <= 1008)
#define DEAD_TIME_COUNTS                    (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/8) /1000uL) + 320)
#elif (DEAD_TIME_COUNTS_1 <= 2015)
#define DEAD_TIME_COUNTS                    (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/16) /1000uL) + 384)
#else
#define DEAD_TIME_COUNTS 510
#endif

/*************** Timer for speed measurement  ******/
#define LFTIM_PERIOD_CYCLES                    (uint16_t)(((uint32_t)APB1TIM_FREQ \
                                             / ((uint32_t)(PWM_FREQUENCY))) & (uint16_t)0xFFFE)
/**********************/
/* MOTOR 1 ADC Timing */
/**********************/
/* In ADV_TIMER CLK cycles*/
#define SAMPLING_TIME                       ((ADC_SAMPLING_CYCLES * ADV_TIM_CLK_MHz) / ADC_CLK_MHz)

/* USER CODE BEGIN temperature */
#define M1_VIRTUAL_HEAT_SINK_TEMPERATURE_VALUE 25u
#define M1_TEMP_SW_FILTER_BW_FACTOR         250u
/* USER CODE END temperature */

/****** Prepares the UI configurations according the MCconfxx settings ********/
#define DAC_ENABLE
#define DAC_OP_ENABLE

/*******************************************************************************
  * UI configurations settings. It can be manually overwritten if special
  * configuartion is required.
*******************************************************************************/
/* Specific options of UI */
#define UI_CONFIG_M1                        (UI_CFGOPT_NONE DAC_OP_ENABLE FW_ENABLE DIFFTERM_ENABLE\
                                            | (MAIN_SCFG << MAIN_SCFG_POS)\
                                            | (AUX_SCFG << AUX_SCFG_POS)\
                                            | UI_CFGOPT_SETIDINSPDMODE PLLTUNING_ENABLE UI_CFGOPT_PFC_ENABLE\
                                            | UI_CFGOPT_PLLTUNING)
#define UI_CONFIG_M2
#define DIN_ACTIVE_LOW                      Bit_RESET
#define DIN_ACTIVE_HIGH                     Bit_SET

#define DOUT_ACTIVE_HIGH                    DOutputActiveHigh
#define DOUT_ACTIVE_LOW                     DOutputActiveLow

#define SAMPLING_CYCLE_CORRECTION           0.5 /* Add half cycle required by STM32G431RBTx ADC */
#define LL_ADC_SAMPLINGTIME_1CYCLES_5       LL_ADC_SAMPLINGTIME_1CYCLE_5

//cstat !MISRAC2012-Rule-20.10 !DEFINE-hash-multiple
#define LL_ADC_SAMPLING_CYCLE(CYCLE)        LL_ADC_SAMPLINGTIME_ ## CYCLE ## CYCLES_5

#endif /*PARAMETERS_CONVERSION_H*/

/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
