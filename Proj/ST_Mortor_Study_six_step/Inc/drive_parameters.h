
/**
  ******************************************************************************
  * @file    drive_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a motor drive.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVE_PARAMETERS_H
#define DRIVE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED_RPM          1572 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM          (0 + (U_RPM / SPEED_UNIT)) /*!< rpm, mechanical, absolute value */
#define M1_SS_MEAS_ERRORS_BEFORE_FAULTS    3 /*!< Number of speed measurement errors before main sensor goes in fault */

/****** Bemf Observer ****/
#define BEMF_AVERAGING_FIFO_DEPTH          8 /*!< depth of the FIFO used to average  mechanical speed in 0.1Hz resolution */
#define VARIANCE_THRESHOLD                 0.1 /*!< Maximum accepted variance on speed estimates (percentage) */
#define BEMF_THRESHOLD_PWM_PERC            50 /*!< Percentage of Bus for zero crossing detection with on-sensing */
#define BEMF_THRESHOLD_HIGH_PERC           100 /*!< Percentage of Bus for zero crossing detection with on-sensing and low side modulation */
#define BEMF_THRESHOLD_LOW_PERC  	       1	/*!< Percentage of Bus for zero crossing detection with on-sensing and high side modulation */
#define BEMF_ADC_TRIG_TIME_DPP             ((uint16_t)921) /*!< 1/1024 of PWM period elapsed */
#define BEMF_ADC_TRIG_TIME_ON_DPP          ((uint16_t)256) /*!< 1/1024 of PWM period  elapsed */
#define BEMF_PWM_ON_ENABLE_THRES_DPP       ((uint16_t)716) /*!< 1/1024 of PWM period   elapsed */
#define BEMF_PWM_ON_ENABLE_HYSTERESIS_DPP  ((uint16_t)51) /*!< 1/1024 of PWM period    elapsed */
#define ZCD_RISING_TO_COMM                 ((uint16_t)30) /*!< Zero Crossing detection to commutation delay in degrees */
#define ZCD_FALLING_TO_COMM                ((uint16_t)30) /*!< Zero Crossing detection to commutation delay in degrees */
#define MIN_DEMAG_TIME                     ((uint16_t)3) /*!< Demagnetization delay in number of HF timer periods elapsed before a first
                                                           BEMF ADC measurement is processed in an attempt to detect the BEMF zero crossing */
#define SPEED_THRESHOLD_DEMAG              ((uint32_t)786) /*!< Speed threshold above which the  RUN_DEMAGN_DELAY_MIN is applied */
#define DEMAG_RUN_STEP_RATIO               ((uint16_t)20) /*!< Percentage of step time allowed for  demagnetization */
#define DEMAG_REVUP_STEP_RATIO             ((uint16_t)20) /*!< Percentage of step time allowed for demagnetization */

#define M1_MAX_CONSECUTIVE_BEMF_ERRORS     3 /*!< Max # of consecutive zero crossing event not detected */

#define ADC_AWD_FILTER_NUMBER              (uint8_t)0 /*!< Analog watchdog filtering configuration */

#define COMPUTATION_DELAY                  -2 /*!< Selection of the period for computation of the delay between the zero crossing and the step change */

#define BEMF_DIVIDER_DIODE_V               1 /*!< Voltage drop of the GPIO bemf divider diode */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */
#define PWM_FREQUENCY                      15000
#define PWM_FREQ_SCALING                   1
#define PWM_FREQUENCY_REF                  60000
#define LOW_SIDE_SIGNALS_ENABLING          ES_GPIO

#define REP_COUNTER                        0

/* Speed control loop */
#define SPEED_LOOP_FREQUENCY_HZ            (uint16_t)1000 /*!<Execution rate of speed regulation loop (Hz) */
#define PID_SPEED_KP_DEFAULT          2316/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT          238/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT          0/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/

/* Speed PID parameter dividers */
#define SP_KPDIV                           512
#define SP_KIDIV                           16384
#define SP_KDDIV                           16
#define SP_KPDIV_LOG                       LOG2((512))
#define SP_KIDIV_LOG                       LOG2((16384))
#define SP_KDDIV_LOG                       LOG2((16))

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV 1 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING     DISABLE
#define PERIODMAX                          (uint16_t)((PWM_PERIOD_CYCLES * 100 / 100) + 1)
#define LF_TIMER_ARR                       65535
#define LF_TIMER_PSC                       100U

/* Default settings */
#define DEFAULT_CONTROL_MODE               MCM_SPEED_MODE
#define DEFAULT_DRIVE_MODE                 VM /*!< VOLTAGE_MODE (VM) or CURRENT_MODE (CM) */
#define DEFAULT_TARGET_SPEED_RPM           236
#define DEFAULT_TARGET_SPEED_UNIT          (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/U_RPM)

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
#define OV_VOLTAGE_THRESHOLD_V             15 /*!< Over-voltage threshold */
#define UD_VOLTAGE_THRESHOLD_V             7 /*!< Under-voltage threshold */
#ifdef NOT_IMPLEMENTED
#define ON_OVER_VOLTAGE                    TURN_OFF_PWM /*!< TURN_OFF_PWM */
#endif /* NOT_IMPLEMENTED */
#define OV_TEMPERATURE_THRESHOLD_C         110 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C        10 /*!< Celsius degrees */

/******************************   START-UP PARAMETERS   **********************/

/* Phase 1 */
#define PHASE1_DURATION                    500 /* milliseconds */
#define PHASE1_FINAL_SPEED_UNIT            (0*SPEED_UNIT/U_RPM)
#define PHASE1_VOLTAGE_RMS                 0.52

/* Phase 2 */
#define PHASE2_DURATION                    1000 /* milliseconds */
#define PHASE2_FINAL_SPEED_UNIT            (236*SPEED_UNIT/U_RPM)
#define PHASE2_VOLTAGE_RMS                 4.6

/* Phase 3 */
#define PHASE3_DURATION                    500 /* milliseconds */
#define PHASE3_FINAL_SPEED_UNIT            (236*SPEED_UNIT/U_RPM)
#define PHASE3_VOLTAGE_RMS                 4.6

/* Phase 4 */
#define PHASE4_DURATION                    0 /* milliseconds */
#define PHASE4_FINAL_SPEED_UNIT            (0*SPEED_UNIT/U_RPM)
#define PHASE4_VOLTAGE_RMS                 0.52

/* Phase 5 */
#define PHASE5_DURATION                    0 /* milliseconds */
#define PHASE5_FINAL_SPEED_UNIT            (0*SPEED_UNIT/U_RPM)
#define PHASE5_VOLTAGE_RMS                 0.52

/* Sensor-less rev-up sequence */
#define ALIGN_STEP                         1 /*!< Step during rotor alignment*/

/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM              236

#define NB_CONSECUTIVE_TESTS               10 /* corresponding to former NB_CONSECUTIVE_TESTS / (TF_REGULATION_RATE /  MEDIUM_FREQUENCY_TASK_RATE) */

/******************************   Initial Position Detection   **********************/

/******************************   BUS VOLTAGE Motor 1  **********************/
#define  M1_VBUS_SAMPLING_TIME             LL_ADC_SAMPLING_CYCLE(47)

/******************************   Temperature sensing Motor 1  **********************/
#define  M1_TEMP_SAMPLING_TIME             LL_ADC_SAMPLING_CYCLE(47)

#define REGULAR_CONVERSION_RATE_MS         5U

/******************************   ADDITIONAL FEATURES   **********************/

/*** On the fly start-up ***/

/**************************
 *** Control Parameters ***
 **************************/

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
