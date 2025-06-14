/**
 ******************************************************************************
 * @file    pwmc_sixstep.h
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file contains all definitions and functions prototypes for the
 *          six-step PWM component of the Motor Control SDK.
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
 * @ingroup PWMC_SixStep
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PWMC_SIXSTEP_H
#define PWMC_SIXSTEP_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#define NB_CONVERSIONS 16u

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup SixStep
 * @{
 */

/** @addtogroup PWMC_SixStep
 * @{
 */
/* Exported defines ------------------------------------------------------------*/

#define STEP_1     0U
#define STEP_2     1U
#define STEP_3     2U
#define STEP_4     3U
#define STEP_5     4U
#define STEP_6     5U
#define STEP_INIT  5U

/* Exported defines ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief  ThreePwm parameters definition
 */
typedef struct
{
  TIM_TypeDef *TIMx;           /*!< It contains the pointer to the timer used for PWM generation. */
  GPIO_TypeDef *pwm_en_u_port; /*!< phase u enable driver signal GPIO port */
  uint16_t pwm_en_u_pin;       /*!< phase u enable driver signal pin */
  GPIO_TypeDef *pwm_en_v_port; /*!< phase v enable driver signal GPIO port */
  uint16_t pwm_en_v_pin;       /*!< phase v enable driver signal pin */
  GPIO_TypeDef *pwm_en_w_port; /*!< phase w enable driver signal GPIO port */
  uint16_t pwm_en_w_pin;       /*!< phase w enable driver signal pin */
} PWMC_Params_t;

typedef struct
{
  uint32_t CCER_cfg[6];
  uint32_t CCER_Align_cfg;
  uint32_t CCMR1_BootCharge;
  uint32_t CCMR2_BootCharge;
  uint32_t CCMR1_CW_Align_cfg[6];
  uint32_t CCMR2_CW_Align_cfg[6];
  uint32_t CCMR1_CCW_Align_cfg[6];
  uint32_t CCMR2_CCW_Align_cfg[6];
  uint32_t CCMR1_Standard_cfg[6];
  uint32_t CCMR2_Standard_cfg[6];
  uint32_t CCMR1_LSMod_cfg[6];
  uint32_t CCMR2_LSMod_cfg[6];
  uint32_t CCER_QuasiSynch_cfg[6];
} PWMC_TimerCfg_t;

/**
 * @brief This structure is used to handle the data of an instance of the PWM  component
 *
 */
typedef struct
{
  uint16_t CntPh;                  /**< PWM Duty cycle phase*/
  uint16_t StartCntPh;             /**< Start-up PWM Duty cycle phase*/
  uint16_t ADCTriggerCnt;          /**< Timer output trigger point used for ADC triggering */
  uint16_t PWMperiod;              /**< PWM period expressed in timer clock cycles unit:
                                     *  @f$hPWMPeriod = TimerFreq_{CLK} / F_{PWM}@f$    */
  int16_t AlignFlag;               /*!< phase current 0 is reliable, 1 is bad */
  uint8_t AlignStep;
  uint8_t Step;                    /**< Current step number */
  uint8_t LSModArray[6];
  bool OverCurrentFlag;            /*!< This flag is set when an overcurrent occurs.*/
  bool OverVoltageFlag;            /*!< This flag is set when an overvoltage occurs.*/
  bool driverProtectionFlag;
  bool QuasiSynchDecay;
  bool TurnOnLowSidesAction;
  PWMC_Params_t const *pParams_str;
  uint32_t *pCCER_cfg;
  uint32_t *pCCMR1_cfg;
  uint32_t *pCCMR2_cfg;
  PWMC_TimerCfg_t *TimerCfg;
  LowSideOutputsFunction_t LowSideOutputs;
} PWMC_Handle_t;

/* Exported functions --------------------------------------------------------*/

/**
 * It initializes TIMx and NVIC
 */
void PWMC_Init(PWMC_Handle_t *pHandle);

/**
 * It updates the stored duty cycle variable.
 */
void PWMC_SetPhaseVoltage(PWMC_Handle_t *pHandle, uint16_t DutyCycle);

/**
 * It writes the duty cycle into shadow timer registers.
 */
void PWMC_LoadNextStep(PWMC_Handle_t *pHandle);

/**
 * Turns low sides on. This function is intended to be used for
 * charging boot capacitors of driving section. It has to be called on each
 * motor start-up when using high voltage drivers.
 */
void PWMC_TurnOnLowSides(PWMC_Handle_t *pHandle, uint32_t ticks);

/**
 * Switches the PWM generation on
 */
void PWMC_SwitchOnPWM(PWMC_Handle_t *pHandle);

/**
 * Set the trigger instant of the ADC for Bemf acquisition
 */
void PWMC_SetADCTriggerChannel(PWMC_Handle_t *pHdl, uint16_t SamplingPoint);

/**
 * Switches the PWM generation off, setting the outputs to inactive
 */
void PWMC_SwitchOffPWM(PWMC_Handle_t *pHandle);

/**
 * @brief  It contains the Break event interrupt.
 * @param  pHandle: handler of the current instance of the PWM component.
 * @retval none
 */
static inline void *PWMC_BRK_IRQHandler(PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    pHandle->OverCurrentFlag = true;
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  }
#endif
  return (MC_NULL);
}

/**
* @brief  It is used to set the align motor flag.
* @param  this related object of class CPWMC.
* @param  flag to be applied in uint8_t, 1: motor is in align stage, 2: motor is not in align stage.
* @retval none
*/
static inline void PWMC_SetAlignFlag(PWMC_Handle_t *pHandle, int16_t flag)
{
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do. */
  }
  else
  {
#endif
    pHandle->AlignFlag = flag;
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  }
#endif
}

/**
 * It forces the next step in closed loop operation
 */
void PWMC_ForceNextStep(PWMC_Handle_t *pHandle, int16_t Direction, uint8_t tStep);

/**
 * Checks if an overcurrent occurred since last call.
 */
uint16_t PWMC_IsFaultOccurred(PWMC_Handle_t *pHandle);

/**
 * it gets low side modulation status.
 */
uint8_t PWMC_GetLSModConfig(const PWMC_Handle_t *pHandle);

/**
 * it enables/disables low side modulation.
 */
void PWMC_SetLSModConfig(PWMC_Handle_t *pHandle, uint8_t newConfig);

/**
 * @brief  It returns the Quasi Synch feature status.
 * @param  pHandle: handler of the current instance of the PWM component.
 * @retval uint8_t: 0=disabled, 1=enabled
 */
static inline bool PWMC_GetQuasiSynchState(const PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PWM_SIXSTEP
  return ((MC_NULL == pHandle) ? false : pHandle->QuasiSynchDecay);
#else
  return (pHandle->QuasiSynchDecay);
#endif
}

/**
 * it enables/disables quasi-synchronous rectification feature.
 */
void PWMC_SetQuasiSynchState(PWMC_Handle_t *pHandle, uint8_t State);

/**
  * @brief  Retrieves the satus of TurnOnLowSides action.
  *
  * @param  pHandle: Handler of the current instance of the PWMC component.
  * @retval bool State of TurnOnLowSides action:
  *         **true** if TurnOnLowSides action is active, **false** otherwise.
  */
static inline bool PWMC_GetTurnOnLowSidesAction(const PWMC_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_PWR_CUR_FDB
  return ((MC_NULL == pHandle) ? false : pHandle->TurnOnLowSidesAction);
#else
  return (pHandle->TurnOnLowSidesAction);
#endif
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* PWMC_SIXSTEP_H*/

/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
