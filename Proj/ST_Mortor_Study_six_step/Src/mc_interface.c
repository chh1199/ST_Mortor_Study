
/**
  ******************************************************************************
  * @file    mc_interface.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the MC Interface component of the Motor Control SDK:
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
  * @ingroup MCInterface
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_math.h"
#include "speed_duty_ctrl.h"
#include "mc_interface.h"
#include "motorcontrol.h"

#define ROUNDING_OFF

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup CAI
  * @{
  */

/** @defgroup MCInterface Motor Control Interface
  * @brief MC Interface component of the Motor Control SDK
  *
  *  This interface allows for performing basic operations on the motor driven by a
  *  Motor Control SDK based application. With it, motors can be started and stopped, speed or
  *  torque ramps can be programmed and executed and information on the state of the motor can
  *  be retrieved, among others.
  *
  *  These functions aims at being the main interface used by an application to control the motor.
  *
  * @{
  */
/* Private macros ------------------------------------------------------------*/

#define round(x) ((x)>=0?(int32_t)((x)+0.5):(int32_t)((x)-0.5))

/* Functions -----------------------------------------------*/

/**
  * @brief  Programs a motor speed ramp
  *
  * @param  pHandle Pointer on the component instance to operate on.
  * @param  hFinalSpeed The value of mechanical rotor speed reference at the
  *         end of the ramp expressed in the unit defined by #SPEED_UNIT.
  * @param  hDurationms The duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the
  *         value.
  *
  *  This command is executed immediately if the target motor's state machine is in
  * the #RUN state. Otherwise, it is buffered and its execution is delayed until This
  * state is reached.
  *
  * Users can check the status of the command by calling the MCI_IsCommandAcknowledged()
  * function.
  *
  * @sa MCI_ExecSpeedRamp
  */
__weak void MCI_ExecSpeedRamp(MCI_Handle_t *pHandle, int16_t hFinalSpeed, uint16_t hDurationms)
{
#ifdef NULL_PTR_CHECK_MC_INT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if ((hFinalSpeed * pHandle->hFinalSpeed) >= 0)
    {
      pHandle->lastCommand = MCI_CMD_EXECSPEEDRAMP;
      pHandle->hFinalSpeed = hFinalSpeed;
      pHandle->hDurationms = hDurationms;
      pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
      pHandle->LastModalitySetByUser = MCM_SPEED_MODE;
    }
    else
    {
      pHandle->DirectCommand = MCI_STOP;
      pHandle->hFinalSpeed = hFinalSpeed;
      pHandle->hDurationms = hDurationms;
      pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
      pHandle->LastModalitySetByUser = MCM_SPEED_MODE;
    }
#ifdef NULL_PTR_CHECK_MC_INT
  }
#endif
}

/**
  * @brief  Programs a motor speed ramp
  *
  * @param  pHandle Pointer on the component instance to operate on.
  * @param  FinalSpeed is the value of mechanical rotor speed reference at the
  *         end of the ramp expressed in RPM.
  * @param  hDurationms the duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the
  *         value.
  *
  *  This command is executed immediately if the target motor's state machine is in
  * the #RUN state. Otherwise, it is buffered and its execution is delayed until This
  * state is reached.
  *
  * Users can check the status of the command by calling the MCI_IsCommandAcknowledged()
  * function.
  *
  * @sa MCI_ExecSpeedRamp_F
  */
__weak void MCI_ExecSpeedRamp_F(MCI_Handle_t *pHandle, const float_t FinalSpeed, uint16_t hDurationms)
{
#ifdef NULL_PTR_CHECK_MC_INT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if ((FinalSpeed * (float_t)pHandle->pSDC->SpeedRefUnitExt) >= 0.0f)
    {
      float_t hFinalSpeed = ((FinalSpeed * (float_t)SPEED_UNIT) / (float_t)U_RPM);
      MCI_ExecSpeedRamp(pHandle, (int16_t)hFinalSpeed, hDurationms);
  }
  else
  {
      pHandle->DirectCommand = MCI_STOP;
      float_t hFinalSpeed = ((FinalSpeed * (float_t)SPEED_UNIT) / (float_t)U_RPM);
      MCI_ExecSpeedRamp(pHandle, (int16_t)hFinalSpeed, hDurationms);
    }
#ifdef NULL_PTR_CHECK_MC_INT
  }
#endif
}

/**
  * @brief  Initiates a motor startup procedure
  *
  * @param  pHandle Handle on the target motor interface structure
  * @retval Returns true if the command is successfully executed;
  *         returns false otherwise
  *
  *  If the state machine of target the motor is in #IDLE state the command is
  * executed instantaneously otherwise it is discarded. Users can check
  * the return value of the function to get its status. The state of the motor
  * can be queried with the MCI_GetSTMState() function.
  *
  * Before calling MCI_StartMotor() it is mandatory to execute one of the
  * following commands, in order to set a torque or a speed reference
  * otherwise the behavior of the motor when it reaches the #RUN state will
  * be unpredictable:
  *  - MCI_ExecSpeedRamp
  *  - MCI_ExecTorqueRamp
  *  - MCI_SetCurrentReferences
  *
  * If the offsets of the current measurement circuitry offsets are not known yet,
  * an offset calibration procedure is executed to measure them prior to acutally
  * starting up the motor.
  *
  * @note The MCI_StartMotor command only triggers the execution of the start-up
  * procedure (or eventually the offset calibration procedure) and returns
  * immediately after. It is not blocking the execution of the application until
  * the motor is indeed running in steady state. If the application needs to wait
  * for the motor to be running in steady state, the application has to check the
  * state machine of the motor and verify that the #RUN state has been reached.
  * Note also that if the startup sequence fails the #RUN state may never be reached.
  */
__weak bool MCI_StartMotor(MCI_Handle_t *pHandle)
{
  bool retVal = false;
#ifdef NULL_PTR_CHECK_MC_INT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if ((IDLE == MCI_GetSTMState(pHandle)) &&
        (MC_NO_FAULTS == MCI_GetOccurredFaults(pHandle)) &&
        (MC_NO_FAULTS == MCI_GetCurrentFaults(pHandle)))
    {
      pHandle->DirectCommand = MCI_START;
      pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
      retVal = true;
    }
    else
    {
      /* Reject the command as the condition are not met */
    }
#ifdef NULL_PTR_CHECK_MC_INT
  }
#endif
  return (retVal);
}

/**
  * @brief Initiates the stop procedure for a motor
  *
  *  If the state machine is in any state but the #ICLWAIT, #IDLE, #FAULT_NOW and
  * #FAULT_OVER states, the command is immediately executed. Otherwise, it is
  * discarded. The Application can check the return value to know whether the
  * command was executed or discarded.
  *
  * @note The MCI_StopMotor() command only triggers the stop motor procedure
  * and then returns. It is not blocking the application until the motor is indeed
  * stopped. To know if it has stopped, the application can query the motor's state
  * machine and check if the #IDLE state has been reached.
  *
  * @param  pHandle Pointer on the component instance to work on.
  * @retval returns true if the command is successfully executed, false otherwise.
  */
__weak bool MCI_StopMotor(MCI_Handle_t *pHandle)
{
  bool retVal = false;
#ifdef NULL_PTR_CHECK_MC_INT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    bool status;
    MCI_State_t State;

    State = MCI_GetSTMState(pHandle);
    if ((IDLE == State) || (ICLWAIT == State))
    {
      status = false;
    }
    else
    {
      status = true;
    }

    if ((MC_NO_FAULTS == MCI_GetOccurredFaults(pHandle)) &&
        (MC_NO_FAULTS == MCI_GetCurrentFaults(pHandle)) &&
        (status == true))
    {
      pHandle->DirectCommand = MCI_STOP;
      retVal = true;
    }
    else
    {
      /* Reject the command as the condition are not met */
    }
#ifdef NULL_PTR_CHECK_MC_INT
  }
#endif
  return (retVal);
}

/**
 * @brief Acknowledges Motor Control faults that occurred on the target motor 1.
 *
 *  This function must be called before the motor can be started again when a fault
 * condition has occured. It clears the faults status and resets the state machine
 * of the target motor to the #IDLE state provided that there is no active fault
 * condition anymore.
 *
 *  If the state machine of the target motor is in the #FAULT_OVER state, the function
 * clears the list of past faults, transitions to the #IDLE state and returns true.
 * Otherwise, it oes nothing and returns false.
 *
 * @param  pHandle Pointer on the target motor drive structure.
 */
__weak bool MCI_FaultAcknowledged(MCI_Handle_t *pHandle)
{
  bool reVal = false;
#ifdef NULL_PTR_CHECK_MC_INT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if ((FAULT_OVER == MCI_GetSTMState(pHandle)) && (MC_NO_FAULTS == MCI_GetCurrentFaults(pHandle)))
    {
      pHandle->PastFaults = MC_NO_FAULTS;
      pHandle->DirectCommand = MCI_ACK_FAULTS;
      reVal = true;
    }
    else
    {
      /* Reject the command as the conditions are not met */
    }
#ifdef NULL_PTR_CHECK_MC_INT
  }
#endif
  return (reVal);
}

/**
 * @brief It clocks both HW and SW faults processing and update the state
 *        machine accordingly with hSetErrors, hResetErrors and present state.
 *        Refer to State_t description for more information about fault states.
 * @param pHandle pointer of type  STM_Handle_t
 * @param hSetErrors Bit field reporting faults currently present
 * @param hResetErrors Bit field reporting faults to be cleared
 */
__weak void MCI_FaultProcessing(MCI_Handle_t *pHandle, uint16_t hSetErrors, uint16_t hResetErrors)
{
#ifdef NULL_PTR_CHECK_MC_INT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    /* Set current errors */
    pHandle->CurrentFaults = (pHandle->CurrentFaults | hSetErrors) & (~hResetErrors);
    pHandle->PastFaults |= hSetErrors;
#ifdef NULL_PTR_CHECK_MC_INT
  }
#endif
}

/**
  * @brief  This is usually a method managed by task. It must be called
  *         periodically in order to check the status of the related pSTM object
  *         and eventually to execute the buffered command if the condition
  *         occurs.
  * @param  pHandle Pointer on the component instance to work on.
  */
__weak void MCI_ExecBufferedCommands(MCI_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_MC_INT
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if (pHandle->CommandState == MCI_COMMAND_NOT_ALREADY_EXECUTED)
    {
      bool commandHasBeenExecuted = false;
      switch (pHandle->lastCommand)
      {
        case MCI_CMD_EXECSPEEDRAMP:
        {
          SDC_SetControlMode(pHandle->pSDC, MCM_SPEED_MODE);
          pHandle->pSDC->RampRemainingStep = 0u; /* Interrupts previous ramp. */
          commandHasBeenExecuted = SDC_ExecRamp(pHandle->pSDC, pHandle->hFinalSpeed, pHandle->hDurationms);
          break;
        }

        default:
          break;
      }

      if (commandHasBeenExecuted)
      {
        pHandle->CommandState = MCI_COMMAND_EXECUTED_SUCCESSFULLY;
      }
      else
      {
        pHandle->CommandState = MCI_COMMAND_EXECUTED_UNSUCCESSFULLY;
      }
    }
#ifdef NULL_PTR_CHECK_MC_INT
  }
#endif
}

/**
  * @brief  Returns information about the state of the last buffered command.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval The state of the last buffered command
  *
  * The state returned by this function can be one of the following codes:
  * - #MCI_BUFFER_EMPTY if no buffered command has been called.
  * - #MCI_COMMAND_NOT_ALREADY_EXECUTED if the buffered command
  * condition has not already occurred.
  * - #MCI_COMMAND_EXECUTED_SUCCESSFULLY if the buffered command has
  * been executed successfully. In this case calling this function resets
  * the command state to #MCI_BUFFER_EMPTY.
  * - #MCI_COMMAND_EXECUTED_UNSUCCESSFULLY if the buffered command has
  * been executed unsuccessfully. In this case calling this function
  * resets the command state to #MCI_BUFFER_EMPTY.
  */
__weak MCI_CommandState_t MCI_IsCommandAcknowledged(MCI_Handle_t *pHandle)
{
  MCI_CommandState_t retVal;
#ifdef NULL_PTR_CHECK_MC_INT
  if (MC_NULL == pHandle)
  {
    retVal = MCI_COMMAND_EXECUTED_UNSUCCESSFULLY;
  }
  else
  {
#endif
    retVal = pHandle->CommandState;

    if ((MCI_COMMAND_EXECUTED_SUCCESSFULLY == retVal) || (MCI_COMMAND_EXECUTED_UNSUCCESSFULLY == retVal))
    {
      pHandle->CommandState = MCI_BUFFER_EMPTY;
    }
    else
    {
      /* Nothing to do */
    }
#ifdef NULL_PTR_CHECK_MC_INT
  }
#endif
  return (retVal);
}

/**
  * @brief  It returns information about the state of the related pSTM object.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval State_t It returns the current state of the related pSTM object.
  */
__weak MCI_State_t MCI_GetSTMState(MCI_Handle_t *pHandle) //cstat !MISRAC2012-Rule-8.13
{
#ifdef NULL_PTR_CHECK_MC_INT
  return ((MC_NULL == pHandle) ? FAULT_NOW : pHandle->State);
#else
  return (pHandle->State);
#endif
}

/**
  * @brief Returns the list of non-acknowledged faults that occured on the target motor
  *
  * This function returns a bitfield indicating the faults that occured since the state machine
  * of the target motor has been moved into the #FAULT_NOW state.
  *
  * Possible error codes are listed in the @ref fault_codes "Fault codes" section.
  *
  * @param  pHandle Pointer on the target motor drive structure.
  * @retval uint16_t  16 bit fields with information about the faults
  *         historically occurred since the state machine has been moved into
  */
__weak uint16_t MCI_GetOccurredFaults(MCI_Handle_t *pHandle) //cstat !MISRAC2012-Rule-8.13
{
#ifdef NULL_PTR_CHECK_MC_INT
  return ((MC_NULL == pHandle) ? MC_SW_ERROR : (uint16_t)pHandle->PastFaults);
#else
  return ((uint16_t)pHandle->PastFaults);
#endif
}

/**
  * @brief Returns the list of faults that are currently active on the target motor
  *
  * This function returns a bitfield that indicates faults that occured on the Motor
  * Control subsystem for the target motor and that are still active (the conditions
  * that triggered the faults returned are still true).
  *
  * Possible error codes are listed in the @ref fault_codes "Fault codes" section.
  *
  * @param  pHandle Pointer on the target motor drive structure.
  */
__weak uint16_t MCI_GetCurrentFaults(MCI_Handle_t *pHandle) //cstat !MISRAC2012-Rule-8.13
{
#ifdef NULL_PTR_CHECK_MC_INT
  return ((MC_NULL == pHandle) ? MC_SW_ERROR : (uint16_t)pHandle->CurrentFaults);
#else
  return ((uint16_t)pHandle->CurrentFaults);
#endif
}

/**
  * @brief Returns the lists of current and past faults that occurred on the target motor
  *
  *  This function returns two bitfields containing information about the faults currently
  * present and the faults occurred since the state machine has been moved into the #FAULT_NOW
  * state.
  *
  * These two bitfields are 16 bits wide each and are concatenated into the 32-bit data. The
  * 16 most significant bits contains the status of the current faults while that of the
  * past faults is in the 16 least significant bits.
  *
  * @sa MCI_GetOccurredFaults, MCI_GetCurrentFaults
  *
  * @param  pHandle Pointer on the target motor drive structure.
  */
__weak uint32_t MCI_GetFaultState(MCI_Handle_t *pHandle) //cstat !MISRAC2012-Rule-8.13
{
  uint32_t LocalFaultState;
#ifdef NULL_PTR_CHECK_MC_INT
  if (MC_NULL == pHandle)
  {
    LocalFaultState = MC_SW_ERROR | (MC_SW_ERROR << 16);
  }
  else
  {
#endif
    LocalFaultState = (uint32_t)(pHandle->PastFaults);
    LocalFaultState |= (uint32_t)(pHandle->CurrentFaults) << 16;
#ifdef NULL_PTR_CHECK_MC_INT
  }
#endif
  return (LocalFaultState);
}

/**
  * @brief  It returns the modality of the speed and torque controller.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval MC_ControlMode_t It returns the modality of STC. It can be one of
  *         these two values: MCM_TORQUE_MODE or MCM_SPEED_MODE.
  */
__weak MC_ControlMode_t MCI_GetControlMode(MCI_Handle_t *pHandle) //cstat !MISRAC2012-Rule-8.13
{
#ifdef NULL_PTR_CHECK_MC_INT
  return ((MC_NULL == pHandle) ? MCM_TORQUE_MODE : pHandle->LastModalitySetByUser);
#else
  return (pHandle->LastModalitySetByUser);
#endif
}

/**
  * @brief  It returns the motor direction imposed by the last command
  *         (MCI_ExecSpeedRamp, MCI_ExecTorqueRamp or MCI_SetCurrentReferences).
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t It returns 1 or -1 according the sign of hFinalSpeed,
  *         hFinalTorque or Iqdref.q of the last command.
  */
__weak int16_t MCI_GetImposedMotorDirection(MCI_Handle_t *pHandle) //cstat !MISRAC2012-Rule-8.13
{
  int16_t retVal = 1;

#ifdef NULL_PTR_CHECK_MC_INT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    switch (pHandle->lastCommand)
    {
      case MCI_CMD_EXECSPEEDRAMP:
      {
        if (pHandle->hFinalSpeed < 0)
        {
          retVal = -1;
        }
        else
        {
          /* Nothing to do */
        }
        break;
      }

      default:
        break;
    }
#ifdef NULL_PTR_CHECK_MC_INT
  }
#endif
  return (retVal);
}

/**
  * @brief  It returns information about the last ramp final speed sent by the
  *         user expressed in the unit defined by #SPEED_UNIT.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval int16_t last ramp final speed sent by the user expressed in
  *         the unit defined by #SPEED_UNIT.
  */
__weak int16_t MCI_GetLastRampFinalSpeed(MCI_Handle_t *pHandle) //cstat !MISRAC2012-Rule-8.13
{
#ifdef NULL_PTR_CHECK_MC_INT
  int16_t retVal = 0;

  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    retVal = pHandle->hFinalSpeed;
  }
  return (retVal);
#else
  return (pHandle->hFinalSpeed);
#endif
}

/**
  * @brief  It returns information about the last ramp Duration sent by the
  *         user .
  * @param  pHandle Pointer on the component instance to work on.
  * @retval uint16_t last ramp final torque sent by the user expressed in digit
  */
__weak uint16_t MCI_GetLastRampFinalDuration(MCI_Handle_t *pHandle) //cstat !MISRAC2012-Rule-8.13
{
#ifdef NULL_PTR_CHECK_MC_INT
  uint16_t retVal = 0;

  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    retVal = pHandle->hDurationms;
  }
  return (retVal);
#else
  return (pHandle->hDurationms);
#endif
}

/**
  * @brief  It returns last ramp final speed expressed in rpm.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval float_t last ramp final speed sent by the user expressed in rpm.
  */
__weak float_t MCI_GetLastRampFinalSpeed_F(MCI_Handle_t *pHandle) //cstat !MISRAC2012-Rule-8.13
{
  float_t reVal = 0.0f;

  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    reVal = (((float_t)pHandle->hFinalSpeed * (float_t)U_RPM) / (float_t)SPEED_UNIT);
  }
  return (reVal);
}

/**
  * @brief  Check if the settled speed or torque ramp has been completed.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the ramp is completed, false otherwise.
  */
__weak bool MCI_RampCompleted(MCI_Handle_t *pHandle)
{
  bool retVal = false;
#ifdef NULL_PTR_CHECK_MC_INT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if (RUN == MCI_GetSTMState(pHandle))
    {
      retVal = SDC_RampCompleted(pHandle->pSDC);
    }
    else
    {
      /* Nothing to do */
    }
#ifdef NULL_PTR_CHECK_MC_INT
  }
#endif
  return (retVal);
}

/**
  * @brief  Stop the execution of speed ramp.
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the command is executed, false otherwise.
  *
  * @deprecated This function is deprecated and should not be used anymore. It will be
  *             removed in a future version of the MCSDK. Use MCI_StopRamp() instead.
  */
__weak bool MCI_StopSpeedRamp(MCI_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_MC_INT
  return ((MC_NULL == pHandle) ? false : SDC_StopSpeedRamp(pHandle->pSDC));
#else
  return (SDC_StopSpeedRamp(pHandle->pSDC));
#endif
}

/**
  * @brief  Stop the execution of ongoing ramp.
  * @param  pHandle Pointer on the component instance to work on.
  */
__weak void MCI_StopRamp(MCI_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_MC_INT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    SDC_StopRamp(pHandle->pSDC);
#ifdef NULL_PTR_CHECK_MC_INT
  }
#endif
}

/**
  * @brief  It returns speed sensor reliability with reference to the sensor
  *         actually used for reference frame transformation
  * @param  pHandle Pointer on the component instance to work on.
  * @retval bool It returns true if the speed sensor utilized for reference
  *         frame transformation and (in speed control mode) for speed
  *         regulation is reliable, false otherwise
  */
__weak bool MCI_GetSpdSensorReliability(const MCI_Handle_t *pHandle)
{
  bool status;
#ifdef NULL_PTR_CHECK_MC_INT
  if (MC_NULL == pHandle)
  {
    status = false;
  }
  else
  {
#endif
    const SpeednPosFdbk_6S_Handle_t *SpeedSensor = SDC_GetSpeedSensor(pHandle->pSDC);
    status = SPD_Check(SpeedSensor);
#ifdef NULL_PTR_CHECK_MC_INT
  }
#endif

  return (status);
}

/**
  * @brief  Returns the last computed average mechanical speed, expressed in
  *         the unit defined by #SPEED_UNIT and related to the sensor actually
  *         used by FOC algorithm
  * @param  pHandle Pointer on the component instance to work on.
  */
__weak int16_t MCI_GetAvrgMecSpeedUnit(MCI_Handle_t *pHandle)
{
  int16_t temp_speed;
#ifdef NULL_PTR_CHECK_MC_INT
  if (MC_NULL == pHandle)
  {
    temp_speed = 0;
  }
  else
  {
#endif
    SpeednPosFdbk_6S_Handle_t * SpeedSensor = SDC_GetSpeedSensor(pHandle->pSDC);
    temp_speed = SPD_GetAvrgMecSpeedUnit(SpeedSensor);
#ifdef NULL_PTR_CHECK_MC_INT
  }
#endif
  return (temp_speed);
}

/**
  * @brief  Returns the last computed average mechanical speed, expressed in rpm
  *         and related to the sensor actually used by FOC algorithm.
  * @param  pHandle Pointer on the component instance to work on.
  */
__weak float_t MCI_GetAvrgMecSpeed_F(MCI_Handle_t *pHandle)
{
  float_t returnAvrgSpeed;
#ifdef NULL_PTR_CHECK_MC_INT
  if (MC_NULL == pHandle)
  {
    returnAvrgSpeed = 0.0f;
  }
  else
  {
#endif
    SpeednPosFdbk_6S_Handle_t * SpeedSensor = SDC_GetSpeedSensor(pHandle->pSDC);
    returnAvrgSpeed = (((float_t)SPD_GetAvrgMecSpeedUnit(SpeedSensor) * (float_t)U_RPM) / (float_t)SPEED_UNIT);
#ifdef NULL_PTR_CHECK_MC_INT
  }
#endif
  return (returnAvrgSpeed);
}

/**
  * @brief  Returns the current mechanical rotor speed reference expressed in the unit defined by #SPEED_UNIT
  *
  * @param  pHandle Pointer on the component instance to work on.
  *
  */
__weak int16_t MCI_GetMecSpeedRefUnit(MCI_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_MC_INT
  return ((MC_NULL == pHandle) ? 0 : SDC_GetMecSpeedRefUnit(pHandle->pSDC));
#else
  return (SDC_GetMecSpeedRefUnit(pHandle->pSDC));
#endif
}

/**
  * @brief  Returns the current mechanical rotor speed reference expressed in rpm.
  *
  * @param  pHandle Pointer on the component instance to work on.
  *
  */
__weak float_t MCI_GetMecSpeedRef_F(MCI_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_MC_INT
  return ((MC_NULL == pHandle) ? 0.0f :
          (((float_t)SDC_GetMecSpeedRefUnit(pHandle->pSDC) * (float_t)U_RPM) / (float_t)SPEED_UNIT));
#else
  return ((((float_t)SDC_GetMecSpeedRefUnit(pHandle->pSDC) * (float_t)U_RPM) / (float_t)SPEED_UNIT));
#endif
}

/**
  * @brief  It returns the reference eletrical torque, fed to derived class for
  *         Iqref and Idref computation
  * @param  pHandle Pointer on the component instance to work on.
  * @retval uint16_t Teref
  */
__weak uint16_t MCI_GetDutyCycleRef(const MCI_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_MC_INT
  return ((MC_NULL == pHandle) ? 0U: pHandle->pSixStepVars->DutyCycleRef);
#else
  return (pHandle->pSixStepVars->DutyCycleRef);
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

/************************ (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
