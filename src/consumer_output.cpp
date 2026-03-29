#include "consumer_handler.h"


static void setSwBlinkDelay(can_msg_t *msg)
{
  const uint8_t switchID = msg->data[4];               /* switch ID */
  const uint8_t freq     = msg->data[5];               /* blink delay */
  if (!nodeIsValidSubmodule(switchID)) return;         /* invalid switch ID */
  subModule_t *sub = nodeGetSubModule(switchID);       /* get submodule reference */

  const personalityDef_t* p =
      nodeGetPersonality(sub->personalityIndex);       /* Pointer to the personality definition for this sub-module */

  sub->config.gpioOutput.param1 = freq;                /* update blink delay */
  sub->runTime.valueU32         = freq;                /* update blink delay runtime */

    /* Queue hardware update */
  enqueueOutputCmd(
      OUTPUT_CMD_SET_BLINK_RATE,
      switchID,
      (uint32_t)freq
  );  

  Serial.printf("Blink Delay: %d Switch: %d\n", sub->config.gpioOutput.param1, switchID);
}

static void setSwStrobePat(can_msg_t *msg) {
  const uint8_t switchID =  msg->data[4];              /* switch ID */
  const uint8_t strobePat = msg->data[5];              /* strobe pattern */

  if (!nodeIsValidSubmodule(switchID)) return;         /* invalid switch ID */

  subModule_t *sub = nodeGetSubModule(switchID);       /* get submodule reference */
  sub->config.gpioOutput.param2 = strobePat;           /* update strobe pattern */
  sub->runTime.valueU32         = strobePat;           /* update strobe pattern runtime */  
  /* Queue hardware update */
  enqueueOutputCmd(
      OUTPUT_CMD_SET_STROBE_PAT,
      switchID,
      (uint32_t)strobePat
  );  

  Serial.printf("Strobe Pattern: %d Switch: %d\n", sub->config.gpioOutput.param2, switchID);
}


static void setPWMDuty(can_msg_t *msg)
{ 
  const uint8_t switchID = msg->data[4];                   /* switch ID */
  double pwmDuty         = (double)(msg->data[5] * 1.0);   /* pwm duty from master */

  if (!nodeIsValidSubmodule(switchID)) return;             /* invalid switch ID */

  pwmDuty = (double)(pwmDuty / 100.0);                     /* convert to decimal */
  pwmDuty = (double)(pwmDuty * LEDC_13BIT_100PCT);         /* convert to LEDC duty cycle */

  /* Update runtime config */
  subModule_t *sub = nodeGetSubModule(switchID);           /* get submodule reference */
  sub->config.gpioOutput.param2 = (uint8_t)msg->data[5];   /* store duty cycle percent */
  sub->runTime.valueU32         = (uint32_t)pwmDuty;       /* store duty cycle value in runtime */ 

  /* Queue hardware update */
  enqueueOutputCmd(
      OUTPUT_CMD_SET_PWM_DUTY,
      switchID,
      (uint32_t)pwmDuty
  );  
                    
  Serial.printf("PWM Duty: %d Switch: %d\n", pwmDuty, switchID);
}

static void setPWMFreq(can_msg_t *msg)
{ /* 0x118 */
  const uint8_t switchID = msg->data[4];         /* switch ID */
  const uint8_t pwmFreq  = msg->data[5];         /* pwm frequency index*/

  if (!nodeIsValidSubmodule(switchID)) return;   /* invalid switch ID */

  /* Calculate working frequency */
  uint32_t workingFreq = (uint32_t)(pwmFreq * PWM_SCALING_FACTOR);

  subModule_t *sub = nodeGetSubModule(switchID); /* get submodule reference */
  sub->config.gpioOutput.param1 = pwmFreq;       /* update pwm frequency index in config */
  sub->runTime.valueU32         = workingFreq;   /* store full pwm frequency in runtime */

  /* Queue hardware update */
  enqueueOutputCmd(
      OUTPUT_CMD_SET_PWM_FREQ,
      switchID,
      (uint32_t)workingFreq
  );  

  Serial.printf("PWM Frequency: %d Switch: %d\n", workingFreq, switchID);
}


/**
 * @brief Set the mode of a switch, does not set the state
 * @param msg The message containing the switch ID and mode
 *
*/
// NOTE: This function still uses the legacy tracker + direct mode-handling path.
//       Switch mode is a behavioral configuration that affects how momentary,
//       strobe, toggle, and PWM outputs behave, but it does NOT map cleanly
//       onto the new queue-driven architecture yet.
//
//       Refactoring this properly requires:
//         • moving all tracker state transitions into task_output
//         • centralizing PWM/GPIO backend switching in task_output
//         • removing all direct tracker mutations from consumer code
//         • defining a dedicated OUTPUT_CMD_* for mode changes
//
//       This is a larger architectural change and will be done AFTER
//       task_output is fully wired up and stable. For now, this function
//       remains in its pre-queue form so the existing behavior continues
//       to work without triggering a full tracker rewrite.

static void setSwitchMode(can_msg_t *msg) { /* 0x112 */
  const uint8_t switchID = msg->data[4];    /* switch ID */
  const uint8_t switchMode = msg->data[5];  /* switch mode */

  if (!nodeIsValidSubmodule(switchID)) return;         /* invalid switch ID */

  subModule_t *sub = nodeGetSubModule(switchID); /* get submodule reference */

  // TODO clearPwmHardware(switchID); /* in case it was previously PWM */

  Serial.printf("Switch %d set to mode %d\n", switchID, switchMode);

  switch (switchMode) {
    /* these modes do not support output tracking */
    case OUT_MODE_BLINK:
    case OUT_MODE_PWM:     
    case OUT_MODE_TOGGLE: // solid state (on/off)
      sub->config.gpioOutput.mode = switchMode;
      
      enqueueOutputCmd(
          OUTPUT_CMD_TRACKER_ACTIVE,
          switchID,
          (uint32_t)false
      );  

      enqueueOutputCmd(
          OUTPUT_CMD_TRACKER_CFG,
          switchID,
          (uint32_t)false
      );  

      break;

    /* tracker handles both strobe and momentary modes */
    case OUT_MODE_MOMENTARY: // momentary
    case OUT_MODE_STROBE: // strobe
      sub->config.gpioOutput.mode = switchMode;
      enqueueOutputCmd(
          OUTPUT_CMD_TRACKER_CFG,
          switchID,
          (uint32_t)true
      ); /* enable output for tracker */

      enqueueOutputCmd(
          OUTPUT_CMD_TRACKER_ACTIVE,
          switchID,
          (uint32_t)false
      ); /* start strobing immediately */

      break;


    default:
      Serial.println("Invalid switch mode");
      break;
  }

}

/**
 * @brief Set the state of a switch based on the configured mode
 * @param msg The message containing the switch ID and state
 * @param swState The state of the switch (OUT_STATE_OFF, OUT_STATE_ON, OUT_STATE_MOMENTARY)
 */
// NOTE: This function still uses the legacy outputTracker + direct hardware
//       control path. Unlike simple configuration consumers, setSwitchState()
//       attempts to drive the *actual output behavior* immediately, including:
//
//         • momentary timing (nextActionTime, isActive)
//         • strobe/blink state machine setup
//         • PWM backend activation/deactivation
//         • direct GPIO writes via setOutput()
//         • mode-dependent branching for ON/OFF/MOMENTARY
//
//       All of this predates the new queue-driven task_output architecture.
//       In the new design, the consumer layer should NOT:
//
//         • mutate tracker state directly
//         • perform hardware actions (GPIO, PWM, timers)
//         • branch on behavioral modes
//         • schedule momentary/strobe timing
//
//       A proper refactor requires:
//         • moving all momentary/strobe/blink logic into task_output
//         • centralizing PWM/GPIO backend switching in task_output
//         • defining OUTPUT_CMD_* messages for ON/OFF/MOMENTARY transitions
//         • letting tracker live entirely inside task_output as the runtime
//           behavior engine
//
//       This is a large, multi-module change and depends on task_output being
//       fully implemented first. For now, this function remains in its legacy
//       form so existing behavior continues to work without forcing a full
//       tracker rewrite during the current queue migration.

static void setSwitchState(can_msg_t *msg, uint8_t swState)
{ /* SET_SWITCH_STATE_ID */

  const uint8_t switchID = msg->data[4]; /* switch ID */

  if (!nodeIsValidSubmodule(switchID)) return;         /* invalid switch ID */

  subModule_t *sub = nodeGetSubModule(switchID); /* get submodule reference */

  /* Pointer to the personality definition for this sub-module */
  const personalityDef_t* p =
      nodeGetPersonality(sub->personalityIndex);

  const uint8_t outPin = p->gpioPin;

  switch (swState) {
    case OUT_STATE_OFF: // switch off
      switch (sub->config.gpioOutput.mode) {
        case OUT_MODE_MOMENTARY:
          // getOutputTracker(switchID)->isActive = false;
          break;
        case OUT_MODE_STROBE:
          // getOutputTracker(switchID)->isActive = false;
          break;
        case OUT_MODE_BLINK:
        case OUT_MODE_PWM: // pwm and blinking
          // TODO stopHardwarePwm(switchID);
          // TODO clearPwmHardware(switchID);
          break;
        default:
          // digitalWrite(outPin, LOW); /* set output driver low */
          // setOutput(sub, p, false);
          Serial.printf("Switch %d (pin %d) OFF\n", switchID, outPin);
          break;
      }
      break;

    case OUT_STATE_ON: // switch on
      switch (sub->config.gpioOutput.mode) {
        /* If we are in momentary mode, 'ON' should behave like a trigger */
        case OUT_MODE_MOMENTARY:
          // getOutputTracker(switchID)->nextActionTime = millis() +
          //   (MOM_SW_SCALING_FACTOR * sub->config.gpioOutput.param1);
          // digitalWrite(outPin, HIGH); /* Ensure it starts HIGH */
          // setOutput(sub, p, true);
          // getOutputTracker(switchID)->isActive = true;
          break;
        case OUT_MODE_BLINK: /* If we are in blink mode, 'ON' should behave like a trigger */
        case OUT_MODE_PWM: /* Use LEDC hardware for blinking and pwm*/
        {
          //TODO: optimize this
          uint32_t freq = (uint32_t)(sub->config.gpioOutput.param1 * BLINK_SCALING_FACTOR);
          uint8_t  pin  = outPin;
          if (freq == 0) { /* for debugging don't let the blink rate equal 0*/
            freq = 5; //TODO: FIX THIS NO MAGIC NUMBERS
            sub->config.gpioOutput.param1 = freq;
          }
          // TODO handleHardwarePwm(switchID, pin, freq);
          break;
        }

        default:
          // digitalWrite(outPin, HIGH); /* set output driver high */
          // setOutput(sub, p, true);
          Serial.printf("Switch %d (pin %d) ON\n", switchID, outPin);
          break;
      }
      break;

    case OUT_STATE_MOMENTARY: // momentary press
      /* let the output task deal with this */
      Serial.printf("Output Task Switch %d (pin %d) MOMENTARY (%dms)\n",
                    switchID, outPin,
                    (MOM_SW_SCALING_FACTOR * sub->config.gpioOutput.param1));

      // getOutputTracker(switchID)->nextActionTime = millis() +
      //               (MOM_SW_SCALING_FACTOR * sub->config.gpioOutput.param1);
      // digitalWrite(outPin, HIGH); /* Ensure it starts HIGH */
      // setOutput(sub, p, true);
      // getOutputTracker(switchID)->isActive = true; /* enable momentary timer */
      // getOutputTracker(switchID)->isConfigured = true; /* enable this output for nonblocking control*/
      break;

    default:
      Serial.println("Invalid switch state");
      break;
  }
}


void handleOutputCommands(can_msg_t *msg)
{
    switch (msg->identifier)
    {
        case SW_SET_PWM_DUTY_ID:      // set output switch pwm duty
            setPWMDuty(msg);
            break;

        case SW_SET_PWM_FREQ_ID:      // set output switch pwm frequency
            setPWMFreq(msg);
            break;

        case SW_SET_MODE_ID:          // setup output switch modes
            setSwitchMode(msg);
            break;

        case SW_SET_OFF_ID:           // set output switch off
            setSwitchState(msg, OUT_STATE_OFF);
            break;

        case SW_SET_ON_ID:            // set output switch on
            setSwitchState(msg, OUT_STATE_ON);
            break;

        case SW_MOM_PRESS_ID:         // set output switch momentary press
            setSwitchState(msg, OUT_STATE_MOMENTARY);
            break;

        case SW_SET_BLINK_DELAY_ID:   // set output switch blink delay
            setSwBlinkDelay(msg);
            break;

        case SW_SET_STROBE_PAT_ID:    // set output switch strobe pattern
            setSwStrobePat(msg);
            break;

        default:
            /* ID is in 0x110–0x1FF but not currently handled */
            break;
    }
}
