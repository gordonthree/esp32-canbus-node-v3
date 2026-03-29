#include "consumer_handler.h"
#include "esp_log.h"

static const char *TAG = "consumer_output";
static void setSwBlinkDelay(can_msg_t *msg);
static void setSwStrobePat(can_msg_t *msg);
static void setPWMDuty(can_msg_t *msg);
static void setPWMFreq(can_msg_t *msg);
static void setSwitchMode(can_msg_t *msg);
static void setSwitchState(can_msg_t *msg, uint8_t swState);

static void setSwBlinkDelay(can_msg_t *msg)
{
  const uint8_t switchID = msg->data[4]; /* switch ID */
  const uint8_t freq = msg->data[5];     /* blink delay */
  if (!nodeIsValidSubmodule(switchID))
    return;                                      /* invalid switch ID */
  subModule_t *sub = nodeGetSubModule(switchID); /* get submodule reference */

  const personalityDef_t *p =
      nodeGetPersonality(sub->personalityIndex); /* Pointer to the personality definition for this sub-module */

  sub->config.gpioOutput.param1 = freq; /* update blink delay */
  sub->runTime.valueU32 = freq;         /* update blink delay runtime */

  /* no need to queue hardware update, new freq takes effect immediately */

  ESP_LOGI(TAG, "Blink Delay: %d Switch: %d", sub->config.gpioOutput.param1, switchID);
}

static void setSwStrobePat(can_msg_t *msg)
{
  const uint8_t switchID = msg->data[4];  /* switch ID */
  const uint8_t strobePat = msg->data[5]; /* strobe pattern */

  if (!nodeIsValidSubmodule(switchID))
    return; /* invalid switch ID */

  subModule_t *sub = nodeGetSubModule(switchID); /* get submodule reference */
  sub->config.gpioOutput.param2 = strobePat;     /* update strobe pattern */
  sub->runTime.valueU32 = strobePat;             /* update strobe pattern runtime */

  /* Reset strobe hardware */
  enqueueOutputCmd(
      OUTPUT_CMD_TRACKER_RESET,
      switchID,
      (uint32_t)true /* this value doesn't matter */
  );

  ESP_LOGI(TAG, "Strobe Pattern: %d Switch: %d", sub->config.gpioOutput.param2, switchID);
}

static void setPWMDuty(can_msg_t *msg)
{
  const uint8_t switchID = msg->data[4];         /* switch ID */
  double pwmDuty = (double)(msg->data[5] * 1.0); /* pwm duty from master */

  if (!nodeIsValidSubmodule(switchID))
    return; /* invalid switch ID */

  pwmDuty = (double)(pwmDuty / 100.0);             /* convert to decimal */
  pwmDuty = (double)(pwmDuty * LEDC_13BIT_100PCT); /* convert to LEDC duty cycle */

  /* Update runtime config */
  subModule_t *sub = nodeGetSubModule(switchID);         /* get submodule reference */
  sub->config.gpioOutput.param2 = (uint8_t)msg->data[5]; /* store duty cycle percent */
  sub->runTime.valueU32 = (uint32_t)pwmDuty;             /* store duty cycle value in runtime */

  /* Queue hardware update */
  enqueueOutputCmd(
      OUTPUT_CMD_SET_PWM_DUTY,
      switchID,
      (uint32_t)pwmDuty);

  ESP_LOGI(TAG, "PWM Duty: %d Switch: %d", pwmDuty, switchID);
}

static void setPWMFreq(can_msg_t *msg)
{                                        /* 0x118 */
  const uint8_t switchID = msg->data[4]; /* switch ID */
  const uint8_t pwmFreq = msg->data[5];  /* pwm frequency index*/

  if (!nodeIsValidSubmodule(switchID))
    return; /* invalid switch ID */

  /* Calculate working frequency */
  uint32_t workingFreq = (uint32_t)(pwmFreq * PWM_SCALING_FACTOR);

  subModule_t *sub = nodeGetSubModule(switchID); /* get submodule reference */
  sub->config.gpioOutput.param1 = pwmFreq;       /* update pwm frequency index in config */
  sub->runTime.valueU32 = workingFreq;           /* store full pwm frequency in runtime */

  /* Queue hardware update */
  enqueueOutputCmd(
      OUTPUT_CMD_SET_PWM_FREQ,
      switchID,
      (uint32_t)workingFreq);

  ESP_LOGI(TAG, "PWM Frequency: %d Switch: %d", workingFreq, switchID);
}

/**
 * @brief Set the mode of a switch, does not set the state
 * @param msg The message containing the switch ID and mode
 *
 */
static void setSwitchMode(can_msg_t *msg)
{                                          /* 0x112 */
  const uint8_t switchID = msg->data[4];   /* switch ID */
  const uint8_t switchMode = msg->data[5]; /* switch mode */

  if (!nodeIsValidSubmodule(switchID))
    return; /* invalid switch ID */

  subModule_t *sub = nodeGetSubModule(switchID); /* get submodule reference */

  // TODO clearPwmHardware(switchID); /* in case it was previously PWM */

  ESP_LOGI(TAG, "Switch %d set to mode %d", switchID, switchMode);

  switch (switchMode)
  {
  /* these modes do not support output tracking */
  case OUT_MODE_BLINK:
  case OUT_MODE_PWM:
  case OUT_MODE_TOGGLE: // solid state (on/off)
    sub->config.gpioOutput.mode = switchMode;

    enqueueOutputCmd(
        OUTPUT_CMD_TRACKER_ACTIVE,
        switchID,
        (uint32_t)false);

    enqueueOutputCmd(
        OUTPUT_CMD_TRACKER_CFG,
        switchID,
        (uint32_t)false);

    break;

  /* tracker handles both strobe and momentary modes */
  case OUT_MODE_MOMENTARY: // momentary
  case OUT_MODE_STROBE:    // strobe
    sub->config.gpioOutput.mode = switchMode;
    enqueueOutputCmd(
        OUTPUT_CMD_TRACKER_CFG,
        switchID,
        (uint32_t)true); /* enable output for tracker */

    enqueueOutputCmd(
        OUTPUT_CMD_TRACKER_ACTIVE,
        switchID,
        (uint32_t)false); /* start strobing immediately */

    break;

  default:
    ESP_LOGW(TAG, "Invalid switch mode");
    break;
  }
}

/**
 * @brief Set the state of a switch based on the configured mode
 * @param msg The message containing the switch ID and state
 * @param swState The state of the switch (OUT_STATE_OFF, OUT_STATE_ON, OUT_STATE_MOMENTARY)
 */
static void setSwitchState(can_msg_t *msg, uint8_t swState)
{ /* SET_SWITCH_STATE_ID */

  const uint8_t switchID = msg->data[4]; /* switch ID */

  if (!nodeIsValidSubmodule(switchID))
    return; /* invalid switch ID */

  subModule_t *sub = nodeGetSubModule(switchID); /* get submodule reference */

  /* Pointer to the personality definition for this sub-module */
  const personalityDef_t *p =
      nodeGetPersonality(sub->personalityIndex);

  const uint8_t outPin = p->gpioPin;

  switch (swState)
  {
  case OUT_STATE_OFF: // switch off
    switch (sub->config.gpioOutput.mode)
    {
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
      ESP_LOGI(TAG, "Switch %d (pin %d) OFF", switchID, outPin);
      break;
    }
    break;

  case OUT_STATE_ON: // switch on
    switch (sub->config.gpioOutput.mode)
    {
    /* If we are in momentary mode, 'ON' should behave like a trigger */
    case OUT_MODE_MOMENTARY:
      // getOutputTracker(switchID)->nextActionTime = millis() +
      //   (MOM_SW_SCALING_FACTOR * sub->config.gpioOutput.param1);
      // digitalWrite(outPin, HIGH); /* Ensure it starts HIGH */
      // setOutput(sub, p, true);
      // getOutputTracker(switchID)->isActive = true;
      break;
    case OUT_MODE_BLINK: /* If we are in blink mode, 'ON' should behave like a trigger */
    case OUT_MODE_PWM:   /* Use LEDC hardware for blinking and pwm*/
    {
      // TODO: optimize this
      uint32_t freq = (uint32_t)(sub->config.gpioOutput.param1 * BLINK_SCALING_FACTOR);
      uint8_t pin = outPin;
      if (freq == 0)
      {           /* for debugging don't let the blink rate equal 0*/
        freq = 5; // TODO: FIX THIS NO MAGIC NUMBERS
        sub->config.gpioOutput.param1 = freq;
      }
      // TODO handleHardwarePwm(switchID, pin, freq);
      break;
    }

    default:
      // digitalWrite(outPin, HIGH); /* set output driver high */
      // setOutput(sub, p, true);
      ESP_LOGI(TAG, "Switch %d (pin %d) ON", switchID, outPin);
      break;
    }
    break;

  case OUT_STATE_MOMENTARY: // momentary press
    /* let the output task deal with this */
    ESP_LOGI(TAG, "Output Task Switch %d (pin %d) MOMENTARY (%dms)",
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
    ESP_LOGW(TAG, "Invalid switch state %d", swState);
    break;
  }
}

void handleOutputCommands(can_msg_t *msg)
{
  /* skip local messages for all functions in this block */
  if (blockLocalMsg(msg))
    return;

  switch (msg->identifier)
  {
  case SW_SET_PWM_DUTY_ID: // set output switch pwm duty
    setPWMDuty(msg);
    break;

  case SW_SET_PWM_FREQ_ID: // set output switch pwm frequency
    setPWMFreq(msg);
    break;

  case SW_SET_MODE_ID: // setup output switch modes
    setSwitchMode(msg);
    break;

  case SW_SET_OFF_ID: // set output switch off
    setSwitchState(msg, OUT_STATE_OFF);
    break;

  case SW_SET_ON_ID: // set output switch on
    setSwitchState(msg, OUT_STATE_ON);
    break;

  case SW_MOM_PRESS_ID: // set output switch momentary press
    setSwitchState(msg, OUT_STATE_MOMENTARY);
    break;

  case SW_SET_BLINK_DELAY_ID: // set output switch blink delay
    setSwBlinkDelay(msg);
    break;

  case SW_SET_STROBE_PAT_ID: // set output switch strobe pattern
    setSwStrobePat(msg);
    break;

  default:
    /* ID is in 0x110–0x1FF but not currently handled */
    ESP_LOGW(TAG, "Unknown message received 0x%x", msg->identifier);
    break;
  }
}
