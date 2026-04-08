#include "task_output.h"
#include "strobe_patterns.h"
#include "task_output_tracker.h"
#include "task_output_gpio.h"
#include "task_output_display.h"
#include "task_output_tracker.h"
#include "task_output_argb.h"

#include "esp_log.h"

extern "C" uint32_t isrGetCounter(void);

/*
 * ==========================================================================
 * Private Constants and Variables
 * =========================================================================
 */

static const char *TAG = "task_output";

/*
 * ==========================================================================
 * Private Declarations
 * =========================================================================
 */

static void TaskOutput(void *pvParameters);

/*
 * ==========================================================================
 * Private Functions
 * =========================================================================
 */

static void TaskOutput(void *pvParameters)
{
  OutputCmd_t cmd;

  ESP_LOGI(TAG, "Output task started!");

  for (;;)
  {
    /* Block until a command is received */
    if (xQueueReceive(outputTaskQueue, &cmd, portMAX_DELAY))
    {

      switch (cmd.type)
      {

      case OUTPUT_CMD_APPLY_STATE:
        gpioApplyState(cmd.index, cmd.param1);
        break;

      case OUTPUT_CMD_SET_PWM_DUTY:
        gpioApplyPwmDuty(cmd.index, cmd.param1);
        break;

      case OUTPUT_CMD_SET_PWM_FREQ:
        gpioApplyPwmFreq(cmd.index, cmd.param1);
        break;

      case OUTPUT_CMD_SET_DISPLAY_MODE:
        displayApplyMode(cmd.index, cmd.param1);
        break;

      case OUTPUT_CMD_SET_COLOR_INDEX:
        argbRequestStripColor(cmd.index, cmd.param1);
        break;

      case OUTPUT_CMD_TRACKER_ACTIVE:
        outputTrackerActive(cmd.index, cmd.param1);
        break;

      case OUTPUT_CMD_TRACKER_CFG:
        outputTrackerConfig(cmd.index, cmd.param1);
        break;

      case OUTPUT_CMD_TRACKER_RESET:
        outputTrackerReset(cmd.index);
        break;
      /* No-op commands */
      case OUTPUT_CMD_APPLY_MODE:
        break;

      default:
        ESP_LOGE(TAG, "Unknown output command type: %d", cmd.type);
        break;
      }
    }

    /* call the legacy output tracker code */
    outputTrackerTick();


    vTaskDelay(pdMS_TO_TICKS(10)); /* 100 Hz output task */
    const uint32_t isrCnt = isrGetCounter();
  }
} /* end TaskOutput */

/*
 * ==========================================================================
 * Public Functions
 * =========================================================================
 */

void startTaskOutput()
{
  /* Initialise output tracker memory */
  outputTrackerInit();

  /* Start the output handler task*/
  xTaskCreate(
      TaskOutput,
      "Task Output",
      TASK_OUTPUT_STACK_SIZE,
      NULL,
      tskLowPriority, /* idle priority plus one */
      &xOutputHandle);
}

/**
 * @brief Enqueue an output command for the output task to process
 *
 * @param type OutputCmdType_t value representing the type of output command
 * @param index uint8_t value representing the submodule index (0-255)
 * @param param1 uint32_t value representing the generic parameter for the output command
 *
 * This function creates an OutputCmd_t struct and enqueues it to the output task
 * queue for processing. The output task will process the command according to its type
 * and submodule index.
 */
void enqueueOutputCmd(OutputCmdType_t type,
                      uint8_t index,
                      uint32_t param1)
{
  OutputCmd_t cmd = {
      .type = type,
      .index = index,
      .param1 = param1};

  ESP_LOGD(TAG, "Enqueueing output command: type=%d, index=%d, param1=%d", type, index, param1);
  xQueueSend(outputTaskQueue, &cmd, 0); /* no wait */
}

/**
 * @brief Determine whether a message should be blocked by the output task
 *
 * @param msg Pointer to the CAN message to be processed
 *
 * @return true if the message should be blocked, false otherwise
 */
bool blockLocalMsg(const can_msg_t *msg)
{
  /* Only block local messages */
  if (msg->isLocal)
    return true;

  /* future message type exceptions go here*/

  /* Default: do not block messages */
  return false;
}