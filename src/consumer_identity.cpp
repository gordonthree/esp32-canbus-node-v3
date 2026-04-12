// #include "task_consumer.h"     /* Consumer task */
#include "byte_conversion.h"  /* Byte conversion functions */
#include "consumer_handler.h" /* Consumer handler */
// #include "storage.h"
#include "can_dispatch.h" /* for sendIntroduction*/
#include "crc16.h"        /* CRC16 functions */
#include "esp_log.h"
#include "isr_gpio.h"      /* for attachDigitalInputISR() */
#include "node_state.h"    /* Node and sub-module state table */
#include "task_consumer.h" /* For sendRouteList */
#include "task_twai.h"     /* TWAI API */

static const char *TAG = "consumer_identity";

static uint8_t introMsgPtr = 0; /* intro message pointer */

void handleIdentityConfig(can_msg_t *msg) {
  /* skip local messages for all functions in this block */
  if (blockLocalMsg(msg))
    return;

  const uint8_t modIdx = msg->data[4]; /* byte 4 holds the sub module index */
  ESP_LOGI(TAG, "Processing identity command 0x%03X for sub module %d",
           msg->identifier, modIdx);

  /* get submodule pointer */
  subModule_t *sub = nodeGetActiveSubModule(modIdx);

  /* get personality pointer */
  const personalityDef_t *p = nodeGetActivePersonality(sub->personalityIndex);

  switch (msg->identifier) {
    /* 0x400 */
    case ACK_INTRO_ID:
      {
        const uint8_t subModCnt = nodeGetActiveSubModuleCount();
        ESP_LOGD(TAG, "Received introduction acknowledgement, advance pointer");
        introMsgPtr++;
        if (introMsgPtr >= (subModCnt * 2)) {
          /* we've completed the intro sequence, wrap the pointer */
          introMsgPtr = 0;
        }
        sendIntroduction(introMsgPtr);
      }
      break;

    /* 0x401 */
    case REQ_NODE_INTRO_ID:
      {
        ESP_LOGD(TAG,
                 "Interface intro request, responding with our introduction");
        introMsgPtr = 0;
        sendIntroduction(introMsgPtr);
      }
      break;

    case CFG_SUB_FLAG_ADD_ID: /* add flag to submod_flags */
      {

        sub->submod_flags |= msg->data[5]; /* add flag to submod_flags */
        sub->submod_flags |= SUBMOD_FLAG_DIRTY;
        ESP_LOGI(TAG, "Update Sub %d FLAGS: 0x%03X", modIdx, sub->submod_flags);
      }
      break;

    case CFG_SUB_FLAG_REMOVE_ID: /* remove flag from submod_flags */
      {

        sub->submod_flags &= ~msg->data[5]; /* remove flag from submod_flags */
        sub->submod_flags |= SUBMOD_FLAG_DIRTY;
        ESP_LOGI(TAG, "Update Sub %d FLAGS: 0x%03X", modIdx, sub->submod_flags);
      }
      break;

    case CFG_SUB_DATA_MSG_ID: /* setup sub module data message */
      /* no longer user configured */
      break;

    /* 0x42D */
    case CFG_SUB_INTRO_MSG_ID: /* setup sub module intro message */
      {

        sub->introMsgId = ((msg->data[5] << 8) | (msg->data[6] & 0xFF));
        sub->introMsgDLC = msg->data[7];
        sub->submod_flags |= SUBMOD_FLAG_DIRTY;
        ESP_LOGI(TAG, "Update Sub %d INTRO MSG: 0x%03X DLC: %d", modIdx,
                 sub->introMsgId, sub->introMsgDLC);
      }
      break;

    /* 0x43A */
    case CFG_ARGB_STRIP_ID: /**< setup ARGB channel */
      {

        sub->config.argb.reserved = msg->data[5];
        sub->config.argb.ledCount = msg->data[6];
        sub->config.argb.colorOrder = msg->data[7];
        sub->submod_flags |= SUBMOD_FLAG_DIRTY;
      }
      break;

    /* 0x43B */
    case CFG_DIGITAL_INPUT_ID: /**< Setup digital input channel */
      {
        uint8_t oldFlags = sub->config.gpioInput.flags;
        uint8_t newFlags = msg->data[5];

        // Mask out reserved bits so they don't trigger reinstall
        uint8_t relevantOld = oldFlags & ~INPUT_FLAG_MASK_RESERVED;
        uint8_t relevantNew = newFlags & ~INPUT_FLAG_MASK_RESERVED;

        bool needsReattach = (relevantOld != relevantNew);

        sub->config.gpioInput.flags = newFlags;
        sub->config.gpioInput.debounce_ms = msg->data[6];
        sub->config.gpioInput.reserved = msg->data[7];
        sub->submod_flags |= SUBMOD_FLAG_DIRTY;

        const personalityDef_t *p =
            nodeGetActivePersonality(sub->personalityIndex);

        if (needsReattach) {
          attachDigitalInputISR(p->gpioPin, modIdx);
        }
      }
      break;

    /* 0x43D */
    case CFG_ANALOG_INPUT_ID: /**< Setup analog ADC input channel */
      {

        sub->config.analogInput.overSampleFlag = msg->data[5];
        sub->config.analogInput.reserved1 = msg->data[6];
        sub->config.analogInput.reserved2 = msg->data[7];
        sub->submod_flags |= SUBMOD_FLAG_DIRTY;
      }
      break;

    /* 0x439 */
    case CFG_ANALOG_STRIP_ID: /**< Setup analog RGB/RGBW strip */
      {

        sub->config.analogStrip.configIndex = msg->data[5];
        sub->config.analogStrip.reserved1 = msg->data[6];
        sub->config.analogStrip.reserved2 = msg->data[7];
        sub->submod_flags |= SUBMOD_FLAG_DIRTY;
      }
      break;

    /* 0x43E */
    case CFG_ANALOG_OUTPUT_ID: /**< Setup analog DAC output channel */
      {

        sub->config.analogOutput.outputMode = msg->data[5];
        sub->config.analogOutput.param1 = msg->data[6];
        sub->config.analogOutput.param2 = msg->data[7];
        sub->submod_flags |= SUBMOD_FLAG_DIRTY;
      }
      break;

    /* 0x438, 0x43F, 0x43C — grouped output configs */
    case CFG_BLINK_OUTPUT_ID:   /**< Setup blinking/strobing output channel */
    case CFG_PWM_OUTPUT_ID:     /**< Setup PWM output channel */
    case CFG_DIGITAL_OUTPUT_ID: /**< Setup digital output channel
                                   (relays/mosfets) */
      {

        sub->config.gpioOutput.mode = msg->data[5];
        sub->config.gpioOutput.param1 = msg->data[6];
        sub->config.gpioOutput.param2 = msg->data[7];
        sub->submod_flags |= SUBMOD_FLAG_DIRTY;
      }
      break;

    default:
      {
        ESP_LOGI(TAG, "Unknown command 0x%03X", msg->identifier);
      }
      /* ID is in 0x400–0x4FF but not currently handled */
      break;

  } /* end of switch (msg->identifier) */
} /* end of handleIdentityConfig() */

void introResetSequence(void) {
  introMsgPtr = 0; /* Reset the intro message pointer */
                   // sendIntroduction(0);
}
