// #include "task_consumer.h"     /* Consumer task */
#include "consumer_handler.h"  /* Consumer handler */
#include "byte_conversion.h"   /* Byte conversion functions */
// #include "storage.h"
#include "task_twai.h"         /* TWAI API */
#include "task_consumer.h"     /* For sendRouteList */
#include "crc16.h"             /* CRC16 functions */
#include "can_dispatch.h"      /* for sendIntroduction*/
#include "node_state.h"        /* Node and sub-module state table */
#include "esp_log.h"
static const char *TAG = "consumer_identity";

static uint8_t introMsgPtr = 0; /* intro message pointer */

static void setEpochTime(uint32_t epochTime);

/**
 * @brief Receive time in seconds and write it to the ESP32 clock
 *
 * This function is used to receive time in seconds from the gateway node and write it to the ESP32 clock.
 * The time received is used to synchronize the ESP32 clock with the gateway node's clock.
 *
 * @param epochTime The time in seconds to be written to the ESP32 clock
 * @return None
 */
static void setEpochTime(uint32_t epochTime)
{
/* Receive time in seconds and write it to the ESP32 clock */

  struct timespec newTime;
  newTime.tv_sec = (time_t)epochTime;
  newTime.tv_nsec = 0;
  clock_settime(CLOCK_REALTIME, &newTime);

}


void handleIdentityConfig(can_msg_t *msg)
{
    const uint8_t modIdx = msg->data[4];   /* byte 4 holds the sub module index */
    ESP_LOGI(TAG, "Processing identity command 0x%03X for sub module %d", msg->identifier, modIdx);
    switch (msg->identifier)
    {
        /* 0x400 */
        case ACK_INTRO_ID:
            Serial.println("Received introduction acknowledgement, advance pointer");
            introMsgPtr++;
            if (introMsgPtr > (nodeGetInfo()->subModCnt * 2)) {
                introMsgPtr = 0;
            }
            sendIntroduction(introMsgPtr);
            break;

        /* 0x401 */
        case REQ_NODE_INTRO_ID:
            Serial.println("Interface intro request, responding with our introduction");
            introMsgPtr = 0;
            sendIntroduction(introMsgPtr);
            break;

        /* 0x40C */
        case DATA_EPOCH_ID:
        {
            uint32_t epochTime;
            epochTime = ((uint32_t)msg->data[4] << 24) |
                        ((uint32_t)msg->data[5] << 16) |
                        ((uint32_t)msg->data[6] << 8)  |
                        (uint32_t)msg->data[7];
            setEpochTime((uint32_t)epochTime);
            Serial.println("[CONSUMER] Received epoch from master; updating clock");
        }
        break;

        /* 0x42C */
        case CFG_SUB_DATA_MSG_ID:           /* setup sub module data message */
            /* no longer user configured */
            break;

        /* 0x42D */
        case CFG_SUB_INTRO_MSG_ID:          /* setup sub module intro message */
        {
            if (nodeIsValidSubmodule(modIdx) == false) {
                Serial.printf("Invalid sub module index %d\n", modIdx);
                break;
            }
            subModule_t *sub = nodeGetSubModule(modIdx);

            sub->introMsgId    = ((msg->data[5] << 8) | (msg->data[6] & 0xFF));
            sub->introMsgDLC   = msg->data[7];
            sub->submod_flags |= SUBMOD_FLAG_DIRTY;
            Serial.printf("Update Sub %d INTRO MSG: 0x%03X DLC: %d\n",
                        modIdx, sub->introMsgId, sub->introMsgDLC);
        }
        break;

        /* 0x43A */
        case CFG_ARGB_STRIP_ID: /**< setup ARGB channel */
        {
            if (nodeIsValidSubmodule(modIdx) == false) {
                Serial.printf("Invalid sub module index %d\n", modIdx);
                break;
            }
            subModule_t *sub = nodeGetSubModule(modIdx);
            sub->config.argb.reserved   = msg->data[5];
            sub->config.argb.ledCount   = msg->data[6];
            sub->config.argb.colorOrder = msg->data[7];
            sub->submod_flags |= SUBMOD_FLAG_DIRTY;
        }
        break;

        /* 0x43B */
        case CFG_DIGITAL_INPUT_ID: /**< Setup digital input channel */
        {
            if (nodeIsValidSubmodule(modIdx) == false) {
                Serial.printf("Invalid sub module index %d\n", modIdx);
                break;
            }
            subModule_t *sub = nodeGetSubModule(modIdx);

            sub->config.gpioInput.flags       = msg->data[5];
            sub->config.gpioInput.debounce_ms = msg->data[6];
            sub->config.gpioInput.reserved    = msg->data[7];
            sub->submod_flags |= SUBMOD_FLAG_DIRTY;
        }
        break;

        /* 0x43D */
        case CFG_ANALOG_INPUT_ID: /**< Setup analog ADC input channel */
        {
            if (nodeIsValidSubmodule(modIdx) == false) {
                Serial.printf("Invalid sub module index %d\n", modIdx);
                break;
            }
            subModule_t *sub = nodeGetSubModule(modIdx);

            sub->config.analogInput.overSampleFlag = msg->data[5];
            sub->config.analogInput.reserved1      = msg->data[6];
            sub->config.analogInput.reserved2      = msg->data[7];
            sub->submod_flags |= SUBMOD_FLAG_DIRTY;
        }
        break;

        /* 0x439 */
        case CFG_ANALOG_STRIP_ID: /**< Setup analog RGB/RGBW strip */
        {
            if (nodeIsValidSubmodule(modIdx) == false) {
                Serial.printf("Invalid sub module index %d\n", modIdx);
                break;
            }
            subModule_t *sub = nodeGetSubModule(modIdx);

            sub->config.analogStrip.configIndex = msg->data[5];
            sub->config.analogStrip.reserved1   = msg->data[6];
            sub->config.analogStrip.reserved2   = msg->data[7];
            sub->submod_flags |= SUBMOD_FLAG_DIRTY;
        }
        break;

        /* 0x43E */
        case CFG_ANALOG_OUTPUT_ID: /**< Setup analog DAC output channel */
        {
            if (nodeIsValidSubmodule(modIdx) == false) {
                Serial.printf("Invalid sub module index %d\n", modIdx);
                break;
            }
            subModule_t *sub = nodeGetSubModule(modIdx);

            sub->config.analogOutput.outputMode  = msg->data[5];
            sub->config.analogOutput.param1      = msg->data[6];
            sub->config.analogOutput.param2      = msg->data[7];
            sub->submod_flags |= SUBMOD_FLAG_DIRTY;
        }
        break;

        /* 0x438, 0x43F, 0x43C — grouped output configs */
        case CFG_BLINK_OUTPUT_ID: /**< Setup blinking/strobing output channel */
        case CFG_PWM_OUTPUT_ID:   /**< Setup PWM output channel */
        case CFG_DIGITAL_OUTPUT_ID: /**< Setup digital output channel (relays/mosfets) */
        {
            if (nodeIsValidSubmodule(modIdx) == false) {
                Serial.printf("Invalid sub module index %d\n", modIdx);
                break;
            }
            subModule_t *sub = nodeGetSubModule(modIdx);

            sub->config.gpioOutput.mode   = msg->data[5];
            sub->config.gpioOutput.param1 = msg->data[6];
            sub->config.gpioOutput.param2 = msg->data[7];
            sub->submod_flags |= SUBMOD_FLAG_DIRTY;
        }
        break;

        default:
            /* ID is in 0x400–0x4FF but not currently handled */
            break;

    } /* end of switch (msg->identifier) */
} /* end of handleIdentityConfig() */

void introResetSequence(void)
{
    introMsgPtr = 0; /* Reset the intro message pointer */
    // sendIntroduction(0);
}
