#include "consumer_handler.h"
#include "byte_conversion.h"
#include "storage.h"   /**< NVS API */
#include "task_twai.h" /**< TWAI API */
#include "esp_log.h"
#include "node_state.h"
#include "driver/twai.h"

static const char *TAG = "consumer_nvs";


static void setEpochTime(uint32_t epochTime);
static void dumpTwaiStatus(void);

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


void handleNvsConfig(can_msg_t *msg)
{
    const uint8_t modIdx = msg->data[4]; /* byte 4 holds the sub module index */

    /* skip local messages for all functions in this block */
    if (blockLocalMsg(msg))
        return;

    switch (msg->identifier)
    {

    case REQ_TWAI_INFO_PRT_ID: /** dump twai status */
    {
        dumpTwaiStatus();
        break;
    }
    case REQ_ROUTE_TABLE_PRT_ID: /** Master requesting route table print */
    {
        ESP_LOGI(TAG, "[CONSUMER] Received route table print request.");
        prettyPrintRoutes();
        break;
    }
    case REQ_NODEINFO_PRT_ID: /** Master requesting nodeInfo print */;
    {
        ESP_LOGI(TAG, "[CONSUMER] Received nodeInfo print request.");
        printNodeInfo(nodeGetInfo());
        break;
    }
    case DATA_ROUTE_ACK_ID: /** Node sends this message after a route is received and saved */
    {
        printf("[CONSUMER] Sending route commit ack.");
        canEnqueueMessage(DATA_ROUTE_ACK_ID, msg->data, DATA_ROUTE_ACK_DLC);
        break;
    }
    case CFG_SET_LOGLEVEL_ID: /** Master requesting log level change */
    {
        esp_log_level_t logLevel = (esp_log_level_t)msg->data[4];
        
        if (logLevel > ESP_LOG_VERBOSE) {
            logLevel = ESP_LOG_VERBOSE;
        }
        /* typedef enum {
        /* ESP_LOG_NONE,       /*!< No log output */
        /* ESP_LOG_ERROR,      /*!< Critical errors, software module can not recover on its own */
        /* ESP_LOG_WARN,       /*!< Error conditions from which recovery measures have been taken */
        /* ESP_LOG_INFO,       /*!< Information messages which describe normal flow of events */
        /* ESP_LOG_DEBUG,      /*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
        /* ESP_LOG_VERBOSE     /*!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. */
        /* } esp_log_level_t; */

        esp_log_level_set("*",logLevel);
        
        ESP_LOGI(TAG, "[CONSUMER] Proccesing log level change request, new log level is %d.", (uint8_t)logLevel);

    }
    break;

    case CFG_ERASE_NVS_ID: /**< Master requesting NVS erase */
    {
        ESP_LOGI(TAG, "[CONSUMER] Proccesing NVS erase request.");
        handleEraseCfgNVS();
    }
    break;

    case CFG_REBOOT_ID: /** Master requesting reboot */
    {
        ESP_LOGI(TAG, "[CONSUMER] Master requested reboot...");
        vTaskDelay(pdMS_TO_TICKS(10)); /* Short sleep before reboot */
        ESP.restart();
    }
    break;

    case DATA_EPOCH_ID:
    {
        uint32_t epochTime;
        epochTime = ((uint32_t)msg->data[4] << 24) |
                    ((uint32_t)msg->data[5] << 16) |
                    ((uint32_t)msg->data[6] << 8)  |
                    (uint32_t)msg->data[7];
        setEpochTime((uint32_t)epochTime);
        ESP_LOGD(TAG, "[CONSUMER] Received epoch from master; updating clock");
    }
    break;

    case CFG_WRITE_NVS_ID: /** Master requesting NVS commit */
    {
        // uint16_t masterCrc = (msg->data[4] << 8) | msg->data[5];
        const uint16_t localCrc = crc16_ccitt((const uint8_t *)nodeGetInfo(), sizeof(nodeInfo_t));

        uint8_t responseData[6];
        /** Prepare response: [NodeID_B0..B3] [CRC_Hi] [CRC_Lo] */
        packUint32ToBytes(nodeGetInfo()->nodeID, &responseData[0]);
        responseData[4] = (uint8_t)(localCrc >> 8) & 0xFF;
        responseData[5] = (uint8_t)(localCrc & 0xFF);

        /** CRCs match, attempt to persist to flash */
        if (saveConfigNvs() == CFG_OK)
        {
            canEnqueueMessage(DATA_CONFIG_CRC_ID, responseData, DATA_CONFIG_CRC_DLC);
            ESP_LOGI(TAG, "[CONSUMER] NVS commit successful");
        }
        else
        {
            /** Flash hardware error */
            canEnqueueMessage(DATA_CFGWRITE_FAILED_ID, responseData, DATA_CFGWRITE_FAILED_DLC);
            ESP_LOGI(TAG, "[CONSUMER] NVS commit failed");
        }
    }
    break;

    case CFG_READ_NVS_ID: /**< Master requesting NVS read */
    {
        ESP_LOGI(TAG, "[CONSUMER] NVS read request");
        const uint16_t masterCrc = (msg->data[4] << 8) | msg->data[5];

        (void)masterCrc; /* currently unused */

        handleReadCfgNVS();

        const uint16_t localCrc = crc16_ccitt((const uint8_t *)nodeGetInfo(), sizeof(nodeInfo_t));
        uint8_t responseData[6];

        /** Prepare response: [NodeID_B0..B3] [CRC_Hi] [CRC_Lo] */
        packUint32ToBytes(nodeGetInfo()->nodeID, &responseData[0]);
        responseData[4] = (uint8_t)(localCrc >> 8) & 0xFF;
        responseData[5] = (uint8_t)(localCrc & 0xFF);

        canEnqueueMessage(DATA_CONFIG_CRC_ID, responseData, DATA_CONFIG_CRC_DLC);
    }
    break;

    default:
    {
        ESP_LOGW(TAG, "[CONSUMER] Unknown message ID: 0x%03X", msg->identifier);
    }
    break;
    }
}


/**
 * @brief Pretty-print the TWAI controller status using ESP_LOGD.
 *
 * Safe to call from any task context. Not ISR-safe.
 */
static void dumpTwaiStatus(void)
{
    twai_status_info_t st;
    esp_err_t err = twai_get_status_info(&st);

    if (err != ESP_OK) {
        ESP_LOGD(TAG, "[TWAI] twai_get_status_info failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGD(TAG,
        "[TWAI STATUS]\n"
        "  state:            %d\n"
        "  msgs_to_tx:       %u\n"
        "  msgs_to_rx:       %u\n"
        "  tx_error_counter: %u\n"
        "  rx_error_counter: %u\n"
        "  tx_failed_count:  %u\n"
        "  rx_missed_count:  %u\n"
        "  arb_lost_count:   %u\n"
        "  bus_error_count:  %u",
        st.state,
        st.msgs_to_tx,
        st.msgs_to_rx,
        st.tx_error_counter,
        st.rx_error_counter,
        st.tx_failed_count,
        st.rx_missed_count,
        st.arb_lost_count,
        st.bus_error_count
    );
}
