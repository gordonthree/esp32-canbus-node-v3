#include "consumer_handler.h"
#include "byte_conversion.h"
#include "storage.h"   /**< NVS API */
#include "task_twai.h" /**< TWAI API */
#include "esp_log.h"
static const char *TAG = "consumer_nvs";


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


void handleNvsConfig(can_msg_t *msg)
{
    const uint8_t modIdx = msg->data[4]; /* byte 4 holds the sub module index */

    /* skip local messages for all functions in this block */
    if (blockLocalMsg(msg))
        return;

    switch (msg->identifier)
    {

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
    }
}