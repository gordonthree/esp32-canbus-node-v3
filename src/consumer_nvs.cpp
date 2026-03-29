#include "consumer_handler.h"
#include "byte_conversion.h"
#include "storage.h"              /**< NVS API */
#include "task_twai.h"            /**< TWAI API */
#include "esp_log.h"
static const char *TAG = "consumer_nvs";

void handleNvsConfig(can_msg_t *msg)
{
    const uint8_t modIdx = msg->data[4];   /* byte 4 holds the sub module index */

    switch (msg->identifier)
    {

    case CFG_ERASE_NVS_ID: /**< Master requesting NVS erase */
        {
            ESP_LOGI(TAG, "Proccesing NVS erase request.");
            handleEraseCfgNVS();
        }
    break;

    case CFG_REBOOT_ID: /** Master requesting reboot */
    {
        ESP_LOGI(TAG, "Master requested reboot...");
        vTaskDelay(pdMS_TO_TICKS(10)); /* Short sleep before reboot */
        ESP.restart();
    }
    break;

    case CFG_WRITE_NVS_ID: /** Master requesting NVS commit */
    {
        // uint16_t masterCrc = (msg->data[4] << 8) | msg->data[5];
        const uint16_t localCrc  = crc16_ccitt((const uint8_t*)nodeGetInfo(), sizeof(nodeInfo_t));

        uint8_t responseData[6];
        /** Prepare response: [NodeID_B0..B3] [CRC_Hi] [CRC_Lo] */
        packUint32ToBytes(nodeGetInfo()->nodeID, &responseData[0]);
        responseData[4] = (uint8_t)(localCrc >> 8) & 0xFF;
        responseData[5] = (uint8_t)(localCrc & 0xFF);

        /** CRCs match, attempt to persist to flash */
        if (saveConfigNvs() == CFG_OK) {
            canEnqueueMessage(DATA_CONFIG_CRC_ID, responseData, DATA_CONFIG_CRC_DLC);
            ESP_LOGI(TAG, "NVS Commit Successful");
        } else {
            /** Flash hardware error */
            canEnqueueMessage(DATA_CFGWRITE_FAILED_ID, responseData, DATA_CFGWRITE_FAILED_DLC);
            ESP_LOGI(TAG, "NVS Commit Failed: Flash Error");
        }
    }
    break;

    case CFG_READ_NVS_ID: /**< Master requesting NVS read */
    {
        ESP_LOGI(TAG, "NVS Read Request");
        const uint16_t masterCrc = (msg->data[4] << 8) | msg->data[5];

        (void)masterCrc; /* currently unused */

        handleReadCfgNVS();

        const uint16_t localCrc  = crc16_ccitt((const uint8_t*)nodeGetInfo(), sizeof(nodeInfo_t));
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