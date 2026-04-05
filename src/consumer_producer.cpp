#include "consumer_handler.h"
#include "canbus_msg.h"
#include "can_producer.h"
#include "node_state.h" /* Node and sub-module */
#include "esp_log.h"

static const char *TAG = "consumer_producer";

void handleProducerConfig(can_msg_t *msg)
{
    const uint8_t modIdx = msg->data[4]; /* byte 4 holds the sub module index */
    /* skip local messages for all functions in this block */
    if (blockLocalMsg(msg))
        return;

    switch (msg->identifier)
    {
    case CFG_PRODUCER_CFG_ID: /**< Configure a single producer submodule */
        if (SUBMODULE_INDEX_INVALID(modIdx))
        {
            ESP_LOGI(TAG, "Invalid sub module index %d", modIdx);
            break;
        }
        handleProducerCfg(msg);
        break;

    case CFG_PRODUCER_WRITE_NVS_ID: /**< Commit producer config to NVS. */
        if (SUBMODULE_INDEX_INVALID(modIdx))
        {
            ESP_LOGI(TAG, "Invalid sub module index %d", modIdx);
            break;
        }
        requestProducerSave();
        break;

    case CFG_PRODUCER_READ_NVS_ID: /**< Request producer config for all submodules */
        if (SUBMODULE_INDEX_INVALID(modIdx))
        {
            ESP_LOGI(TAG, "Invalid sub module index %d", modIdx);
            break;
        }
        requestProducerLoad();
        break;

    case REQ_PRODUCER_CFG_ID: /**< Request producer config  for idx */
        if (SUBMODULE_INDEX_INVALID(modIdx))
        {
            ESP_LOGI(TAG, "Invalid sub module index %d", modIdx);
            break;
        }
        // TODO: IMPLEMENT
        break;

    case CFG_PRODUCER_PURGE_ID: /**< Purge the producer list */
        if (SUBMODULE_INDEX_INVALID(modIdx))
        {
            ESP_LOGI(TAG, "Invalid sub module index %d", modIdx);
            break;
        }
        producerPurgeSingle(modIdx);
        break;

    case CFG_PRODUCER_DEFAULTS_ID: /**< Reset the producer at idx to defaults */
        if (SUBMODULE_INDEX_INVALID(modIdx))
        {
            ESP_LOGI(TAG, "Invalid sub module index %d", modIdx);
            break;
        }
        producerDefaultSingle(modIdx);
        break;

    case CFG_PRODUCER_ENABLE_ID: /**< Enable the producer at idx */
    {
        if (SUBMODULE_INDEX_INVALID(modIdx))
        {
            ESP_LOGI(TAG, "Invalid sub module index %d", modIdx);
            break;
        }
        const subModule_t *sub = nodeGetSubModule(modIdx);
        const personalityDef_t *p = nodeGetPersonality(sub->personalityIndex);
        if (!p)
        {
            ESP_LOGI(TAG, "Personality not found for index %d", modIdx);
            break;
        }
        producerEnable(modIdx); /* Enable the producer */
        // TODO enableDigitalInputISR(p->gpioPin); /* Enable the digital input interrupt */
        break;
    }

    case CFG_PRODUCER_DISABLE_ID: /**< Disable the producer at idx */
    {
        if (SUBMODULE_INDEX_INVALID(modIdx))
        {
            ESP_LOGI(TAG, "Invalid sub module index %d", modIdx);
            break;
        }
        const subModule_t *sub = nodeGetSubModule(modIdx);
        const personalityDef_t *p = nodeGetPersonality(sub->personalityIndex);
        if (!p)
        {
            ESP_LOGI(TAG, "Personality not found for index %d", modIdx);
            break;
        }
        producerDisable(modIdx); /* Disable the producer */
        // TODO disableDigitalInputISR(p->gpioPin); /* Disable the digital input interrupt */
        break;
    }

    case CFG_PRODUCER_TOGGLE_ID: /**< Toggle operation of producer at idx */
        // Not implemented
        break;

    case REQ_PRODUCER_LIST_ID: /**< Ask the node to dump the entire producer cfg list */
        if (SUBMODULE_INDEX_INVALID(modIdx))
        {
            ESP_LOGI(TAG, "Invalid sub module index %d", modIdx);
            break;
        }
        break;

    case PRODUCER_LIST_BEGIN_ID: /**< Node will announce the count of defined producers */
        if (SUBMODULE_INDEX_INVALID(modIdx))
        {
            ESP_LOGI(TAG, "Invalid sub module index %d", modIdx);
            break;
        }
        break;

    case PRODUCER_LIST_DATA_ID: /**< Producer cfg data for index */
        if (SUBMODULE_INDEX_INVALID(modIdx))
        {
            ESP_LOGI(TAG, "Invalid sub module index %d", modIdx);
            break;
        }
        break;

    case PRODUCER_LIST_END_ID: /**< Node will announce the end of the defined producers list */
        if (SUBMODULE_INDEX_INVALID(modIdx))
        {
            ESP_LOGI(TAG, "Invalid sub module index %d", modIdx);
            break;
        }
        break;

    default:
        /* ID is in 0x320–0x3FF but not currently handled */
        ESP_LOGW(TAG, "Invalid producer command %d", msg->identifier);
        break;
    }
}
