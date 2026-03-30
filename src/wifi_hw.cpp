/**
 * @file wifi_hw.cpp
 * @brief Minimal non-blocking WiFi initialization and control.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <lwip/sockets.h>
#include <lwip/inet.h> // optional, only if you want inet_ntoa(), etc.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "freertos.h" /* task constants */
#include "task_ota.h" /* OTA task */

#include "wifi_hw.h"
#include "secrets.h"

#include "esp_log.h"

/* ==========================================================================
 *  Private variables and constants
 * ==========================================================================*/

static const char *TAG = "wifi_hw";


/* ==========================================================================
 *  Private Declarations
 * ==========================================================================*/

static void wifiInit(void);



static void wifiInit(void)
{
    ESP_LOGI(TAG, "[WIFI] Starting up.");

    WiFi.mode(WIFI_STA);

    WiFi.begin(SECRET_SSID, SECRET_PSK);

    /* Start the OTA task */
    xTaskCreate(
        TaskOTA,
        "Task OTA",
        TASK_OTA_STACK_SIZE,
        NULL,
        tskHighPriority,
        &xOTAHandle);
    // No waiting, no blocking.
}

/* ==========================================================================
 *  Public API
 * ========================================================================== */

bool wifiIsConnected(void)
{
    return (WiFi.status() == WL_CONNECTED);
}

void wifiEnable(void)
{
    /* If WiFi is already enabled, do nothing. */
    if (WiFi.getMode() != WIFI_OFF)
    {
        ESP_LOGW(TAG, "[WIFI] Already already started.");
        return;
    }

    wifiInit(); /* call the private WiFi start-up function */
}

void wifiDisable(void)
{
    ESP_LOGI(TAG, "[WIFI] Shutting down...");

    /* Stop the OTA task */
    if (xOTAHandle != NULL)
    {
        xTaskNotify(xOTAHandle, 0, eNoAction);
    }

    /* Disconnect and erase connection state. */
    WiFi.disconnect(true);

    /* Fully shut down WiFi hardware and stop internal tasks */
    WiFi.mode(WIFI_OFF);

    ESP_LOGI(TAG, "[WIFI] Disabled.");
}

/**
 * @brief Return the current WiFi IP address as a C-string.
 *
 * @details
 * WiFi.localIP().toString() returns an Arduino String, so we convert it
 * into a static C-string buffer. This buffer is overwritten on each call.
 */
const char *wifiGetIpAddress(void)
{
    static char ipBuffer[32];

    if (WiFi.status() != WL_CONNECTED)
    {
        strcpy(ipBuffer, "0.0.0.0");
        return ipBuffer;
    }

    String ip = WiFi.localIP().toString();
    strncpy(ipBuffer, ip.c_str(), sizeof(ipBuffer) - 1);
    ipBuffer[sizeof(ipBuffer) - 1] = '\0';

    return ipBuffer;
}

const int wifiGetRssi(void)
{
    return WiFi.RSSI();
}

void wifiPrintDebugInfo()
{
    const uint8_t bufSize = 96; /* 96 bytes for safety */
    char line1[bufSize];
    char line2[bufSize];

    /* First line: "[INIT] Connected to <SSID>" */
    snprintf(line1, sizeof(line1),
             "[INIT] Connected to %s",
             SECRET_SSID);

    /* Second line: "IP address: <addr>" */
    snprintf(line2, sizeof(line2),
             "IP address: %s",
             WiFi.localIP().toString().c_str());

    ESP_LOGI(TAG, "%s", line1);
    ESP_LOGI(TAG, "%s", line2);
}

