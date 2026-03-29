/**
 * @file wifi_hw.cpp
 * @brief Minimal non-blocking WiFi initialization and control.
 */
#include <Arduino.h>
#include <WiFi.h>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "freertos.h"           /* task constants */
#include "task_ota.h"           /* OTA task */

#include "wifi_hw.h"
#include "secrets.h"

/* ==========================================================================
 *  Private functions
 * ==========================================================================*/

static void wifiInit(void)
{
    Serial.println("[WiFi] Starting up.");

    WiFi.mode(WIFI_STA);
    WiFi.begin(SECRET_SSID, SECRET_PSK);

    /* Start the OTA task */
    xTaskCreate(
      TaskOTA,
      "Task OTA",
      TASK_OTA_STACK_SIZE,
      NULL,
      tskHighPriority,
      &xOTAHandle
    );
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
    // Serial.println("[WiFi] Enabling...");

    /* If WiFi is already enabled, do nothing. */
    if (WiFi.getMode() != WIFI_OFF) {
        Serial.println("[WIFI] Already already started.");
        return;
    }

    wifiInit(); /* call the private WiFi start-up function */

}

void wifiDisable(void)
{
    Serial.println("[WIFI] Shutting down...");

    /* Stop the OTA task */
    if (xOTAHandle != NULL) {
        xTaskNotify(xOTAHandle, 0, eNoAction);
    }

    /* Disconnect and erase connection state. */
    WiFi.disconnect(true);

    /* Fully shut down WiFi hardware and stop internal tasks */
    WiFi.mode(WIFI_OFF);

    Serial.println("[WIFI] Disabled.");
}

/**
 * @brief Return the current WiFi IP address as a C-string.
 *
 * @details
 * WiFi.localIP().toString() returns an Arduino String, so we convert it
 * into a static C-string buffer. This buffer is overwritten on each call.
 */
const char* wifiGetIpAddress(void)
{
    static char ipBuffer[32];

    if (WiFi.status() != WL_CONNECTED) {
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
  Serial.print("[INIT] Connected to ");
  Serial.println(SECRET_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}