/**
 * @file wifi_hw.h
 * @brief Public interface for non-blocking WiFi initialization and control.
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
 *  CONSTANTS 
 * ==========================================================================*/

// typedef enum {
//     WIFI_OFF,
//     WIFI_STARTING,
//     WIFI_CONNECTING,
//     WIFI_CONNECTED,
//     WIFI_ERROR
// } wifi_state_t;


/* ===========================================================================
 *  PUBLIC API FUNCTIONS 
 * ==========================================================================*/

// wifi_state_t wifiGetState();


bool wifiIsConnected(void);

/**
 * @brief Enable WiFi and begin a connection attempt.
 *
 * @note Non-blocking. Safe to call even if WiFi is already enabled.
 */
void wifiEnable(void);

/**
 * @brief Disable WiFi completely.
 *
 * @note This stops the internal WiFi task and prevents watchdog starvation.
 */
void wifiDisable(void);

/**
 * @brief Get the current WiFi IP address as a C-string.
 *
 * @return const char*  Null-terminated string containing the IP address.
 *                      Returns "0.0.0.0" if not connected.
 */
const char* wifiGetIpAddress(void);

/**
 * @brief Get the current WiFi signal strength in dBm.
 *
 * @return int  WiFi signal strength in dBm.
 */
const int wifiGetRssi(void);

/**
 * @brief Print WiFi SSID and IP address to the serial port.
 * 
 */
void wifiPrintDebugInfo();

#ifdef __cplusplus
}
#endif
