#include "task_ota.h"
#include "wifi_hw.h"

/* ========================================================================== 
*  Private variables 
* ==========================================================================*/
bool ota_started                  = false;
const char* otaHostname           = "ESPOTA";
unsigned long ota_progress_millis = 0;

/* ========================================================================== 
*  Public variables
* ==========================================================================*/

/* ========================================================================== 
*  Private functions
* ==========================================================================*/


void TaskOTA(void *pvParameters) {

  /* Wait until WiFi is connected */
  while (!wifiIsConnected()) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  Serial.println("[RTOS] OTA task started.");

  /* Configure ArduinoOTA */
  ArduinoOTA.setHostname(otaHostname);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    twaiSetSuspended(true); /* Stop the task logic */

    /* Suspend tasks that might be using hardware */
    if (xTWAIHandle != NULL)   vTaskSuspend(xTWAIHandle);   /* suspend the TWAI task */
    if (xOutputHandle != NULL) vTaskSuspend(xOutputHandle); /* suspend the output switch task */
    if (xInputHandle != NULL)  vTaskSuspend(xInputHandle);  /* suspend the input event task */
    if (xProducerHandle != NULL) vTaskSuspend(xProducerHandle); /* suspend the producer task */
    if (xConsumerHandle != NULL) vTaskSuspend(xConsumerHandle); /* suspend the consumer task */
#ifdef ESP32CYD
    if (xDisplayHandle != NULL) vTaskSuspend(xDisplayHandle); /* suspend the display task */
    if (xTouchHandle != NULL) vTaskSuspend(xTouchHandle); /* suspend the touch task */
#endif
    /* Stop the TWAI driver */
    // twai_stop();

    Serial.println("OTA Start: Background tasks suspended, starting flash... ");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.print(".");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]\n", error);
    ESP.restart(); /* Cleanest way to recover CAN hardware after a failed OTA */
  });

  /* Option A: Always enable OTA listener */
  ArduinoOTA.begin();
  ota_started = true;
  Serial.println("[OTA] OTA library ready");

  /* Main loop: handle OTA requests */
  for (;;) {
    /* Check for shutdown request  */
    if (xTaskNotifyWait(0, 0, NULL, 0) == pdTRUE) {
      ArduinoOTA.end();  /* Unload OTA library */
      vTaskDelete(NULL); /* Delete OTA task */
    }
    ArduinoOTA.handle();
    vTaskDelay(pdMS_TO_TICKS(10));    
  }

  vTaskDelete(NULL);
}


/* ========================================================================== 
*  Public functions
* ==========================================================================*/

bool getOtaStarted() { return ota_started; }

// void startTaskOTA() 
// {
//   /* Start OTA task  */
//   xTaskCreate(
//     TaskOTA,
//     "Task OTA",
//     TASK_OTA_STACK_SIZE,
//     NULL,
//     tskHighPriority,
//     NULL
//   );
// }