#pragma once

#include "Arduino.h"
#include <ArduinoOTA.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include "canbus_project.h"
#include "freertos.h"
#include "secrets.h"
#include "timers.h"
#include "task_twai.h" /* for twaiSetSuspended() */


/* ========================================================================== 
*  Macros and constants 
* ==========================================================================*/

/* ========================================================================== 
*  Public variables
* ==========================================================================*/

/* ========================================================================== 
*  Private functions
* ==========================================================================*/

/* ========================================================================== 
*  Public functions
* ==========================================================================*/

void TaskOTA(void *pvParameters);

// void startTaskOTA();

bool getOtaStarted();