#pragma once

#include "Arduino.h"
#include "personality_table.h"
#include "submodule_types.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "node_state.h"
#include "canbus_project.h"
#include "freertos.h"

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

void startTaskInput();

void enqueueInputCmd(InputCmdType_t type,
                     uint8_t        index,
                     uint8_t        edge,
                     uint32_t       timestamp);