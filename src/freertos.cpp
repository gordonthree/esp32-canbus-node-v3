#include "freertos.h"
#include "can_platform.h"  /* brings in can_msg_t */
#include "driver/twai.h"   /* esp32 native TWAI CAN library */

#include "esp_log.h"

static const char* TAG = "freertos";


/* ========================================================================== 
*  Public variables
* ==========================================================================*/

/* Define Task handles */
TaskHandle_t xTWAIHandle     = NULL;
TaskHandle_t xInputHandle    = NULL;
TaskHandle_t xOutputHandle   = NULL;
TaskHandle_t xOTAHandle      = NULL;
TaskHandle_t xConsumerHandle = NULL;
TaskHandle_t xProducerHandle = NULL;

SemaphoreHandle_t flashMutex = xSemaphoreCreateMutex(); 

#ifdef ESP32CYD
TaskHandle_t xDisplayHandle = NULL;   /**< Handle for the display task */
TaskHandle_t xTouchHandle   = NULL;   /**< Handle for the touch task */
#endif

/* ========================================================================== 
 *  Message Queue Definitions
 * ==========================================================================*/

 /* Main firmware message queues */
QueueHandle_t canMsgTxQueue     = NULL;
QueueHandle_t canMsgRxQueue     = NULL;
QueueHandle_t outputTaskQueue   = NULL;
QueueHandle_t inputTaskQueue    = NULL;
QueueHandle_t gpioEventQueue    = NULL;

/* CYD message queues */
QueueHandle_t cydDisplayQueue   = NULL;    // cydDisplayQueue = xQueueCreate(8, sizeof(DisplayCmd));
QueueHandle_t cydTouchQueue     = NULL;      // cydTouchQueue   = xQueueCreate(8, sizeof(TouchEvent));


/* ========================================================================== 
*  Private functions
* ==========================================================================*/

/* ========================================================================== 
*  Public functions
* ==========================================================================*/

void freeRtosInit()
{
    /* Initialize message queues */
    canMsgTxQueue   = xQueueCreate(CAN_TX_QUEUE_LEN, sizeof(twai_message_t));
    canMsgRxQueue   = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(can_msg_t));
    outputTaskQueue = xQueueCreate(OUTPUT_TASK_QUEUE_LEN, sizeof(OutputCmd_t));
    inputTaskQueue  = xQueueCreate(INPUT_TASK_QUEUE_LEN, sizeof(InputCmd_t));
    gpioEventQueue  = xQueueCreate(GPIO_EVENT_QUEUE_LEN, sizeof(gpio_event_t));
    
#ifdef ESP32CYD
    // cydDisplayQueue = xQueueCreate(CYD_DISP_QUEUE_LEN, sizeof(DisplayCmd));
    // cydTouchQueue   = xQueueCreate(CYD_TOUCH_QUEUE_LEN, sizeof(TouchEvent));
    // xTaskCreate(TaskUpdateDisplay, "Display", TASK_DISPLAY_STACK_SIZE, NULL, tskNormalPriority, &xDisplayHandle);
    // xTaskCreate(TaskReadTouch,     "Touch",   TASK_TOUCH_STACK_SIZE,   NULL, tskNormalPriority, &xTouchHandle);
#endif
    
    ESP_LOGI(TAG, "[INIT] FreeRTOS queue memory initialized.");
}
