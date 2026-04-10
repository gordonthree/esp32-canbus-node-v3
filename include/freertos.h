#pragma once
#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* ========================================================================== 
*  Macros and constants 
* ==========================================================================*/
/* memory allocation for main tasks */
#define TASK_TWAI_STACK_SIZE     (4096U)
#define TASK_OTA_STACK_SIZE      (4096U)
#define TASK_OUTPUT_STACK_SIZE   (4096U)
#define TASK_INPUT_STACK_SIZE    (4096U)
#define TASK_PRODUCER_STACK_SIZE (4096U)
#define TASK_CONSUMER_STACK_SIZE (4096U)

#define tskLowPriority           (tskIDLE_PRIORITY + 1)
#define tskNormalPriority        (tskIDLE_PRIORITY + 2)
#define tskHighPriority          (tskIDLE_PRIORITY + 4)
#define tskHigherPriority        (tskIDLE_PRIORITY + 6)

/* Queue send wait times */
#define QUEUE_SHORT_WAIT         (1)             /* 1 ticks */
#define QUEUE_NO_WAIT            (0)             /* no wait */
#define QUEUE_WAIT_FOREVER       (portMAX_DELAY) /* wait forever */

/* Queue lengths */
#define CAN_TX_QUEUE_LEN         (16)
#define CAN_RX_QUEUE_LEN         (16)
#define OUTPUT_TASK_QUEUE_LEN    (MAX_SUB_MODULES + 8)  /* enough for each sub module + headroom */
#define INPUT_TASK_QUEUE_LEN     (MAX_SUB_MODULES * 2)  /* enough for each sub module to handle 2 input events */
#define GPIO_EVENT_QUEUE_LEN     (INPUT_TASK_QUEUE_LEN) /* ISR Event Queue */
#define CYD_DISP_QUEUE_LEN       (16)
#define CYD_TOUCH_QUEUE_LEN      (16)

/* Output command types */
typedef enum {
    OUTPUT_CMD_APPLY_STATE,      // param1 = outputState_t
    OUTPUT_CMD_APPLY_MODE,       // param1 = outputMode_t
    OUTPUT_CMD_SET_PWM_FREQ,     // param1 = Hz
    OUTPUT_CMD_SET_PWM_DUTY,     // param1 = 13-bit duty
    OUTPUT_CMD_TRACKER_ACTIVE,   // param1 = true / false, enable tracker
    OUTPUT_CMD_TRACKER_CFG,      // param1 = true / false, output supported
    OUTPUT_CMD_TRACKER_RESET,    // param1 = true, reset strobe counter and momentary timer
    OUTPUT_CMD_SET_COLOR_INDEX,  // param1 = palette index
    OUTPUT_CMD_SET_DISPLAY_MODE, // param1 = display mode
    OUTPUT_CMD_INVALID           // keep this last in the list
} OutputCmdType_t;

typedef struct {
    OutputCmdType_t type;
    uint8_t         index;       // submodule index
    uint32_t        param1;      // generic parameter
} OutputCmd_t;

/* Input command types */
typedef enum {
    INPUT_CMD_GPIO_EVENT,     /**< GPIO ISR event has fired*/
    INPUT_CMD_CFG_CHANGE,     /**< Configuration update flag */
    INPUT_CMD_INTERNAL_EVENT, /**< Operational data based event */
    INPUT_CMD_DEBOUNCE_TICK   /**< Periodic debounce update */
} inputCommand_t ;

typedef struct {
    inputCommand_t type;   /**< What action the input task should perform */
    uint8_t        index;  /**< Submodule index */
    uint32_t       timestamp; /**< timestamp when event occurred */
} inputCommandMsg_t;

/* GPIO event structure */
typedef struct {
    uint8_t  subIdx;
    uint8_t  raw;
} gpio_event_t;


/* ========================================================================== 
*  Public variables
* ==========================================================================*/

/* Semaphore handle declarations */
extern SemaphoreHandle_t flashMutex; /**< Mutex for NVS access */

/* Task handle declarations */
extern TaskHandle_t xTWAIHandle;     /**< Handle for the TWAI task */
extern TaskHandle_t xInputHandle;    /**< Handle for the input event logic task */
extern TaskHandle_t xOutputHandle;   /**< Handle for the output switch logic task */
extern TaskHandle_t xOTAHandle;      /**< Handle for the OTA task */
extern TaskHandle_t xConsumerHandle; /**< Handle for the consumer task */
extern TaskHandle_t xProducerHandle; /**< Handle for the producer task */


#ifdef ESP32CYD
extern TaskHandle_t xDisplayHandle; /**< Handle for the display task */
extern TaskHandle_t xTouchHandle;   /**< Handle for the touch task */
#endif

/* Message Queues */
extern QueueHandle_t canMsgTxQueue;    /**< TWAI transmit queue  */
extern QueueHandle_t canMsgRxQueue;    /**< TWAI receive queue   */
extern QueueHandle_t outputTaskQueue;  /**< Output task queue    */
extern QueueHandle_t inputTaskQueue;   /**< Input task queue     */
extern QueueHandle_t gpioEventQueue;   /**< ISR GPIO event queue */


/* Doesn't hurt to declare these CYD queue even if they are not used */
extern QueueHandle_t cydDisplayQueue;    // cydDisplayQueue = xQueueCreate(8, sizeof(DisplayCmd));
extern QueueHandle_t cydTouchQueue;      // cydTouchQueue   = xQueueCreate(8, sizeof(TouchEvent));

/* Command buffers */
extern OutputCmd_t outputCmd;

/* ========================================================================== 
*  Private functions
* ==========================================================================*/

/* ========================================================================== 
*  Public functions
* ==========================================================================*/

/** Initialize FreeRTOS resources */ 
void freeRtosInit();


void enqueueLoopback(const uint16_t msgid, const uint8_t *data, uint8_t dlc);