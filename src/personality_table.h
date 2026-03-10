/**
 * @file personality_table.h
 * @brief Hardware personality definitions for all submodules.
 *
 * This file defines the firmware-side hardware mapping for each personality.
 * These definitions describe:
 *   - GPIO pins
 *   - PWM timer/channel assignments
 *   - Hardware capabilities (input/output/PWM/strobe/etc.)
 *   - CAN data message IDs and DLCs
 *   - Electrical topology (source vs sink driver)
 *
 * IMPORTANT:
 *   - This file contains *hardware* definitions only.
 *   - User-configurable behavior (mode, pull-up, blink delay, etc.)
 *     lives in subModule_t.config (3-byte config block).
 *   - User-level semantic identity (introMsgId/DLC) also lives in subModule_t.
 */

#pragma once            /** Prevent multiple inclusion */
#include <stdint.h>
#include <stdbool.h>

#include "canbus_project.h"

/* --------------------------------------------------------------------------
 * Capability Flags
 * -------------------------------------------------------------------------- */

/**
 * @brief Capability bitmask for submodule hardware features.
 *
 * These flags describe what the hardware *can* do, not what the user configures.
 * Multiple flags may be combined (e.g., CAP_OUTPUT | CAP_PWM).
 */
#define CAP_NONE         (0x00)     /**< No capabilities */
#define CAP_INPUT        (1U << 0)  /**< Submodule can read digital/analog input */
#define CAP_OUTPUT       (1U << 1)  /**< Submodule can drive a GPIO output */
#define CAP_PWM          (1U << 2)  /**< Submodule supports PWM output */
#define CAP_STROBE       (1U << 3)  /**< Submodule supports strobe patterns */
#define CAP_ANALOG       (1U << 4)  /**< Submodule supports ADC input */
#define CAP_ARGB         (1U << 5)  /**< Submodule supports addressable RGB output */
#define CAP_ANALOGRGB    (1U << 6)  /**< Submodule supports analog RGB output.  */
#define CAP_RESERVED7    (1U << 7)  /**< Reserved for future use */

#define NO_DATA_REPORTING 0x00
#define NO_GPIO_ASSIGNED  0xFF
#define NO_PWM_ASSIGNED   0xFF

/* --------------------------------------------------------------------------
 * Personality Definition Structure
 * -------------------------------------------------------------------------- */

/**
 * @brief Firmware-defined hardware personality descriptor.
 *
 * This structure defines the *hardware* characteristics of a submodule.
 * It is compiled into firmware and never transmitted over CAN.
 *
 * Fields:
 *   - personalityId: Unique ID that matches subModule_t.personalityId
 *   - capabilities: Bitmask describing what the hardware can do
 *   - gpioPin: Physical pin number for GPIO/PWM
 *   - pwmChannel: PWM channel index (0xFF if not PWM-capable)
 *   - pwmTimer: PWM timer index (0xFF if not PWM-capable)
 *   - isSinkDriver: true = low-side switch, false = high-side
 *   - dataMsgId: CAN message ID used for runtime data reporting
 *   - dataMsgDlc: DLC for runtime data messages
 */
typedef struct {
    uint8_t  personalityId;     /**< Unique personality identifier */
    uint8_t  capabilities;      /**< CAP_* bitmask describing hardware features */

    /* Hardware mapping */
    uint8_t  gpioPin;           /**< Physical GPIO pin index */
    uint8_t  pwmChannel;        /**< PWM channel (0xFF if unused) */
    uint8_t  pwmTimer;          /**< PWM timer (0xFF if unused) */
    bool     isSinkDriver;      /**< true = low-side driver, false = high-side */

    /* CAN data reporting */
    uint16_t dataMsgId;         /**< CAN ID for runtime data messages */
    uint8_t  dataMsgDlc;        /**< DLC for runtime data messages */

} personalityDef_t;

/**
 * @brief Enumerates all hardware personality IDs.
 * @enum personalityId_t
 * 
 * These values appear in subModule_t.personalityId and must match
 * the entries in the personality table.
 */
typedef enum {
    PERS_NONE = 0,              /**< Unconfigured / invalid */

    /* Output personalities */
    PERS_GPIO_OUTPUT = 1,       /**< Generic GPIO output (toggle/momentary/strobe/PWM) */
    PERS_ARGB_OUTPUT = 2,       /**< Addressable ARGB LED strip (NeoPixelBus) */
    PERS_RGBW_OUTPUT = 3,       /**< Analog RGBW LED strip (GPIO + PWM) */
    PERS_ANA_OUTPUT  = 4,       /**< Generic analog output (DAC) */

    /* Input personalities */
    PERS_GPIO_INPUT    = 10,    /**< Digital GPIO input */
    PERS_ANALOG_INPUT  = 11,    /**< ADC input */

    /* System personalities */
    SYS_TOUCH_LCD     = 0xF0,     /**< Touchscreen (e.g., CYD / XPT2046) */
    SYS_NON_TOUCH_LCD = 0xF1      /**< Generic display (e.g., SSD1306) */

    /* Add more as needed */
} personalityId_t;


/* --------------------------------------------------------------------------
 * Personality Table Declaration
 * -------------------------------------------------------------------------- */

/**
 * @brief Personality table for this node type.
 *
 * This table is generated per node type (via automation or build scripts).
 * The firmware selects the correct table at boot based on nodeTypeMsg.
 *
 * Example:
 *   extern const personalityDef_t personalityTable_0x700[];
 *   extern const uint8_t personalityCount_0x700;
 *
 * The actual table is defined in a node-type-specific .c file.
 */
extern const personalityDef_t *personalityTable;  /**< Active personality table */
extern uint8_t g_personalityCount;                  /**< Number of personalities */

/* --------------------------------------------------------------------------
 * Lookup Helpers
 * -------------------------------------------------------------------------- */

#ifdef __cplusplus
extern "C" {
#endif

const personalityDef_t *getPersonality(uint8_t index);

/**
 * @brief Retrieve a personality definition by ID.
 *
 * @param personalityId The ID to look up.
 * @return Pointer to personalityDef_t, or NULL if not found.
 */
const personalityDef_t *getPersonality(uint8_t personalityId);

#ifdef __cplusplus
}
#endif 
