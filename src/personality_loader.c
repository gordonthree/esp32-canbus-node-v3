/**
 * @file personality_loader.c
 * @brief Load the appropriate personality file.
 *
 * @details I guess this is dynamic include, it should load the correct personality file based on NODE_TYPE build flag
 * 
 * @author Gordon McLellan
 * @date 2026-03-09
 * 
 */

#include "personality_table.h"

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define CONCAT_HELPER(a, b) a ## b
#define CONCAT(a, b) CONCAT_HELPER(a, b)

/** * Create the specific filename token by appending the NODE_TYPE.
 * If NODE_TYPE is defined as SENSOR_01, this becomes personality_SENSOR_01.
 */
#define PERSONALITY_ID CONCAT(personality_, NODE_TYPE)

/**
 * Construct the final path string.
 * The 'file' argument is expanded before the STR macro performs stringification.
 */
#define PERSONALITY_PATH(file) STR(../personalities/file.c)

#include PERSONALITY_PATH(PERSONALITY_ID)
