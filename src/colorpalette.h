#ifndef COLOR_PALETTE_H_
#define COLOR_PALETTE_H_

#include <NeoPixelBus.h>

#define COLOR_PALETTE_SIZE     (32U)       /**< Size of the SystemPalette array */

/** * @brief Visually corrected 32-color palette.
 * @details Uses a power-law distribution for brightness to appear linear to the eye.
 * Updated to use NeoPixelBus RgbColor types.
 */
const RgbColor SystemPalette[COLOR_PALETTE_SIZE] = {
    /* Row 1: Very Dim (approx 6% brightness) */
    RgbColor(0, 0, 0),      /* Black */
    RgbColor(32, 0, 0),     RgbColor(0, 32, 0),     RgbColor(0, 0, 32),   
    RgbColor(20, 20, 0),    RgbColor(0, 20, 20),    RgbColor(20, 0, 20),  
    RgbColor(32, 16, 0),
    
    /* Row 2: Medium-Low (approx 25% brightness) */
    RgbColor(64, 64, 64),   RgbColor(96, 0, 0),     RgbColor(0, 96, 0),   
    RgbColor(0, 0, 96),     RgbColor(64, 64, 0),    RgbColor(0, 64, 64),  
    RgbColor(64, 0, 64),    RgbColor(96, 48, 0),
    
    /* Row 3: Medium-High (approx 56% brightness) */
    RgbColor(144, 144, 144), RgbColor(192, 0, 0),    RgbColor(0, 192, 0),  
    RgbColor(0, 0, 192),     RgbColor(144, 144, 0),  RgbColor(0, 144, 144), 
    RgbColor(144, 0, 144),   RgbColor(192, 96, 0),
    
    /* Row 4: Full Brightness (100%) */
    RgbColor(255, 255, 255), /**< White */
    RgbColor(255, 0, 0),     /**< Red */
    RgbColor(0, 255, 0),     /**< Green */
    RgbColor(0, 0, 255),     /**< Blue */
    RgbColor(255, 255, 0),   /**< Yellow */
    RgbColor(0, 255, 255),   /**< Cyan */
    RgbColor(255, 0, 255),   /**< Magenta */
    RgbColor(255, 215, 0)    /**< Gold: RGB(255, 215, 0) */
};

#endif /* END COLOR_PALETTE_H_ */