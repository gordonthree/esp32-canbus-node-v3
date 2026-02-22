#ifndef COLOR_PALETTE_H_
#define COLOR_PALETTE_H_

#include <stdint.h>

#define COLOR_PALETTE_SIZE     (32U)       /**< Size of the SystemPalette array */

/**
 * @brief Standalone RGB structure to avoid naming collisions with NeoPixelBus.
 * @details Renamed to PaletteColor to distinguish from NeoPixelBus::RgbColor.
 */
struct PaletteColor {
    uint8_t R;
    uint8_t G;
    uint8_t B;

    /* Constructor to allow PaletteColor(r, g, b) syntax */
    PaletteColor(uint8_t r, uint8_t g, uint8_t b) : R(r), G(g), B(b) {}
    PaletteColor() : R(0), G(0), B(0) {}
};

/** * @brief Visually corrected 32-color palette.
 * @details Uses a power-law distribution for brightness to appear linear to the eye.
 */
const PaletteColor SystemPalette[COLOR_PALETTE_SIZE] = {
    /* Row 1: Very Dim (approx 6% brightness) */
    PaletteColor(0, 0, 0),      /* Black */
    PaletteColor(32, 0, 0),     PaletteColor(0, 32, 0),     PaletteColor(0, 0, 32),   
    PaletteColor(20, 20, 0),    PaletteColor(0, 20, 20),    PaletteColor(20, 0, 20),  
    PaletteColor(32, 16, 0),
    
    /* Row 2: Medium-Low (approx 25% brightness) */
    PaletteColor(64, 64, 64),   PaletteColor(96, 0, 0),     PaletteColor(0, 96, 0),   
    PaletteColor(0, 0, 96),     PaletteColor(64, 64, 0),    PaletteColor(0, 64, 64),  
    PaletteColor(64, 0, 64),    PaletteColor(96, 48, 0),
    
    /* Row 3: Medium-High (approx 56% brightness) */
    PaletteColor(144, 144, 144), PaletteColor(192, 0, 0),    PaletteColor(0, 192, 0),  
    PaletteColor(0, 0, 192),     PaletteColor(144, 144, 0),  PaletteColor(0, 144, 144), 
    PaletteColor(144, 0, 144),   PaletteColor(192, 96, 0),
    
    /* Row 4: Full Brightness (100%) */
    PaletteColor(255, 255, 255), /**< White */
    PaletteColor(255, 0, 0),     /**< Red */
    PaletteColor(0, 255, 0),     /**< Green */
    PaletteColor(0, 0, 255),     /**< Blue */
    PaletteColor(255, 255, 0),   /**< Yellow */
    PaletteColor(0, 255, 255),   /**< Cyan */
    PaletteColor(255, 0, 255),   /**< Magenta */
    PaletteColor(255, 215, 0)    /**< Gold */
};

#endif /* END COLOR_PALETTE_H_ */