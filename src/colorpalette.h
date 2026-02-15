#ifndef COLOR_PALETTE_H_
#define COLOR_PALETTE_H_

#include <FastLED.h>

/** * @brief Global index-based palette.
 * @note Transmit the index (0-31) over CAN to save bandwidth.
 */
// const CRGB SystemPalette[32] = {
//     CRGB::Black,        CRGB::White,        CRGB::Red,          CRGB::Green,
//     CRGB::Blue,         CRGB::Yellow,       CRGB::Cyan,         CRGB::Magenta,
//     CRGB::Orange,       CRGB::Purple,       CRGB::Lime,         CRGB::DarkGreen,
//     CRGB::DeepPink,     CRGB::HotPink,      CRGB::DarkOrange,   CRGB::LightSkyBlue,
//     CRGB::DodgerBlue,   CRGB::DarkViolet,   CRGB::DarkRed,      CRGB::FireBrick,
//     CRGB::Aqua,         CRGB::Aquamarine,   CRGB::Teal,         CRGB::Gold,
//     CRGB::Salmon,       CRGB::Lavender,     CRGB::Maroon,       CRGB::Olive,
//     CRGB::SteelBlue,    CRGB::Chocolate,    CRGB::OrangeRed,    CRGB::RoyalBlue
// };

// const CRGB SystemPalette[32] = {
//     CRGB::Black,         CRGB::Red4,         CRGB::Green4,       CRGB::Blue4,        CRGB::Yellow4,      CRGB::Cyan4,        CRGB::Magenta4,
//     CRGB::Gray25,        CRGB::Red3,         CRGB::Green3,       CRGB::Blue3,        CRGB::Yellow3,      CRGB::Cyan3,        CRGB::Magenta3,
//     CRGB::Gray50,        CRGB::Red2,         CRGB::Green2,       CRGB::Blue2,        CRGB::Yellow2,      CRGB::Cyan2,        CRGB::Magenta2,
//     CRGB::White,         CRGB::Red,          CRGB::Green,        CRGB::Blue,         CRGB::Yellow,       CRGB::Cyan,         CRGB::Magenta,
// };

/** 
 * @brief Visually corrected 32-color palette.
 * @details Uses a power-law distribution for brightness to appear linear to the eye.
 */
const CRGB SystemPalette[32] = {
    /* Row 1: Very Dim (approx 6% brightness) */
    CRGB::Black, CRGB(32,0,0),   CRGB(0,32,0),   CRGB(0,0,32),   CRGB(20,20,0),  CRGB(0,20,20),  CRGB(20,0,20),  CRGB(32,16,0),
    
    /* Row 2: Medium-Low (approx 25% brightness) */
    CRGB(64,64,64), CRGB(96,0,0),   CRGB(0,96,0),   CRGB(0,0,96),   CRGB(64,64,0),  CRGB(0,64,64),  CRGB(64,0,64),  CRGB(96,48,0),
    
    /* Row 3: Medium-High (approx 56% brightness) */
    CRGB(144,144,144), CRGB(192,0,0), CRGB(0,192,0), CRGB(0,0,192), CRGB(144,144,0), CRGB(0,144,144), CRGB(144,0,144), CRGB(192,96,0),
    
    /* Row 4: Full Brightness (100%) */
    CRGB::White,    CRGB::Red,      CRGB::Green1,    CRGB::Blue,     CRGB::Yellow,   CRGB::Cyan,     CRGB::Magenta,  CRGB::Gold
};
#endif /* END COLOR_PALETTE_H_ */