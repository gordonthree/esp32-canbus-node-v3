#include "argb_hw.h"



#ifdef ARGB_LED
#include <NeoPixelBus.h>  /* NeoPixelBus library */


/* Global strip pointers to allow for dynamic initialization */
// NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0800KbpsMethod>* strip = NULL;
// NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0800KbpsMethod>* g_argbStrips[MAX_SUB_MODULES] = { nullptr };
/**  store raw pointers because NeoPixelBus does not provide a polymorphic base class. */
void* g_argbStrips[MAX_SUB_MODULES] = { nullptr };

using RmtMethod0 = NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel0>;
using RmtMethod1 = NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel1>;
using RmtMethod2 = NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel2>;
using RmtMethod3 = NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel3>;
using RmtMethod4 = NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel4>;
using RmtMethod5 = NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel5>;
using RmtMethod6 = NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel6>;
using RmtMethod7 = NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel7>;

#endif


/**
 * @brief Handles specialized FastLED initialization for ARGB sub-modules.
 * @param index The sub-module index (used to index ledData arrays).
 * @param sub   Reference to the sub-module configuration.
 */
void initArgbHardware(uint8_t index, subModule_t *sub)
{
#ifdef ARGB_LED

    if (index >= MAX_SUB_MODULES) return;

    const personalityDef_t* p = &runtimePersonalityTable[sub->personalityIndex];
    if (!p) return;

    uint8_t  dataPin    = p->gpioPin;
    uint16_t pixelCount = sub->config.argb.ledCount;

    if (pixelCount > MAX_PIXEL_COUNT)
        pixelCount = MAX_PIXEL_COUNT;

    /* Free old strip if reinitializing */
    if (g_argbStrips[index] != nullptr)
    {
        free(g_argbStrips[index]);   // safe because we allocated with new
        g_argbStrips[index] = nullptr;
    }

    Serial.printf("ARGB using RMT channel %u\n", index);


    /* Instantiate strip with unique RMT channel */
    switch (index)
    {
        case 0:
            Serial.printf("Constructed type: RmtMethod%d\n", index);

            g_argbStrips[index] =
                new NeoPixelBus<NeoGrbFeature, RmtMethod0>(pixelCount, dataPin);
            break;

        case 1:
            Serial.printf("Constructed type: RmtMethod%d\n", index);
            g_argbStrips[index] =
                new NeoPixelBus<NeoGrbFeature, RmtMethod1>(pixelCount, dataPin);
            break;

        case 2:
            Serial.printf("Constructed type: RmtMethod%d\n", index);
            g_argbStrips[index] =
                new NeoPixelBus<NeoGrbFeature, RmtMethod2>(pixelCount, dataPin);
            break;

        case 3:
            Serial.printf("Constructed type: RmtMethod%d\n", index);
            g_argbStrips[index] =
                new NeoPixelBus<NeoGrbFeature, RmtMethod3>(pixelCount, dataPin);
            break;

        case 4:
            Serial.printf("Constructed type: RmtMethod%d\n", index);
            g_argbStrips[index] =
                new NeoPixelBus<NeoGrbFeature, RmtMethod4>(pixelCount, dataPin);
            break;

        case 5:
            Serial.printf("Constructed type: RmtMethod%d\n", index);
            g_argbStrips[index] =
                new NeoPixelBus<NeoGrbFeature, RmtMethod5>(pixelCount, dataPin);
            break;

        case 6:
            Serial.printf("Constructed type: RmtMethod%d\n", index);
            g_argbStrips[index] =
                new NeoPixelBus<NeoGrbFeature, RmtMethod6>(pixelCount, dataPin);
            break;

        case 7:
            Serial.printf("Constructed type: RmtMethod%d\n", index);
            g_argbStrips[index] =
                new NeoPixelBus<NeoGrbFeature, RmtMethod7>(pixelCount, dataPin);
            break;

        default:
            Serial.printf("ARGB Init ERROR: No RMT channel for submod %u\n", index);
            return;
    }

    /* Initialize strip */
    if (g_argbStrips[index] != nullptr)
    {
        // Cast back to the correct type based on index
        switch (index)
        {
            case 0:
                ((NeoPixelBus<NeoGrbFeature, RmtMethod0>*)g_argbStrips[index])->Begin();
                ((NeoPixelBus<NeoGrbFeature, RmtMethod0>*)g_argbStrips[index])->Show();
                break;

            case 1:
                ((NeoPixelBus<NeoGrbFeature, RmtMethod1>*)g_argbStrips[index])->Begin();
                ((NeoPixelBus<NeoGrbFeature, RmtMethod1>*)g_argbStrips[index])->Show();
                break;

            case 2:
                ((NeoPixelBus<NeoGrbFeature, RmtMethod2>*)g_argbStrips[index])->Begin();
                ((NeoPixelBus<NeoGrbFeature, RmtMethod2>*)g_argbStrips[index])->Show();
                break;

            case 3:
                ((NeoPixelBus<NeoGrbFeature, RmtMethod3>*)g_argbStrips[index])->Begin();
                ((NeoPixelBus<NeoGrbFeature, RmtMethod3>*)g_argbStrips[index])->Show();
                break;

            case 4:
                ((NeoPixelBus<NeoGrbFeature, RmtMethod4>*)g_argbStrips[index])->Begin();
                ((NeoPixelBus<NeoGrbFeature, RmtMethod4>*)g_argbStrips[index])->Show();
                break;

            case 5:
                ((NeoPixelBus<NeoGrbFeature, RmtMethod5>*)g_argbStrips[index])->Begin();
                ((NeoPixelBus<NeoGrbFeature, RmtMethod5>*)g_argbStrips[index])->Show();
                break;

            case 6:
                ((NeoPixelBus<NeoGrbFeature, RmtMethod6>*)g_argbStrips[index])->Begin();
                ((NeoPixelBus<NeoGrbFeature, RmtMethod6>*)g_argbStrips[index])->Show();
                break;

            case 7:
                ((NeoPixelBus<NeoGrbFeature, RmtMethod7>*)g_argbStrips[index])->Begin();
                ((NeoPixelBus<NeoGrbFeature, RmtMethod7>*)g_argbStrips[index])->Show();
                break;
        }

        Serial.printf("ARGB Init: submod %u pin %u count %u (RMT channel %u)\n",
                      index, dataPin, pixelCount, index);
    }

#endif
}

#ifdef ARGB_LED

void argbSetStripColor(uint8_t subIdx, uint8_t colorIndex)
{
    if (subIdx >= MAX_SUB_MODULES) return;
    if (g_argbStrips[subIdx] == nullptr) return;
    
    PaletteColor p = SystemPalette[colorIndex];


    switch (subIdx)
    {
        case 0:
            ((NeoPixelBus<NeoGrbFeature, RmtMethod0>*)g_argbStrips[subIdx])
                ->ClearTo(RgbColor(p.R, p.G, p.B));
            ((NeoPixelBus<NeoGrbFeature, RmtMethod0>*)g_argbStrips[subIdx])
                ->Show();
            break;

        case 1:
            ((NeoPixelBus<NeoGrbFeature, RmtMethod1>*)g_argbStrips[subIdx])
                ->ClearTo(RgbColor(p.R, p.G, p.B));
            ((NeoPixelBus<NeoGrbFeature, RmtMethod1>*)g_argbStrips[subIdx])
                ->Show();
            break;

        case 2:
            ((NeoPixelBus<NeoGrbFeature, RmtMethod2>*)g_argbStrips[subIdx])
                ->ClearTo(RgbColor(p.R, p.G, p.B));
            ((NeoPixelBus<NeoGrbFeature, RmtMethod2>*)g_argbStrips[subIdx])
                ->Show();
            break;

        case 3:
            ((NeoPixelBus<NeoGrbFeature, RmtMethod3>*)g_argbStrips[subIdx])
                ->ClearTo(RgbColor(p.R, p.G, p.B));
            ((NeoPixelBus<NeoGrbFeature, RmtMethod3>*)g_argbStrips[subIdx])
                ->Show();
            break;

        case 4:
            ((NeoPixelBus<NeoGrbFeature, RmtMethod4>*)g_argbStrips[subIdx])
                ->ClearTo(RgbColor(p.R, p.G, p.B));
            ((NeoPixelBus<NeoGrbFeature, RmtMethod4>*)g_argbStrips[subIdx])
                ->Show();
            break;

        case 5:
            ((NeoPixelBus<NeoGrbFeature, RmtMethod5>*)g_argbStrips[subIdx])
                ->ClearTo(RgbColor(p.R, p.G, p.B));
            ((NeoPixelBus<NeoGrbFeature, RmtMethod5>*)g_argbStrips[subIdx])
                ->Show();
            break;

        case 6:
            ((NeoPixelBus<NeoGrbFeature, RmtMethod6>*)g_argbStrips[subIdx])
                ->ClearTo(RgbColor(p.R, p.G, p.B));
            ((NeoPixelBus<NeoGrbFeature, RmtMethod6>*)g_argbStrips[subIdx])
                ->Show();
            break;

        case 7:
            ((NeoPixelBus<NeoGrbFeature, RmtMethod7>*)g_argbStrips[subIdx])
                ->ClearTo(RgbColor(p.R, p.G, p.B));
            ((NeoPixelBus<NeoGrbFeature, RmtMethod7>*)g_argbStrips[subIdx])
                ->Show();
            break;
    }

    Serial.printf("ARGB[%d] = (%d,%d,%d)\n", subIdx, p.R, p.G, p.B);
}
#endif