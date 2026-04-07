#include "task_output_tracker.h"
#include "task_output_gpio.h"
#include "esp_log.h"

static const char *TAG = "task_output_tracker";

/*
 * =========================================================================
 * Constants and macros
 * =========================================================================
 */

/*
 * =========================================================================
 * Private Variables
 * =========================================================================
 */

static outputTracker_t trackers[MAX_SUB_MODULES];

/*
 * =========================================================================
 * Public Variables
 * =========================================================================
 */

/*
 * =========================================================================
 * Private Function Declarations
 * =========================================================================
 */

static void handleMomentaryLogic(subModule_t *sub, outputTracker_t &trk);
static void handleStrobeLogic(subModule_t *sub, outputTracker_t &trk);

/*
 * =========================================================================
 * Private Functions
 * =========================================================================
 */

static void handleMomentaryLogic(subModule_t *sub, outputTracker_t &trk)
{
    /* Pointer to the personality definition for this sub-module */
    const personalityDef_t *p =
        nodeGetActivePersonality(sub->personalityIndex);

    if (!trk.isActive)
        return;

    /* If timer still running → keep output HIGH */
    if (millis() < trk.nextActionTime)
    {
        setOutput(sub, p, true);
        return;
    }

    /* Timer expired → turn output OFF */
    ESP_LOGD(TAG, "Momentary expired: sub=%u", sub->personalityIndex);

    setOutput(sub, p, false);
    trk.isActive = false;
}

static void handleStrobeLogic(subModule_t *sub, outputTracker_t &trk)
{
    /* Pointer to the personality definition for this sub-module */
    const personalityDef_t *p =
        nodeGetActivePersonality(sub->personalityIndex);
    const uint8_t outPin = p->gpioPin; /* GPIO output pin */

    /* If the strobe is not active, do nothing */
    if (!trk.isActive)
        return;

    /* If it's not time to advance the pattern, do nothing */
    if (millis() < trk.nextActionTime)
        return;

    /* Select the pattern based on user configuration */
    uint8_t patternId = sub->config.gpioOutput.param2;

    /* Check if the pattern index is valid otherwise, disable tracker and exit */
    if (patternId >= (sizeof(STROBE_PATTERNS) / sizeof(STROBE_PATTERNS[0])))
    {
        ESP_LOGW(TAG, "Invalid strobe pattern %u on sub %u", patternId, sub->personalityIndex);
        trk.isActive = false;
        setOutput(sub, p, false);
        return;
    }

    /** Strobe pattern pointer */
    const StrobePatternDef &def = STROBE_PATTERNS[patternId];

    /* Invalid pattern, turn off output and exit */
    if (!def.steps || def.count == 0)
    {
        ESP_LOGW(TAG, "Empty strobe pattern %u on sub %u", patternId, sub->personalityIndex);
        trk.isActive = false;
        setOutput(sub, p, false);
        return;
    }

    /* Load the pattern specifics */
    const uint16_t *pattern = def.steps;
    uint8_t patternSteps = def.count;

    /* Advance to the next step in the pattern */
    trk.currentStep = (trk.currentStep + 1) % patternSteps;

    /* Even steps = ON, Odd steps = OFF */
    const bool state = (trk.currentStep % 2 == 0);

    /* Set the GPIO output state */
    setOutput(sub, p, state);

    /* Schedule the next step */
    trk.nextActionTime = millis() + pattern[trk.currentStep];

    ESP_LOGV(TAG, "Strobe step: sub=%u step=%u state=%d",
             sub->personalityIndex, trk.currentStep, state);

    /* Record the GPIO state in runTime */
    sub->runTime.valueU32 = packStrobeState(
        sub->config.gpioOutput.param2, // pattern ID
        trk.currentStep,               // step index
        state                          // GPIO state
    );
}

/*
 * =============================================================================
 * Public API Functions
 * ============================================================================
 */

/**
 * @brief Retrieve a pointer to the array of outputTracker_t structs.
 *
 * This function returns a pointer to the array of outputTracker_t structs, which
 * contain the current state of all submodules.
 *
 * @return Pointer to the array of outputTracker_t structs.
 */
outputTracker_t *getOutputTrackers(void)
{
    return trackers;
}

/**
 * @brief Retrieve the output tracker struct for a given submodule index.
 *
 * @param index uint8_t value representing the submodule index (0-255).
 * @return Pointer to the outputTracker_t struct associated with the submodule index.
 * @retval nullptr if index is out of range (0-255).
 */
outputTracker_t *getOutputTracker(const uint8_t index)
{
    if (index >= MAX_SUB_MODULES)
        return nullptr;

    return &trackers[index];
}

/**
 * @brief Initialise the output tracker data structure.
 *
 * This function initialises the output tracker data structure by setting all
 * values to zero. This should be called before the output tracker is used.
 */
void outputTrackerInit(void)
{
    memset(trackers, 0, sizeof(trackers));
    ESP_LOGI(TAG, "Tracker initialized");
}

/** Reset the state machine for a particular sub-module tracker */
void outputTrackerReset(const uint8_t index)
{
    outputTracker_t &trk = trackers[index];

    trk.nextActionTime = 0;
    trk.currentStep = 0;
}

void outputTrackerActive(const uint8_t index, bool state)
{
    outputTracker_t &trk = trackers[index];

    trk.isActive = state;
}

void outputTrackerConfig(const uint8_t index, bool state)
{
    outputTracker_t &trk = trackers[index];

    trk.outputSupported = state;
}

void outputTrackerTick(void)
{
    /* reconcile submodule count */
    nodeGetActiveSubModuleCount();

    for (int i = 0; i < MAX_SUB_MODULES; i++)
    {
        subModule_t *sub = nodeGetActiveSubModule(i);

        if (!sub)
            continue;

        /** Pointer to the personality definition for this sub-module */
        const personalityDef_t *p =
            nodeGetActivePersonality(sub->personalityIndex);

        /** Reference to the output tracker for this sub-module */
        outputTracker_t &trk = trackers[i];

        /* Skip submodules that are not configured for output behavior */
        if (!trk.outputSupported)
            continue;

        /* Skip personalities that do not support output behavior */
        if (!(p->capabilities & CAP_OUTPUT))
            continue;

        /* Dispatch based on the configured output mode */
        switch (sub->config.gpioOutput.mode)
        {
        case OUT_MODE_MOMENTARY:
            handleMomentaryLogic(sub, trk);
            break;

        case OUT_MODE_STROBE:
            handleStrobeLogic(sub, trk);
            break;

        case OUT_MODE_BLINK:
        case OUT_MODE_PWM:
            /**
             * BLINK and PWM are handled in a separate helper,
             * nothing to do here
             */
            break;

        case OUT_MODE_ALWAYS_ON:
            if (!trk.hasBeenSet)
            {
                setOutput(sub, p, true);
                trk.hasBeenSet = true;
                ESP_LOGD(TAG, "Always ON: sub=%u", sub->personalityIndex);
            }
            break;

        case OUT_MODE_ALWAYS_OFF:
            if (!trk.hasBeenSet)
            {
                setOutput(sub, p, false);
                trk.hasBeenSet = true;
                ESP_LOGD(TAG, "Always OFF: sub=%u", sub->personalityIndex);
            }
            break;

        case OUT_MODE_FOLLOW:
            /* FOLLOW mode reacts only to incoming CAN events,
            so the output task does nothing here. */
            break;

        case OUT_MODE_TOGGLE:
            /* TOGGLE is also event-driven; nothing to do here. */
            break;

        default:
            /* Unknown mode — do nothing */
            ESP_LOGW(TAG, "Unknown output mode %u on sub %u",
                     sub->config.gpioOutput.mode, sub->personalityIndex);
            break;
        } /* switch (sub.config.digitalOutput.outputMode) */
    } /* for (int i = 0; i < MAX_SUB_MODULES; i++) */
}
