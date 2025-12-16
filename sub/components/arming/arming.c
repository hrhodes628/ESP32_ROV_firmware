#include "arming.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/*
 * Arming state is a safety-critical global.
 * It must be:
 *  - centralized
 *  - thread-safe
 *  - default DISARMED
 */

static bool armed = false;
static SemaphoreHandle_t arm_lock;

/* ---------- Init ---------- */
void arming_init(void)
{
    arm_lock = xSemaphoreCreateMutex();
    armed = false;   // ALWAYS boot disarmed
}

/* ---------- Query ---------- */
bool arming_is_armed(void)
{
    bool v;
    xSemaphoreTake(arm_lock, portMAX_DELAY);
    v = armed;
    xSemaphoreGive(arm_lock);
    return v;
}

/* ---------- Control ---------- */
void arming_set(bool v)
{
    xSemaphoreTake(arm_lock, portMAX_DELAY);
    armed = v;
    xSemaphoreGive(arm_lock);
}