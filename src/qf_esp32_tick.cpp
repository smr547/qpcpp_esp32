// qf_esp32_tick.cpp
//
// ESP32-safe QP time-event tick driver.
// - FreeRTOS tick hook runs in ISR context and may execute while flash cache is disabled.
// - Therefore the hook must be IRAM-only and do the minimum work.
// - We notify a dedicated task (pinned to QP_CPU_NUM) which performs QTimeEvt::TICK_X()
//   in task context.
//
// This is "Option A": BSP calls QP::ESP32_tickHookInit() from QF::onStartup().

#include "qf_port.hpp"     // provides QP_CPU_NUM, QP types, QTimeEvt, etc.

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "esp_freertos_hooks.h"   // esp_register_freertos_tick_hook_for_cpu()
}

namespace QP {

static TaskHandle_t s_qpTickTask = nullptr;
static uint_fast8_t s_tickRate   = 0U;

// QS sender object for tick events (used by QS tracing; harmless if QS_OFF)
#ifdef QS_ON
static QP::QSpyId const s_tickHookId = { 0U };
#endif


// ---------------------------------------------------------------------------
// IRAM-safe tick hook (ISR context). Must not call QP services directly.
static void IRAM_ATTR tickHook_ESP32(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (s_qpTickTask != nullptr) {
        vTaskNotifyGiveFromISR(s_qpTickTask, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken != pdFALSE) {
            portYIELD_FROM_ISR();
        }
    }
}

// ---------------------------------------------------------------------------
// Task context tick processing. Safe to call into QP here.
static void QpTickTask(void *pv) {
    (void)pv;

    for (;;) {
        // Wait for the ISR hook to notify us once per RTOS tick
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Process QP time events for the configured tick rate
#ifdef QS_ON
        QP::QTimeEvt::TICK_X(0U, &s_tickHookId);
#else
        QP::QTimeEvt::TICK_X(0U, static_cast<void const *>(nullptr));
#endif

    }
}

// ---------------------------------------------------------------------------
// Public init called by BSP's QF::onStartup()
void ESP32_tickHookInit(uint_fast8_t tickRate, UBaseType_t tickTaskPrio) {

    // Idempotent: safe to call more than once
    if (s_qpTickTask != nullptr) {
        return;
    }

    s_tickRate = tickRate;

#ifdef QS_ON
    // Register the tick sender object in QS dictionaries
    QS_OBJ_DICTIONARY(&s_tickHookId);
#endif

    // Create the tick task pinned to the same core as QP (QP_CPU_NUM)
    BaseType_t ok = xTaskCreatePinnedToCore(
        QpTickTask,
        "QpTick",
        4096,               // stack size in words (same as your BSP version)
        nullptr,
        tickTaskPrio,
        &s_qpTickTask,
        QP_CPU_NUM
    );
    configASSERT(ok == pdPASS);
    configASSERT(s_qpTickTask != nullptr);

    // Register the FreeRTOS tick hook for the same CPU
    // (hook runs in ISR context; only notifies the QpTick task)
    esp_register_freertos_tick_hook_for_cpu(tickHook_ESP32, QP_CPU_NUM);
}

} // namespace QP

