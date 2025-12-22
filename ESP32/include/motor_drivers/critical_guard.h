// critical_guard.h
#pragma once
#if defined(UNIT_TEST_HOST)
// Provide minimal host-side stubs so unit tests can compile without
// linking FreeRTOS headers. These are intentionally no-ops for host tests.
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
static inline void portENTER_CRITICAL(portMUX_TYPE *) {}
static inline void portEXIT_CRITICAL(portMUX_TYPE *) {}
#else
#include <freertos/portmacro.h>
#endif

namespace abbot {
namespace motor {

/**
 * @brief RAII guard that wraps a FreeRTOS `portMUX` critical section.
 *
 * This small helper calls `portENTER_CRITICAL(mux)` when constructed and
 * `portEXIT_CRITICAL(mux)` when destroyed. Use it to ensure critical
 * sections are balanced across early returns or exceptions.
 *
 * @warning The guard stores a non-owning pointer to the `portMUX_TYPE`.
 * The caller is responsible for ensuring the referenced mutex has a
 * lifetime that outlives the guard (typically a static or global mutex).
 *
 * @warning This uses the normal `portENTER_CRITICAL`/`portEXIT_CRITICAL`
 * API and is not safe for ISR contexts that require the `_ISR` variants.
 * If you need to protect an ISR, use the platform ISR-safe primitives
 * (e.g. `portENTER_CRITICAL_ISR`) and a separate RAII wrapper.
 *
 * @note The type is non-copyable and non-assignable to avoid double-unlock.
 */
struct CriticalGuard {
  explicit CriticalGuard(portMUX_TYPE *m) : mux(m) { portENTER_CRITICAL(mux); }
  ~CriticalGuard() noexcept { portEXIT_CRITICAL(mux); }
  CriticalGuard(const CriticalGuard &) = delete;
  CriticalGuard &operator=(const CriticalGuard &) = delete;

private:
  portMUX_TYPE *mux;
};

} // namespace motor
} // namespace abbot
