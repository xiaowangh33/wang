#ifndef INC_WD_FAST_FEEDBACK_H_
#define INC_WD_FAST_FEEDBACK_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * These are qualification limits, not RobStride protocol facts. They are
 * deliberately build-time overrides so the bench capture can tighten them.
 * A failed qualification always leaves the live command at zero.
 */
#ifndef WD_FAST_FEEDBACK_MIN_HZ
#define WD_FAST_FEEDBACK_MIN_HZ (350u)
#endif
#ifndef WD_FAST_FEEDBACK_RATE_WINDOW_MS
#define WD_FAST_FEEDBACK_RATE_WINDOW_MS (500u)
#endif
#ifndef WD_FAST_FEEDBACK_STALE_MS
#define WD_FAST_FEEDBACK_STALE_MS (5u)
#endif
#ifndef WD_FAST_FEEDBACK_STOP_MS
/*
 * A sample older than STALE_MS is never reused for control.  STOP_MS is a
 * separate, longer bound for escalating that already-zeroed channel into a
 * latched all-motor stop.  At 500 Hz this permits recovery from a short CAN
 * scheduling hole, while a genuinely missing motor is still stopped within
 * 300 ms. Each stale channel is still commanded to
 * zero immediately after WD_FAST_FEEDBACK_STALE_MS; this longer value only
 * controls when that local zero escalates into a latched all-motor stop.
 */
#define WD_FAST_FEEDBACK_STOP_MS (300u)
#endif
#ifndef WD_FAST_REFERENCE_MAX_AGE_MS
#define WD_FAST_REFERENCE_MAX_AGE_MS (10u)
#endif
#ifndef WD_FAST_POSITION_TOLERANCE_RAD
#define WD_FAST_POSITION_TOLERANCE_RAD (0.05f)
#endif
#ifndef WD_FAST_VELOCITY_TOLERANCE_RADPS
#define WD_FAST_VELOCITY_TOLERANCE_RADPS (0.20f)
#endif
#ifndef WD_FAST_POSITION_WRAP_PERIOD_RAD
/* Confirmed motor mechanical/encoder phase period. The 0x02 field's numeric
 * coding range is wider, but that does not change the physical 2*pi phase. */
#define WD_FAST_POSITION_WRAP_PERIOD_RAD (6.283185307179586f)
#endif
#ifndef WD_FAST_REFERENCE_POSITION_WRAP_PERIOD_RAD
/* mech_pos is a single-turn absolute encoder phase, hence a 2*pi wrap. */
#define WD_FAST_REFERENCE_POSITION_WRAP_PERIOD_RAD \
  WD_FAST_POSITION_WRAP_PERIOD_RAD
#endif
#ifndef WD_FAST_REFERENCE_MATCH_COUNT
#define WD_FAST_REFERENCE_MATCH_COUNT (3u)
#endif
#ifndef WD_FAST_REFERENCE_MISMATCH_COUNT
/* Saturation point for the diagnostic mismatch counter. */
#define WD_FAST_REFERENCE_MISMATCH_COUNT (3u)
#endif
#ifndef WD_FAST_POSITION_EXCITATION_RAD
#define WD_FAST_POSITION_EXCITATION_RAD (0.10f)
#endif
#ifndef WD_FAST_VELOCITY_EXCITATION_RADPS
#define WD_FAST_VELOCITY_EXCITATION_RADPS (0.50f)
#endif

typedef struct {
  uint32_t operation_rx_count;
  uint32_t last_operation_ms;
  uint32_t rate_window_start_ms;
  uint32_t rate_window_start_count;
  uint32_t max_operation_gap_ms;
  float operation_position_rad;
  float operation_velocity_radps;
  float position_error_rad;
  float velocity_error_radps;
  float measured_rate_hz;
  float last_reference_position_rad;
  float continuous_reference_position_rad;
  float reference_position_min_rad;
  float reference_position_max_rad;
  float reference_velocity_max_abs_radps;
  uint8_t rate_window_started;
  uint8_t rate_valid;
  uint8_t position_reference_ok;
  uint8_t velocity_reference_ok;
  uint8_t position_match_count;
  uint8_t velocity_match_count;
  uint8_t position_mismatch_count;
  uint8_t velocity_mismatch_count;
  uint8_t position_reference_seen;
} WdFastFeedbackState;

void wd_fast_feedback_init(WdFastFeedbackState *state);
void wd_fast_feedback_note_operation(WdFastFeedbackState *state,
                                     uint32_t now_ms,
                                     float position_rad,
                                     float velocity_radps);
void wd_fast_feedback_check_position(WdFastFeedbackState *state,
                                     uint32_t now_ms,
                                     float reference_position_rad);
void wd_fast_feedback_check_velocity(WdFastFeedbackState *state,
                                     uint32_t now_ms,
                                     float reference_velocity_radps);
float wd_fast_feedback_position_delta(float current_rad, float previous_rad);
int wd_fast_feedback_qualified(const WdFastFeedbackState *state);
int wd_fast_feedback_ready_with_stale_limit(const WdFastFeedbackState *state,
                                            uint32_t now_ms,
                                            uint32_t stale_limit_ms);
int wd_fast_feedback_ready(const WdFastFeedbackState *state, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* INC_WD_FAST_FEEDBACK_H_ */
