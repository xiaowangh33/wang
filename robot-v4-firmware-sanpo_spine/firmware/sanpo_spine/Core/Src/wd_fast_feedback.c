#include "wd_fast_feedback.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static uint8_t wd_fast_increment_match(uint8_t value) {
  if (value < WD_FAST_REFERENCE_MATCH_COUNT) {
    ++value;
  }
  return value;
}

static void wd_fast_note_mismatch(uint8_t *match_count,
                                  uint8_t *mismatch_count,
                                  uint8_t *reference_ok) {
  if (match_count == NULL || mismatch_count == NULL || reference_ok == NULL) {
    return;
  }

  if (*reference_ok != 0u &&
      *match_count >= WD_FAST_REFERENCE_MATCH_COUNT) {
    /*
     * The slow READ_PARAM reply and 500 Hz operation-status frame have no
     * shared acquisition timestamp. Once their scale has been proven during
     * controlled startup qualification, a difference observed under dynamic
     * acceleration is diagnostic only: retain the qualified latch and expose
     * the latest error/counter, but never revoke live control from this
     * asynchronous comparison. Runtime safety is instead enforced by fresh
     * 500 Hz generations, measured rate, motor faults and hard command limits.
     */
    if (*mismatch_count < WD_FAST_REFERENCE_MISMATCH_COUNT) {
      ++(*mismatch_count);
    }
    return;
  }

  /* Initial qualification still requires uninterrupted matching samples. */
  *match_count = 0u;
  *mismatch_count = 0u;
  *reference_ok = 0u;
}

void wd_fast_feedback_init(WdFastFeedbackState *state) {
  if (state != NULL) {
    memset(state, 0, sizeof(*state));
  }
}

static float wd_fast_wrapped_delta(float current_rad,
                                   float previous_rad,
                                   float period) {
  float delta;
  const float half_period = 0.5f * period;

  if (!isfinite(current_rad) || !isfinite(previous_rad) || period <= 0.0f) {
    return current_rad - previous_rad;
  }
  delta = fmodf(current_rad - previous_rad, period);
  if (delta > half_period) {
    delta -= period;
  } else if (delta < -half_period) {
    delta += period;
  }
  return delta;
}

float wd_fast_feedback_position_delta(float current_rad, float previous_rad) {
  return wd_fast_wrapped_delta(current_rad,
                               previous_rad,
                               WD_FAST_POSITION_WRAP_PERIOD_RAD);
}

void wd_fast_feedback_note_operation(WdFastFeedbackState *state,
                                     uint32_t now_ms,
                                     float position_rad,
                                     float velocity_radps) {
  uint32_t operation_gap_ms;
  uint32_t elapsed_ms;
  uint32_t received;

  if (state == NULL || !isfinite(position_rad) || !isfinite(velocity_radps)) {
    return;
  }

  if (state->operation_rx_count != 0u && state->last_operation_ms != 0u) {
    operation_gap_ms = now_ms - state->last_operation_ms;
    if (operation_gap_ms > state->max_operation_gap_ms) {
      state->max_operation_gap_ms = operation_gap_ms;
    }
  }
  ++state->operation_rx_count;
  state->last_operation_ms = now_ms;
  state->operation_position_rad = position_rad;
  state->operation_velocity_radps = velocity_radps;

  if (state->rate_window_started == 0u) {
    state->rate_window_started = 1u;
    state->rate_window_start_ms = now_ms;
    state->rate_window_start_count = state->operation_rx_count;
    return;
  }

  elapsed_ms = now_ms - state->rate_window_start_ms;
  if (elapsed_ms < WD_FAST_FEEDBACK_RATE_WINDOW_MS) {
    return;
  }

  received = state->operation_rx_count - state->rate_window_start_count;
  state->measured_rate_hz =
      ((float)received * 1000.0f) / (float)elapsed_ms;
  state->rate_valid =
      ((uint64_t)received * 1000u >=
       (uint64_t)WD_FAST_FEEDBACK_MIN_HZ * elapsed_ms) ? 1u : 0u;
  state->rate_window_start_ms = now_ms;
  state->rate_window_start_count = state->operation_rx_count;
}

void wd_fast_feedback_check_position(WdFastFeedbackState *state,
                                     uint32_t now_ms,
                                     float reference_position_rad) {
  if (state == NULL || state->operation_rx_count == 0u ||
      !isfinite(reference_position_rad) ||
      (now_ms - state->last_operation_ms) > WD_FAST_REFERENCE_MAX_AGE_MS) {
    return;
  }

  /* The operation-status field has a wider encoded range, but it describes
   * the same physical single-turn phase as mech_pos. Their cross-check must
   * therefore accept aliases separated by any integer multiple of 2*pi. */
  state->position_error_rad = fabsf(wd_fast_wrapped_delta(
      state->operation_position_rad,
      reference_position_rad,
      WD_FAST_REFERENCE_POSITION_WRAP_PERIOD_RAD));
  if (state->position_reference_seen == 0u) {
    state->last_reference_position_rad = reference_position_rad;
    state->continuous_reference_position_rad = reference_position_rad;
    state->reference_position_min_rad =
        state->continuous_reference_position_rad;
    state->reference_position_max_rad =
        state->continuous_reference_position_rad;
    state->position_reference_seen = 1u;
  } else {
    state->continuous_reference_position_rad += wd_fast_wrapped_delta(
        reference_position_rad,
        state->last_reference_position_rad,
        WD_FAST_REFERENCE_POSITION_WRAP_PERIOD_RAD);
    state->last_reference_position_rad = reference_position_rad;
    if (state->continuous_reference_position_rad <
        state->reference_position_min_rad) {
      state->reference_position_min_rad =
          state->continuous_reference_position_rad;
    } else if (state->continuous_reference_position_rad >
               state->reference_position_max_rad) {
      state->reference_position_max_rad =
          state->continuous_reference_position_rad;
    }
  }
  if (state->position_error_rad <= WD_FAST_POSITION_TOLERANCE_RAD) {
    state->position_mismatch_count = 0u;
    state->position_match_count =
        wd_fast_increment_match(state->position_match_count);
    state->position_reference_ok = 1u;
  } else {
    wd_fast_note_mismatch(&state->position_match_count,
                          &state->position_mismatch_count,
                          &state->position_reference_ok);
  }
}

void wd_fast_feedback_check_velocity(WdFastFeedbackState *state,
                                     uint32_t now_ms,
                                     float reference_velocity_radps) {
  if (state == NULL || state->operation_rx_count == 0u ||
      !isfinite(reference_velocity_radps) ||
      (now_ms - state->last_operation_ms) > WD_FAST_REFERENCE_MAX_AGE_MS) {
    return;
  }

  state->velocity_error_radps =
      fabsf(state->operation_velocity_radps - reference_velocity_radps);
  if (fabsf(reference_velocity_radps) >
      state->reference_velocity_max_abs_radps) {
    state->reference_velocity_max_abs_radps =
        fabsf(reference_velocity_radps);
  }
  if (state->velocity_error_radps <= WD_FAST_VELOCITY_TOLERANCE_RADPS) {
    state->velocity_mismatch_count = 0u;
    state->velocity_match_count =
        wd_fast_increment_match(state->velocity_match_count);
    state->velocity_reference_ok = 1u;
  } else {
    wd_fast_note_mismatch(&state->velocity_match_count,
                          &state->velocity_mismatch_count,
                          &state->velocity_reference_ok);
  }
}

int wd_fast_feedback_qualified(const WdFastFeedbackState *state) {
  if (state == NULL) {
    return 0;
  }
  /* Static startup qualification intentionally sends zero commands. It proves
   * fresh 500 Hz delivery and agreement with READ_PARAM at the current pose;
   * the excitation fields remain diagnostics from any later/manual motion. */
  return (state->rate_valid != 0u &&
          state->position_match_count >= WD_FAST_REFERENCE_MATCH_COUNT &&
          state->velocity_match_count >= WD_FAST_REFERENCE_MATCH_COUNT) ?
      1 : 0;
}

int wd_fast_feedback_ready_with_stale_limit(const WdFastFeedbackState *state,
                                            uint32_t now_ms,
                                            uint32_t stale_limit_ms) {
  return (state != NULL &&
          wd_fast_feedback_qualified(state) != 0 &&
          (now_ms - state->last_operation_ms) <= stale_limit_ms) ? 1 : 0;
}

int wd_fast_feedback_ready(const WdFastFeedbackState *state, uint32_t now_ms) {
  return wd_fast_feedback_ready_with_stale_limit(
      state, now_ms, WD_FAST_FEEDBACK_STALE_MS);
}
