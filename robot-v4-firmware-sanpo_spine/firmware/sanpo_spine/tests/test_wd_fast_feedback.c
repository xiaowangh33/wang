#include "wd_fast_feedback.h"

#include <assert.h>
#include <stdio.h>

static void feed_rate(WdFastFeedbackState *state,
                      uint32_t interval_ms,
                      uint32_t samples) {
  uint32_t i;
  for (i = 0u; i < samples; ++i) {
    wd_fast_feedback_note_operation(state, i * interval_ms, 1.0f, 0.0f);
  }
}

static uint32_t qualify_references(WdFastFeedbackState *state, uint32_t now_ms) {
  static const float positions[WD_FAST_REFERENCE_MATCH_COUNT] = {
      0.94f, 1.00f, 1.06f};
  static const float velocities[WD_FAST_REFERENCE_MATCH_COUNT] = {
      -0.60f, 0.60f, 0.70f};
  uint32_t i;
  for (i = 0u; i < WD_FAST_REFERENCE_MATCH_COUNT; ++i) {
    const uint32_t sample_ms = now_ms + (2u * i);
    wd_fast_feedback_note_operation(state,
                                    sample_ms,
                                    positions[i],
                                    velocities[i]);
    wd_fast_feedback_check_position(state, sample_ms, positions[i] + 0.01f);
    wd_fast_feedback_check_velocity(state, sample_ms, velocities[i] + 0.01f);
  }
  return now_ms + (2u * (WD_FAST_REFERENCE_MATCH_COUNT - 1u));
}

static void test_500_hz_matching_feedback_passes(void) {
  WdFastFeedbackState state;
  uint32_t qualified_ms;
  wd_fast_feedback_init(&state);
  feed_rate(&state, 2u, (WD_FAST_FEEDBACK_RATE_WINDOW_MS / 2u) + 1u);
  qualified_ms = qualify_references(&state, WD_FAST_FEEDBACK_RATE_WINDOW_MS);
  assert(state.measured_rate_hz >= 499.0f);
  assert(wd_fast_feedback_ready(&state, qualified_ms) != 0);
}

static void test_125_hz_feedback_is_rejected(void) {
  WdFastFeedbackState state;
  wd_fast_feedback_init(&state);
  feed_rate(&state, 8u, (WD_FAST_FEEDBACK_RATE_WINDOW_MS / 8u) + 2u);
  (void)qualify_references(&state, WD_FAST_FEEDBACK_RATE_WINDOW_MS + 8u);
  assert(state.measured_rate_hz < 126.0f);
  assert(wd_fast_feedback_ready(
             &state, WD_FAST_FEEDBACK_RATE_WINDOW_MS + 8u) == 0);
}

static void test_minimum_rate_threshold(void) {
  WdFastFeedbackState state;
  uint32_t i;
  const uint32_t valid_intervals =
      (WD_FAST_FEEDBACK_MIN_HZ * WD_FAST_FEEDBACK_RATE_WINDOW_MS) / 1000u;
  const uint32_t invalid_intervals = valid_intervals - 1u;

  wd_fast_feedback_init(&state);
  for (i = 0u; i <= valid_intervals; ++i) {
    wd_fast_feedback_note_operation(
        &state,
        (i * WD_FAST_FEEDBACK_RATE_WINDOW_MS) / valid_intervals,
        1.0f,
        0.0f);
  }
  assert(state.measured_rate_hz >= (float)WD_FAST_FEEDBACK_MIN_HZ);
  assert(state.rate_valid != 0u);

  wd_fast_feedback_init(&state);
  for (i = 0u; i <= invalid_intervals; ++i) {
    wd_fast_feedback_note_operation(
        &state,
        (i * WD_FAST_FEEDBACK_RATE_WINDOW_MS) / invalid_intervals,
        1.0f,
        0.0f);
  }
  assert(state.measured_rate_hz < (float)WD_FAST_FEEDBACK_MIN_HZ);
  assert(state.rate_valid == 0u);
}

static void test_no_operation_status_is_rejected(void) {
  WdFastFeedbackState state;
  wd_fast_feedback_init(&state);
  wd_fast_feedback_check_position(&state, 100u, 0.0f);
  wd_fast_feedback_check_velocity(&state, 100u, 0.0f);
  assert(state.operation_rx_count == 0u);
  assert(wd_fast_feedback_ready(&state, 100u) == 0);
}

static void test_stationary_matching_channels_pass_static_readiness(void) {
  WdFastFeedbackState state;
  uint32_t i;
  wd_fast_feedback_init(&state);
  feed_rate(&state, 2u, (WD_FAST_FEEDBACK_RATE_WINDOW_MS / 2u) + 1u);
  for (i = 0u; i < WD_FAST_REFERENCE_MATCH_COUNT; ++i) {
    wd_fast_feedback_check_position(
        &state, WD_FAST_FEEDBACK_RATE_WINDOW_MS, 1.0f);
    wd_fast_feedback_check_velocity(
        &state, WD_FAST_FEEDBACK_RATE_WINDOW_MS, 0.0f);
  }
  assert(state.position_match_count == WD_FAST_REFERENCE_MATCH_COUNT);
  assert(state.velocity_match_count == WD_FAST_REFERENCE_MATCH_COUNT);
  assert(state.reference_position_max_rad == state.reference_position_min_rad);
  assert(state.reference_velocity_max_abs_radps == 0.0f);
  assert(wd_fast_feedback_ready(
             &state, WD_FAST_FEEDBACK_RATE_WINDOW_MS) != 0);
}

static void test_dynamic_mismatch_is_diagnostic_only_after_qualification(void) {
  WdFastFeedbackState state;
  uint32_t i;
  uint32_t qualified_ms;
  wd_fast_feedback_init(&state);
  feed_rate(&state, 2u, (WD_FAST_FEEDBACK_RATE_WINDOW_MS / 2u) + 1u);
  qualified_ms = qualify_references(&state, WD_FAST_FEEDBACK_RATE_WINDOW_MS);
  assert(wd_fast_feedback_ready(&state, qualified_ms) != 0);

  wd_fast_feedback_check_position(&state, qualified_ms, 2.0f);
  assert(state.position_mismatch_count == 1u);
  assert(wd_fast_feedback_ready(&state, qualified_ms) != 0);

  wd_fast_feedback_check_position(
      &state, qualified_ms, state.operation_position_rad);
  assert(state.position_mismatch_count == 0u);
  assert(wd_fast_feedback_ready(&state, qualified_ms) != 0);

  for (i = 0u; i < WD_FAST_REFERENCE_MISMATCH_COUNT; ++i) {
    wd_fast_feedback_check_velocity(&state, qualified_ms, 2.0f);
  }
  assert(state.velocity_mismatch_count == WD_FAST_REFERENCE_MISMATCH_COUNT);
  assert(state.velocity_match_count == WD_FAST_REFERENCE_MATCH_COUNT);
  assert(wd_fast_feedback_ready(&state, qualified_ms) != 0);

  qualified_ms = qualify_references(&state, qualified_ms);
  assert(wd_fast_feedback_ready(&state, qualified_ms + 6u) == 0);
}

static void test_initial_mismatch_cannot_establish_qualification(void) {
  WdFastFeedbackState state;
  uint32_t i;

  wd_fast_feedback_init(&state);
  feed_rate(&state, 2u, (WD_FAST_FEEDBACK_RATE_WINDOW_MS / 2u) + 1u);
  for (i = 0u; i < WD_FAST_REFERENCE_MATCH_COUNT; ++i) {
    wd_fast_feedback_check_position(
        &state, WD_FAST_FEEDBACK_RATE_WINDOW_MS, 2.0f);
    wd_fast_feedback_check_velocity(
        &state, WD_FAST_FEEDBACK_RATE_WINDOW_MS, 2.0f);
  }
  assert(state.position_match_count == 0u);
  assert(state.velocity_match_count == 0u);
  assert(wd_fast_feedback_ready(
             &state, WD_FAST_FEEDBACK_RATE_WINDOW_MS) == 0);
}

static void test_operation_status_wrap_unwraps_at_2pi_phase(void) {
  WdFastFeedbackState state;
  const float pmax = 12.566370614359172f;
  const float before_wrap = pmax - 0.01f;
  const float after_wrap = -pmax + 0.02f;
  uint32_t i;

  wd_fast_feedback_init(&state);
  state.operation_rx_count = 1u;
  state.last_operation_ms = 100u;
  state.operation_position_rad = before_wrap;
  state.position_match_count = WD_FAST_REFERENCE_MATCH_COUNT;
  state.position_reference_ok = 1u;

  for (i = 0u; i < WD_FAST_REFERENCE_MISMATCH_COUNT; ++i) {
    wd_fast_feedback_check_position(&state, 100u, -pmax + 0.01f);
  }
  assert(state.position_error_rad < 0.03f);
  assert(state.position_mismatch_count == 0u);
  assert(state.position_match_count == WD_FAST_REFERENCE_MATCH_COUNT);
  assert(wd_fast_feedback_position_delta(after_wrap, before_wrap) > 0.029f);
  assert(wd_fast_feedback_position_delta(after_wrap, before_wrap) < 0.031f);
}

static void test_mechanical_reference_uses_2pi_phase(void) {
  WdFastFeedbackState state;
  const float two_pi = WD_FAST_REFERENCE_POSITION_WRAP_PERIOD_RAD;
  const float pi = 0.5f * two_pi;

  wd_fast_feedback_init(&state);
  state.operation_rx_count = 1u;
  state.last_operation_ms = 100u;
  state.operation_position_rad = 8.333000f;

  /* The same single-turn mech_pos may arrive one turn lower than the 0x02
   * encoded position. It must still qualify as the same physical phase. */
  wd_fast_feedback_check_position(&state, 100u, 8.333000f - two_pi);
  assert(state.position_error_rad < 1.0e-5f);
  assert(state.position_match_count == 1u);

  /* A mech_pos sample crossing its +/-pi representation seam is only a
   * 0.06-rad physical movement, not a false 2*pi excitation. */
  wd_fast_feedback_init(&state);
  state.operation_rx_count = 1u;
  state.last_operation_ms = 100u;
  state.operation_position_rad = pi - 0.03f;
  wd_fast_feedback_check_position(&state, 100u, pi - 0.03f);
  state.operation_position_rad = -pi + 0.03f;
  wd_fast_feedback_check_position(&state, 100u, -pi + 0.03f);
  assert((state.reference_position_max_rad -
          state.reference_position_min_rad) < 0.07f);
}

static void test_stale_control_sample_is_distinct_from_stop_timeout(void) {
  WdFastFeedbackState state;
  uint32_t qualified_ms;

  wd_fast_feedback_init(&state);
  feed_rate(&state, 2u, (WD_FAST_FEEDBACK_RATE_WINDOW_MS / 2u) + 1u);
  qualified_ms = qualify_references(&state, WD_FAST_FEEDBACK_RATE_WINDOW_MS);
  assert(wd_fast_feedback_qualified(&state) != 0);
  assert(wd_fast_feedback_ready(&state, qualified_ms + 6u) == 0);
  assert(wd_fast_feedback_ready_with_stale_limit(
             &state, qualified_ms + 6u, WD_FAST_FEEDBACK_STOP_MS) != 0);
  assert(wd_fast_feedback_ready_with_stale_limit(
             &state,
             qualified_ms + WD_FAST_FEEDBACK_STOP_MS + 1u,
             WD_FAST_FEEDBACK_STOP_MS) == 0);
}

static void test_max_operation_gap_is_recorded(void) {
  WdFastFeedbackState state;

  wd_fast_feedback_init(&state);
  wd_fast_feedback_note_operation(&state, 10u, 0.0f, 0.0f);
  wd_fast_feedback_note_operation(&state, 12u, 0.0f, 0.0f);
  wd_fast_feedback_note_operation(&state, 18u, 0.0f, 0.0f);
  assert(state.max_operation_gap_ms == 6u);
}

static void test_established_velocity_scale_tolerates_async_dynamic_sample(void) {
  WdFastFeedbackState state;
  uint32_t qualified_ms;
  uint32_t i;

  wd_fast_feedback_init(&state);
  feed_rate(&state, 2u, (WD_FAST_FEEDBACK_RATE_WINDOW_MS / 2u) + 1u);
  qualified_ms = qualify_references(&state, WD_FAST_FEEDBACK_RATE_WINDOW_MS);
  assert(wd_fast_feedback_ready(&state, qualified_ms) != 0);

  /* The slow READ_PARAM sample is 0.30 rad/s away from the latest 0x02
   * sample: too large for initial scale proof, but expected after the scale
   * is established and the motor is accelerating. */
  for (i = 0u; i < WD_FAST_REFERENCE_MISMATCH_COUNT; ++i) {
    wd_fast_feedback_check_velocity(&state, qualified_ms, 1.0f);
  }
  assert(state.velocity_match_count == WD_FAST_REFERENCE_MATCH_COUNT);
  assert(state.velocity_mismatch_count == WD_FAST_REFERENCE_MISMATCH_COUNT);
  assert(wd_fast_feedback_ready(&state, qualified_ms) != 0);
}

int main(void) {
  test_500_hz_matching_feedback_passes();
  test_125_hz_feedback_is_rejected();
  test_minimum_rate_threshold();
  test_no_operation_status_is_rejected();
  test_stationary_matching_channels_pass_static_readiness();
  test_dynamic_mismatch_is_diagnostic_only_after_qualification();
  test_initial_mismatch_cannot_establish_qualification();
  test_operation_status_wrap_unwraps_at_2pi_phase();
  test_mechanical_reference_uses_2pi_phase();
  test_stale_control_sample_is_distinct_from_stop_timeout();
  test_max_operation_gap_is_recorded();
  test_established_velocity_scale_tolerates_async_dynamic_sample();
  puts("wd_fast_feedback tests passed");
  return 0;
}
