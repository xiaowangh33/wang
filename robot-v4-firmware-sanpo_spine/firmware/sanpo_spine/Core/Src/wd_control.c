#include "wd_control.h"

#include "can.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "wd_actuator_units.h"
#include "wd_can_scheduler.h"
#include "wd_fast_feedback.h"
#include "wd_motor_calibration.h"
#include "wd_protocol.h"
#include "wd_robstride_motion.h"
#include "wd_safety.h"

#include <string.h>

#define WD_DEFAULT_COMMAND_TIMEOUT_MS (300u)
#define WD_FEEDBACK_PERIOD_MS (20u)
#define WD_CAN_FEEDBACK_STALE_MS (200u)
/* Two 1 Mbit/s buses deliver about four 500 Hz operation-status frames per
 * millisecond in total. 64 slots preserve roughly 15 ms of burst headroom if
 * the 1 kHz task is briefly preempted; freshness checks still reject old
 * samples, so the larger queue does not authorize stale-state PD. */
#define WD_CAN_RX_QUEUE_SIZE (64u)
#define WD_ROBSTRIDE_COMM_GET_DEVICE_ID (0x00u)
#define WD_ROBSTRIDE_COMM_OPERATION_STATUS (0x02u)
#define WD_ROBSTRIDE_COMM_START (0x03u)
#define WD_ROBSTRIDE_COMM_ENABLE_DISABLE (0x04u)
#define WD_ROBSTRIDE_COMM_READ_PARAM (0x11u)
#define WD_ROBSTRIDE_COMM_WRITE_PARAM (0x12u)
#define WD_ROBSTRIDE_HOST_ID (0xFDu)
#define WD_ROBSTRIDE_PARAM_RUN_MODE (0x7005u)
#define WD_ROBSTRIDE_PARAM_IQ_REF (0x7006u)
#define WD_ROBSTRIDE_PARAM_TORQUE_LIMIT (0x700Bu)
#define WD_ROBSTRIDE_PARAM_I_LIMIT (0x7018u)
#define WD_ROBSTRIDE_PARAM_MECH_POS (0x7019u)
#define WD_ROBSTRIDE_PARAM_MECH_VEL (0x701Bu)
#define WD_ROBSTRIDE_PARAM_VBUS (0x701Cu)
#define WD_READONLY_POLL_PERIOD_MS (5u)
#define WD_MOTOR_TELEMETRY_POLL_PERIOD_MS (3000u)
#define WD_MOTOR_TELEMETRY_POLL_STEP_MS (5u)
#define WD_DEVICE_DISCOVERY_RETRY_MS (250u)
#define WD_MOTORS_PER_CAN_BUS (4u)
#define WD_LOCAL_CAN_BUS_COUNT (2u)
#define WD_READONLY_POLL_MOTOR_COUNT WD_MOTORS_PER_CAN_BUS
#define WD_READONLY_POLL_PARAM_COUNT (2u)
#define WD_LIVE_LOCAL_MOTOR_COUNT (WD_LOCAL_CAN_BUS_COUNT * WD_MOTORS_PER_CAN_BUS)
#define WD_LOCAL_LOGICAL_FIRST_INDEX (WD_CAN_BUS_BASE * WD_MOTORS_PER_CAN_BUS)
#define WD_LOCAL_LOGICAL_END_INDEX (WD_LOCAL_LOGICAL_FIRST_INDEX + WD_LIVE_LOCAL_MOTOR_COUNT)
#define WD_LIVE_ENABLE_STEP_MS (20u)
#define WD_LIVE_TX_PERIOD_MS (1u)
#define WD_LIVE_RUN_MODE_MOTION (0u)
#define WD_LIVE_RUN_MODE_CURRENT (3u)
#define WD_OPERATION_POSITION_MAX_RAD (12.566370614359172f) /* 4*pi */
#define WD_RS06_P_MAX WD_OPERATION_POSITION_MAX_RAD
#define WD_RS06_V_MAX (50.0f)
#define WD_RS06_T_MAX (36.0f)
#define WD_RS01_P_MAX WD_OPERATION_POSITION_MAX_RAD
#define WD_RS01_V_MAX (44.0f)
#define WD_RS01_T_MAX (17.0f)
#define WD_CAN_TX_TAG_NONE (0xFFu)
#define WD_QUALIFICATION_TORQUE_MAX_NM (0.60f)
#define WD_QUALIFICATION_VELOCITY_MAX_RADPS (1.00f)
#define WD_QUALIFICATION_WHEEL_KD (0.40f)
#define WD_QUALIFICATION_POSITION_MAX_RAD (0.50f)
#define WD_QUALIFICATION_COMMAND_EPSILON (0.0001f)

#ifndef WD_CAN_BUS_BASE
#define WD_CAN_BUS_BASE (0u)
#endif

#ifndef WD_BENCH_REMAP_CAN2_TO_LOGICAL0
#define WD_BENCH_REMAP_CAN2_TO_LOGICAL0 (0u)
#endif

#ifndef WD_ALLOW_LIVE_CAN_CONTROL
#define WD_ALLOW_LIVE_CAN_CONTROL (0u)
#endif

#ifndef WD_ENABLE_CALIBRATION_READONLY
#define WD_ENABLE_CALIBRATION_READONLY (1u)
#endif

#if WD_ENABLE_CALIBRATION_READONLY
#define WD_READONLY_CONTROL_FLAG_MASK \
  (WD_CONTROL_FLAG_ESTOP | WD_CONTROL_FLAG_READONLY_POLL | \
   WD_CONTROL_FLAG_READONLY_POLL_CAN2 | \
   WD_CONTROL_FLAG_CALIBRATION_READONLY)
#else
#define WD_READONLY_CONTROL_FLAG_MASK \
  (WD_CONTROL_FLAG_ESTOP | WD_CONTROL_FLAG_READONLY_POLL | \
   WD_CONTROL_FLAG_READONLY_POLL_CAN2)
#endif

enum {
  WD_LIVE_STAGE_IDLE = 0,
  WD_LIVE_STAGE_INIT,
  WD_LIVE_STAGE_STOP_BEFORE_MODE,
  WD_LIVE_STAGE_RUN_MODE,
  WD_LIVE_STAGE_COMMAND_LIMIT,
  WD_LIVE_STAGE_ENABLE,
  WD_LIVE_STAGE_START,
  WD_LIVE_STAGE_ZERO_COMMAND,
  WD_LIVE_STAGE_READY,
};

enum {
  WD_LIVE_STOP_NONE = 0,
  WD_LIVE_STOP_ZERO_COMMAND,
  WD_LIVE_STOP_DISABLE,
};

typedef struct {
  uint32_t can_id;
  uint8_t data[8];
  uint8_t dlc;
  uint8_t bus_index;
} WdCanRxFrame;

typedef struct {
  uint8_t initialized;
  uint8_t active;
  uint8_t dry_run;
  WdLimitMode limit_mode;
  uint32_t status_flags;
  uint32_t control_flags;
  uint32_t last_rx_ms;
  uint32_t last_setpoint_ms;
  uint32_t last_rx_seq;
  uint32_t last_setpoint_seq;
  uint32_t command_timeout_ms;
  uint32_t control_ticks;
  uint32_t last_hz_tick_ms;
  uint32_t last_hz_control_ticks;
  float actual_control_hz;
  uint32_t tx_seq;
  uint32_t last_feedback_tx_ms;
  uint32_t sanitize_status_flags;
  uint32_t can_rx_frames;
  uint32_t can_rx_bad_frames;
  uint32_t can_rx_overflows;
  uint32_t can_tx_errors;
  uint32_t last_can_id;
  uint32_t last_can_data0_3;
  uint32_t last_can_data4_7;
  uint32_t last_can_meta;
  uint32_t command_status_flags;
  uint32_t command_compute_ticks;
  uint32_t last_readonly_poll_ms;
  uint32_t readonly_poll_step;
  uint32_t readonly_discovery_step;
  uint8_t readonly_discovery_bus;
  uint32_t last_live_precheck_poll_ms;
  uint32_t live_precheck_poll_step;
  uint32_t last_motor_telemetry_poll_ms;
  uint32_t last_motor_telemetry_attempt_ms;
  uint32_t motor_telemetry_poll_cursor;
  uint32_t supply_voltage_valid_mask;
  float supply_voltage_v[WD_MOTOR_COUNT];
  uint32_t last_live_discovery_ms;
  uint32_t live_discovery_step;
  uint32_t live_discovery_complete_ms;
  uint8_t readonly_poll_running;
  uint32_t last_live_tx_ms;
  uint32_t live_tx_cursor;
  uint32_t live_enable_cursor;
  uint32_t live_stop_cursor;
  uint8_t live_enable_stage[WD_MOTOR_COUNT];
  uint8_t live_stop_stage[WD_MOTOR_COUNT];
  uint8_t live_enabled[WD_MOTOR_COUNT];
  uint32_t last_can_feedback_ms[WD_MOTOR_COUNT];
  uint32_t last_position_feedback_ms[WD_MOTOR_COUNT];
  uint32_t last_velocity_feedback_ms[WD_MOTOR_COUNT];
  float reference_motor_position[WD_MOTOR_COUNT];
  float last_operation_position_raw[WD_MOTOR_COUNT];
  float continuous_motor_position[WD_MOTOR_COUNT];
  float aligned_raw_position[WD_MOTOR_COUNT];
  float joint_torque_cmd_nm[WD_MOTOR_COUNT];
  float motor_torque_cmd_nm[WD_MOTOR_COUNT];
  float motor_velocity_cmd_radps[WD_MOTOR_COUNT];
  uint8_t mech_position_valid[WD_MOTOR_COUNT];
  uint8_t mech_velocity_valid[WD_MOTOR_COUNT];
  uint8_t operation_position_valid[WD_MOTOR_COUNT];
  uint8_t fast_qualification_started;
  uint8_t fast_feedback_armed;
  WdFastFeedbackState fast_feedback[WD_MOTOR_COUNT];
  uint32_t fast_feedback_generation[WD_MOTOR_COUNT];
  uint32_t last_command_feedback_generation[WD_MOTOR_COUNT];
  uint32_t last_command_compute_ms[WD_MOTOR_COUNT];
  volatile uint32_t observation_lock;
  uint32_t observation_seq;
  uint32_t observation_time_ms;
  uint16_t observation_max_sample_age_ms;
  uint16_t observation_flags;
  uint16_t observation_sample_age_ms[WD_MOTOR_COUNT];
  WdJointFeedback observation_feedback[WD_MOTOR_COUNT];
  uint8_t observation_tx_pending;
  volatile uint8_t pending_setpoint_valid;
  uint32_t pending_setpoint_seq;
  WdSetpointPayload pending_setpoint;
  WdCanScheduler live_command_scheduler;
  uint32_t live_command_tx_queued[WD_MOTOR_COUNT];
  uint32_t live_command_tx_deferred[WD_MOTOR_COUNT];
  uint32_t live_stop_reason_flags;
  uint32_t live_stop_motor_mask;
  uint32_t live_stop_event_count;
  uint32_t live_stop_trigger_time_ms;
  uint32_t live_stop_fast_valid_mask;
  uint32_t live_stop_fault_mask;
  uint8_t live_stop_fast_age_ms[WD_MOTOR_COUNT];
  float live_stop_joint_torque_cmd_nm[WD_MOTOR_COUNT];
  uint8_t overspeed_consecutive_count[WD_MOTOR_COUNT];
  uint32_t overspeed_last_operation_rx_count[WD_MOTOR_COUNT];
  WdProtocolParser parser;
  WdSafetyRuntimeConfig runtime_config;
  WdSetpointPayload setpoint;
  WdJointFeedback feedback[WD_MOTOR_COUNT];
} WdControlState;

static WdControlState g_wd;
static uint8_t g_tx_buffer[WD_PROTOCOL_MAX_PACKET];
/*
 * The CDC TX task has a 1024-byte stack. WdFeedbackPayload is itself 1172
 * bytes, so placing it on that stack would overflow before accounting for
 * call frames. Keep the single-producer payload in static RAM instead.
 */
static WdFeedbackPayload g_feedback_payload;
static WdCanRxFrame g_can_rx_queue[WD_CAN_RX_QUEUE_SIZE];
static volatile uint32_t g_can_rx_head;
static volatile uint32_t g_can_rx_tail;
static volatile uint32_t g_can_rx_overflows;
static volatile uint32_t g_live_command_tx_completed[WD_MOTOR_COUNT];
static volatile uint8_t g_can_tx_mailbox_tag[WD_LOCAL_CAN_BUS_COUNT][3];

static int wd_control_can_local_bus(const CAN_HandleTypeDef *hcan,
                                    uint32_t *local_bus) {
  if (hcan == NULL || local_bus == NULL) {
    return 0;
  }
  if (hcan->Instance == CAN1) {
    *local_bus = 0u;
    return 1;
  }
  if (hcan->Instance == CAN2) {
    *local_bus = 1u;
    return 1;
  }
  return 0;
}

static int wd_control_can_mailbox_index(uint32_t mailbox,
                                        uint32_t *mailbox_index) {
  if (mailbox_index == NULL) {
    return 0;
  }
  if (mailbox == CAN_TX_MAILBOX0) {
    *mailbox_index = 0u;
    return 1;
  }
  if (mailbox == CAN_TX_MAILBOX1) {
    *mailbox_index = 1u;
    return 1;
  }
  if (mailbox == CAN_TX_MAILBOX2) {
    *mailbox_index = 2u;
    return 1;
  }
  return 0;
}

void wd_control_can_tx_complete_from_isr(CAN_HandleTypeDef *hcan,
                                         uint32_t mailbox) {
  uint32_t local_bus;
  uint32_t mailbox_index;
  uint8_t tag;

  if (wd_control_can_local_bus(hcan, &local_bus) == 0 ||
      wd_control_can_mailbox_index(mailbox, &mailbox_index) == 0) {
    return;
  }
  tag = g_can_tx_mailbox_tag[local_bus][mailbox_index];
  g_can_tx_mailbox_tag[local_bus][mailbox_index] = WD_CAN_TX_TAG_NONE;
  if (tag < WD_MOTOR_COUNT) {
    ++g_live_command_tx_completed[tag];
  }
}

void wd_control_can_tx_abort_from_isr(CAN_HandleTypeDef *hcan,
                                      uint32_t mailbox) {
  uint32_t local_bus;
  uint32_t mailbox_index;

  if (wd_control_can_local_bus(hcan, &local_bus) == 0 ||
      wd_control_can_mailbox_index(mailbox, &mailbox_index) == 0) {
    return;
  }
  g_can_tx_mailbox_tag[local_bus][mailbox_index] = WD_CAN_TX_TAG_NONE;
}

static uint32_t wd_can_rx_next(uint32_t index) {
  ++index;
  if (index >= WD_CAN_RX_QUEUE_SIZE) {
    index = 0u;
  }
  return index;
}

static int wd_can_rx_pop(WdCanRxFrame *frame) {
  uint32_t tail;

  if (frame == NULL || g_can_rx_tail == g_can_rx_head) {
    return 0;
  }
  tail = g_can_rx_tail;
  *frame = g_can_rx_queue[tail];
  g_can_rx_tail = wd_can_rx_next(tail);
  return 1;
}

static uint16_t wd_be_u16(const uint8_t *data) {
  return (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

static uint16_t wd_le_u16(const uint8_t *data) {
  return (uint16_t)(((uint16_t)data[1] << 8) | (uint16_t)data[0]);
}

static uint32_t wd_le_u32(const uint8_t *data) {
  return ((uint32_t)data[0]) |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

static float wd_le_f32(const uint8_t *data) {
  float value;
  memcpy(&value, data, sizeof(value));
  return value;
}

#if WD_ALLOW_LIVE_CAN_CONTROL
static void wd_write_le_u16(uint8_t *data, uint16_t value) {
  data[0] = (uint8_t)(value & 0xFFu);
  data[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static void wd_write_le_u32(uint8_t *data, uint32_t value) {
  data[0] = (uint8_t)(value & 0xFFu);
  data[1] = (uint8_t)((value >> 8) & 0xFFu);
  data[2] = (uint8_t)((value >> 16) & 0xFFu);
  data[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static void wd_write_le_f32(uint8_t *data, float value) {
  memcpy(data, &value, sizeof(value));
}
#endif

static float wd_signed_offset_to_float(uint16_t raw, float limit) {
  return ((((float)raw) / 32767.0f) - 1.0f) * limit;
}

static float wd_motor_sign(uint32_t index) {
  return wd_motor_direction_sign(index);
}

static void wd_motor_limits(uint32_t index,
                            float *p_max,
                            float *v_max,
                            float *t_max) {
  const WdJointSafetyConfig *joint = wd_safety_joint_config(index);
  if (joint != NULL && joint->mode == WD_ACTUATOR_VELOCITY) {
    *p_max = WD_RS01_P_MAX;
    *v_max = WD_RS01_V_MAX;
    *t_max = WD_RS01_T_MAX;
  } else {
    *p_max = WD_RS06_P_MAX;
    *v_max = WD_RS06_V_MAX;
    *t_max = WD_RS06_T_MAX;
  }
}

#if WD_ALLOW_LIVE_CAN_CONTROL
static float wd_current_limit_peak_a(uint32_t index) {
  const WdJointSafetyConfig *joint = wd_safety_joint_config(index);
  const float torque_limit =
      wd_safety_effective_torque_limit(index, &g_wd.runtime_config);
  if (joint == NULL || joint->mode != WD_ACTUATOR_POSITION_PD ||
      torque_limit <= 0.0f) {
    return 0.0f;
  }
  return wd_rs06_torque_nm_to_iq_peak_a(torque_limit);
}
#endif

static void wd_control_reset_setpoint(void) {
  uint32_t i;

  memset(&g_wd.setpoint, 0, sizeof(g_wd.setpoint));
  g_wd.setpoint.control_flags = WD_CONTROL_FLAG_DRY_RUN;
  g_wd.setpoint.command_timeout_ms = WD_DEFAULT_COMMAND_TIMEOUT_MS;
  g_wd.setpoint.limit_mode = (uint8_t)WD_LIMIT_CALIBRATED_ABSOLUTE;
  wd_safety_default_modes(g_wd.setpoint.actuator_mode);

  for (i = 0u; i < WD_MOTOR_COUNT; ++i) {
    const WdJointSafetyConfig *joint = wd_safety_joint_config(i);
    if (joint != NULL && joint->mode == WD_ACTUATOR_POSITION_PD) {
      g_wd.setpoint.joint[i].kp = 0.0f;
      g_wd.setpoint.joint[i].kd = 0.0f;
    }
  }
}

static void wd_control_reset_feedback(void) {
  uint32_t i;

  memset(g_wd.feedback, 0, sizeof(g_wd.feedback));
  for (i = 0u; i < WD_MOTOR_COUNT; ++i) {
    const WdJointSafetyConfig *joint = wd_safety_joint_config(i);
    g_wd.feedback[i].actuator_mode = (uint8_t)(joint ? joint->mode : WD_ACTUATOR_POSITION_PD);
  }
}

void wd_control_init(void) {
  uint32_t bus;
  uint32_t mailbox;
  uint32_t motor;

  memset(&g_wd, 0, sizeof(g_wd));
  memset((void *)g_live_command_tx_completed,
         0,
         sizeof(g_live_command_tx_completed));
  for (bus = 0u; bus < WD_LOCAL_CAN_BUS_COUNT; ++bus) {
    for (mailbox = 0u; mailbox < 3u; ++mailbox) {
      g_can_tx_mailbox_tag[bus][mailbox] = WD_CAN_TX_TAG_NONE;
    }
  }
  for (motor = 0u; motor < WD_MOTOR_COUNT; ++motor) {
    wd_fast_feedback_init(&g_wd.fast_feedback[motor]);
  }
  wd_protocol_parser_init(&g_wd.parser);
  wd_safety_default_runtime_config(&g_wd.runtime_config);
  if (wd_motor_calibration_validate_all() == 0) {
    g_wd.status_flags |= WD_STATUS_ABSOLUTE_LIMITS_NOT_CALIBRATED |
                         WD_STATUS_LIVE_CONTROL_BLOCKED;
  }
  g_wd.initialized = 1u;
  g_wd.dry_run = 1u;
  g_wd.limit_mode = WD_LIMIT_CALIBRATED_ABSOLUTE;
  g_wd.command_timeout_ms = WD_DEFAULT_COMMAND_TIMEOUT_MS;
  g_wd.last_rx_ms = HAL_GetTick();
  g_wd.last_setpoint_ms = g_wd.last_rx_ms;
  g_wd.last_hz_tick_ms = g_wd.last_rx_ms;
  wd_control_reset_setpoint();
  wd_control_reset_feedback();
}

int wd_control_is_active(void) {
  return (g_wd.initialized != 0u && g_wd.active != 0u) ? 1 : 0;
}

int wd_control_can_rx_from_isr(CAN_HandleTypeDef *hcan, uint32_t rx_fifo) {
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
  uint32_t head;
  uint32_t next;
  uint32_t bus_index;
  WdCanRxFrame *frame;

  if (hcan == NULL || g_wd.initialized == 0u || g_wd.active == 0u) {
    return 0;
  }

  if (HAL_CAN_GetRxMessage(hcan, rx_fifo, &header, data) != HAL_OK) {
    return 1;
  }

  if (hcan->Instance == CAN1) {
    bus_index = WD_CAN_BUS_BASE + 0u;
  } else if (hcan->Instance == CAN2) {
    bus_index = WD_CAN_BUS_BASE + 1u;
  } else {
    return 1;
  }

  head = g_can_rx_head;
  next = wd_can_rx_next(head);
  if (next == g_can_rx_tail) {
    ++g_can_rx_overflows;
    return 1;
  }

  frame = &g_can_rx_queue[head];
  frame->can_id = (header.IDE == CAN_ID_EXT) ? header.ExtId : header.StdId;
  frame->dlc = (header.DLC > 8u) ? 8u : (uint8_t)header.DLC;
  frame->bus_index = (uint8_t)bus_index;
  memcpy(frame->data, data, frame->dlc);
  if (frame->dlc < 8u) {
    memset(&frame->data[frame->dlc], 0, 8u - frame->dlc);
  }
  g_can_rx_head = next;
  return 1;
}

void wd_control_note_control_task_fallback(void) {
  if (g_wd.initialized == 0u) {
    wd_control_init();
  }
  g_wd.status_flags |= WD_STATUS_CONTROL_TASK_FALLBACK;
}

static void wd_control_apply_hello(uint32_t seq) {
  g_wd.active = 1u;
  g_wd.last_rx_seq = seq;
  g_wd.last_rx_ms = HAL_GetTick();
}

static void wd_control_apply_estop(uint32_t seq) {
  g_wd.active = 1u;
  g_wd.last_rx_seq = seq;
  g_wd.last_rx_ms = HAL_GetTick();
  g_wd.control_flags |= WD_CONTROL_FLAG_ESTOP;
  g_wd.status_flags |= WD_STATUS_ESTOP;
}

static void wd_control_apply_setpoint(uint32_t seq,
                                      const uint8_t *payload,
                                      uint16_t payload_size) {
  g_wd.active = 1u;
  g_wd.last_rx_seq = seq;
  g_wd.last_rx_ms = HAL_GetTick();

  if (payload_size != sizeof(WdSetpointPayload) || payload == NULL) {
    g_wd.status_flags |= WD_STATUS_BAD_PACKET;
    return;
  }

  /*
   * USB receive runs from the USB interrupt path. Publish a raw pending copy
   * and let the 1 kHz task sanitize/apply it; otherwise the task can observe a
   * torn 352-byte setpoint while it is computing all motor commands.
   */
  memcpy(&g_wd.pending_setpoint, payload, sizeof(g_wd.pending_setpoint));
  g_wd.pending_setpoint_seq = seq;
  __DMB();
  g_wd.pending_setpoint_valid = 1u;
}

static void wd_control_apply_setpoint_in_task(uint32_t seq,
                                              WdSetpointPayload *next) {
  uint32_t requested_flags;
  uint32_t live_requested;
  uint32_t live_shape_valid;

  if (next == NULL) {
    return;
  }
  if (next->command_timeout_ms == 0u) {
    next->command_timeout_ms = WD_DEFAULT_COMMAND_TIMEOUT_MS;
  }

  g_wd.sanitize_status_flags =
      wd_safety_sanitize_setpoint(next, &g_wd.runtime_config);
  g_wd.status_flags &= ~(WD_STATUS_ABSOLUTE_LIMITS_NOT_CALIBRATED |
                         WD_STATUS_SETPOINT_CLIPPED |
                         WD_STATUS_ENABLE_BLOCKED_DRY_RUN |
                         WD_STATUS_CAN_TX_ERROR |
                         WD_STATUS_LIVE_CONTROL_BLOCKED);
  g_wd.status_flags |= g_wd.sanitize_status_flags;
  g_wd.limit_mode = (WdLimitMode)next->limit_mode;
  g_wd.command_timeout_ms = next->command_timeout_ms;

  requested_flags = next->control_flags;
  live_requested = ((requested_flags & WD_CONTROL_FLAG_LIVE_CONTROL) != 0u) ?
      1u : 0u;
  live_shape_valid =
      (live_requested != 0u &&
       (requested_flags & WD_CONTROL_FLAG_ENABLE_REQUEST) != 0u &&
       (requested_flags & WD_CONTROL_FLAG_DRY_RUN) == 0u) ? 1u : 0u;

#if WD_ALLOW_LIVE_CAN_CONTROL
  if (live_shape_valid != 0u) {
    g_wd.dry_run = 0u;
    g_wd.control_flags =
        requested_flags & (WD_CONTROL_FLAG_ENABLE_REQUEST |
                           WD_CONTROL_FLAG_ESTOP |
                           WD_CONTROL_FLAG_LIVE_CONTROL |
                           WD_CONTROL_FLAG_QUALIFICATION_EXCITATION);
  } else {
    g_wd.dry_run = 1u;
    g_wd.control_flags =
        (requested_flags & WD_READONLY_CONTROL_FLAG_MASK) |
        WD_CONTROL_FLAG_DRY_RUN;
  }
#else
  g_wd.dry_run = 1u;
  g_wd.control_flags =
      (requested_flags & WD_READONLY_CONTROL_FLAG_MASK) |
      WD_CONTROL_FLAG_DRY_RUN;
#endif

  if ((requested_flags & WD_CONTROL_FLAG_ENABLE_REQUEST) != 0u &&
      g_wd.dry_run != 0u) {
    g_wd.status_flags |= WD_STATUS_ENABLE_BLOCKED_DRY_RUN;
  }
  if (live_requested != 0u && (g_wd.dry_run != 0u || live_shape_valid == 0u)) {
    g_wd.status_flags |= WD_STATUS_LIVE_CONTROL_BLOCKED;
  }
  if ((next->control_flags & WD_CONTROL_FLAG_ESTOP) != 0u) {
    g_wd.status_flags |= WD_STATUS_ESTOP;
  } else {
    g_wd.status_flags &= ~WD_STATUS_ESTOP;
  }

  memcpy(&g_wd.setpoint, next, sizeof(g_wd.setpoint));
  g_wd.last_setpoint_seq = seq;
  g_wd.last_setpoint_ms = HAL_GetTick();
}

static int wd_control_apply_pending_setpoint(void) {
  WdSetpointPayload next;
  uint32_t seq;
  uint32_t primask;
  uint8_t valid = 0u;

  primask = __get_PRIMASK();
  __disable_irq();
  if (g_wd.pending_setpoint_valid != 0u) {
    memcpy(&next, &g_wd.pending_setpoint, sizeof(next));
    seq = g_wd.pending_setpoint_seq;
    g_wd.pending_setpoint_valid = 0u;
    valid = 1u;
  }
  __DMB();
  if (primask == 0u) {
    __enable_irq();
  }

  if (valid != 0u) {
    wd_control_apply_setpoint_in_task(seq, &next);
  }
  return (valid != 0u) ? 1 : 0;
}

static void wd_control_on_packet(uint8_t type,
                                 uint32_t seq,
                                 const uint8_t *payload,
                                 uint16_t payload_size,
                                 void *context) {
  (void)context;
  (void)payload_size;

  switch (type) {
    case WD_PACKET_HELLO:
      wd_control_apply_hello(seq);
      break;
    case WD_PACKET_SETPOINT:
      wd_control_apply_setpoint(seq, payload, payload_size);
      break;
    case WD_PACKET_ESTOP:
      wd_control_apply_estop(seq);
      break;
    case WD_PACKET_CONFIG:
      g_wd.active = 1u;
      g_wd.last_rx_seq = seq;
      g_wd.last_rx_ms = HAL_GetTick();
      break;
    default:
      g_wd.status_flags |= WD_STATUS_BAD_PACKET;
      break;
  }
}

int wd_control_usb_receive(const uint8_t *data, uint32_t len) {
  int protocol_context;
  int starts_packet;
  int owns_payload;
  int decoded;

  if (g_wd.initialized == 0u) {
    wd_control_init();
  }
  if (data == NULL || len == 0u) {
    return 0;
  }

  protocol_context = wd_control_is_active() ||
                     wd_protocol_parser_has_pending(&g_wd.parser);
  starts_packet = wd_protocol_buffer_starts_like_packet(data, len);
  if (protocol_context == 0 && starts_packet == 0) {
    return 0;
  }

  owns_payload = 1;

  decoded = wd_protocol_feed(&g_wd.parser,
                             data,
                             len,
                             wd_control_on_packet,
                             NULL);
  if (decoded > 0) {
    owns_payload = 1;
  }

  if (g_wd.parser.crc_errors != 0u) {
    g_wd.status_flags |= WD_STATUS_CRC_ERROR;
  }
  if (g_wd.parser.bad_packets != 0u) {
    g_wd.status_flags |= WD_STATUS_BAD_PACKET;
  }

  return owns_payload;
}

static void wd_control_update_hz(uint32_t now_ms) {
  uint32_t dt_ms = now_ms - g_wd.last_hz_tick_ms;
  if (dt_ms >= 1000u) {
    uint32_t ticks = g_wd.control_ticks - g_wd.last_hz_control_ticks;
    g_wd.actual_control_hz = (1000.0f * (float)ticks) / (float)dt_ms;
    g_wd.last_hz_tick_ms = now_ms;
    g_wd.last_hz_control_ticks = g_wd.control_ticks;
  }
}

static uint32_t wd_control_logical_bus_index(uint8_t physical_bus_index) {
  uint32_t bus_index = (uint32_t)physical_bus_index;

#if WD_BENCH_REMAP_CAN2_TO_LOGICAL0
  /*
   * Bench bring-up only:
   * The currently wired four-motor route is on this MCU's physical CAN2.
   * Present it to WDP4 as logical bus 0 so motor IDs 1..4 become joints 0..3.
   * Keep production builds on the direct CAN1->0, CAN2->1 mapping.
   */
  if (bus_index == (WD_CAN_BUS_BASE + 0u)) {
    return WD_CAN_BUS_BASE + 1u;
  }
  if (bus_index == (WD_CAN_BUS_BASE + 1u)) {
    return WD_CAN_BUS_BASE + 0u;
  }
#endif

  return bus_index;
}

#if WD_ALLOW_LIVE_CAN_CONTROL
static uint32_t wd_control_physical_bus_index(uint32_t logical_bus_index) {
#if WD_BENCH_REMAP_CAN2_TO_LOGICAL0
  if (logical_bus_index == (WD_CAN_BUS_BASE + 0u)) {
    return WD_CAN_BUS_BASE + 1u;
  }
  if (logical_bus_index == (WD_CAN_BUS_BASE + 1u)) {
    return WD_CAN_BUS_BASE + 0u;
  }
#endif
  return logical_bus_index;
}
#endif

static uint32_t wd_control_index_from_bus_motor(uint8_t bus_index,
                                                uint32_t motor_id,
                                                uint32_t *index) {
  uint32_t mapped_index;
  uint32_t logical_bus_index;

  if (motor_id < 1u || motor_id > 4u || bus_index >= 4u || index == NULL) {
    return 0u;
  }

  logical_bus_index = wd_control_logical_bus_index(bus_index);
  mapped_index = (logical_bus_index * 4u) + (motor_id - 1u);
  if (mapped_index >= WD_MOTOR_COUNT) {
    return 0u;
  }

  *index = mapped_index;
  return 1u;
}

/*
 * RobStride type-0 device discovery is a two-byte request (01 00).  It is
 * intentionally available in dry-run builds because some motors do not answer
 * READ_PARAM after power-up until this query has been seen.  The query neither
 * enables the motor nor writes a parameter.
 */
static int wd_control_send_device_query(uint8_t local_bus, uint8_t motor_id) {
  CAN_TxHeaderTypeDef header;
  uint8_t data[2] = {0x01u, 0x00u};
  uint32_t mailbox;
  CAN_HandleTypeDef *hcan;

  if (motor_id < 1u || motor_id > WD_MOTORS_PER_CAN_BUS) {
    return 0;
  }
  if (local_bus == 0u) {
    hcan = &hcan1;
  } else if (local_bus == 1u) {
    hcan = &hcan2;
  } else {
    return 0;
  }
  if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0u) {
    return 0;
  }

  header.StdId = 0u;
  header.ExtId = ((uint32_t)WD_ROBSTRIDE_COMM_GET_DEVICE_ID << 24) |
                 ((uint32_t)WD_ROBSTRIDE_HOST_ID << 8) |
                 (uint32_t)motor_id;
  header.IDE = CAN_ID_EXT;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 2u;
  header.TransmitGlobalTime = DISABLE;
  return (HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox) == HAL_OK) ? 1 : 0;
}

#if WD_ALLOW_LIVE_CAN_CONTROL
static uint32_t wd_control_local_bus_motor_from_index(uint32_t index,
                                                      uint8_t *local_bus,
                                                      uint8_t *motor_id) {
  uint32_t logical_bus_index;
  uint32_t physical_bus_index;

  if (index >= WD_MOTOR_COUNT || local_bus == NULL || motor_id == NULL) {
    return 0u;
  }

  logical_bus_index = index / 4u;
  physical_bus_index = wd_control_physical_bus_index(logical_bus_index);
  if ((physical_bus_index - (uint32_t)WD_CAN_BUS_BASE) >= 2u) {
    return 0u;
  }

  *local_bus = (uint8_t)(physical_bus_index - WD_CAN_BUS_BASE);
  *motor_id = (uint8_t)((index % 4u) + 1u);
  return 1u;
}

static CAN_HandleTypeDef *wd_control_can_handle(uint8_t local_bus) {
  if (local_bus == 0u) {
    return &hcan1;
  }
  if (local_bus == 1u) {
    return &hcan2;
  }
  return NULL;
}

static uint32_t wd_control_standard_can_id(uint32_t comm_type,
                                           uint8_t motor_id) {
  return ((comm_type & 0x1Fu) << 24) |
         ((uint32_t)WD_ROBSTRIDE_HOST_ID << 8) |
         ((uint32_t)motor_id & 0xFFu);
}

static int wd_control_send_can_frame(uint8_t local_bus,
                                     uint32_t can_id,
                                     const uint8_t data[8]) {
  CAN_TxHeaderTypeDef header;
  uint32_t mailbox;
  CAN_HandleTypeDef *hcan = wd_control_can_handle(local_bus);

  if (hcan == NULL || data == NULL) {
    return 0;
  }
  if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0u) {
    return 0;
  }

  header.StdId = 0u;
  header.ExtId = can_id;
  header.IDE = CAN_ID_EXT;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8u;
  header.TransmitGlobalTime = DISABLE;
  return (HAL_CAN_AddTxMessage(hcan, &header, (uint8_t *)data, &mailbox) == HAL_OK) ?
      1 : 0;
}

/*
 * Return 1 when queued, 0 when all three mailboxes are temporarily occupied,
 * and -1 for a real HAL/configuration failure.
 */
static int wd_control_send_can_frame_tagged(uint8_t local_bus,
                                            uint32_t can_id,
                                            const uint8_t data[8],
                                            uint8_t motor_index) {
  CAN_TxHeaderTypeDef header;
  uint32_t mailbox;
  uint32_t mailbox_index;
  uint32_t primask;
  HAL_StatusTypeDef status;
  CAN_HandleTypeDef *hcan = wd_control_can_handle(local_bus);

  if (hcan == NULL || data == NULL || motor_index >= WD_MOTOR_COUNT) {
    return -1;
  }
  if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0u) {
    return 0;
  }

  header.StdId = 0u;
  header.ExtId = can_id;
  header.IDE = CAN_ID_EXT;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8u;
  header.TransmitGlobalTime = DISABLE;
  primask = __get_PRIMASK();
  __disable_irq();
  status = HAL_CAN_AddTxMessage(hcan, &header, (uint8_t *)data, &mailbox);
  if (status == HAL_OK &&
      wd_control_can_mailbox_index(mailbox, &mailbox_index) != 0) {
    g_can_tx_mailbox_tag[local_bus][mailbox_index] = motor_index;
  }
  __DMB();
  if (primask == 0u) {
    __enable_irq();
  }
  if (status != HAL_OK) {
    return -1;
  }
  if (wd_control_can_mailbox_index(mailbox, &mailbox_index) == 0) {
    return -1;
  }
  return 1;
}

static int wd_control_send_write_param_u32(uint8_t local_bus,
                                           uint8_t motor_id,
                                           uint16_t param_index,
                                           uint32_t value) {
  uint8_t data[8] = {0};

  wd_write_le_u16(&data[0], param_index);
  wd_write_le_u32(&data[4], value);
  return wd_control_send_can_frame(
      local_bus,
      wd_control_standard_can_id(WD_ROBSTRIDE_COMM_WRITE_PARAM, motor_id),
      data);
}

static int wd_control_send_write_param_f32(uint8_t local_bus,
                                           uint8_t motor_id,
                                           uint16_t param_index,
                                           float value) {
  uint8_t data[8] = {0};

  wd_write_le_u16(&data[0], param_index);
  wd_write_le_f32(&data[4], value);
  return wd_control_send_can_frame(
      local_bus,
      wd_control_standard_can_id(WD_ROBSTRIDE_COMM_WRITE_PARAM, motor_id),
      data);
}

static int wd_control_send_write_param_f32_tagged(uint8_t local_bus,
                                                  uint8_t motor_id,
                                                  uint16_t param_index,
                                                  float value,
                                                  uint8_t motor_index) {
  uint8_t data[8] = {0};

  wd_write_le_u16(&data[0], param_index);
  wd_write_le_f32(&data[4], value);
  return wd_control_send_can_frame_tagged(
      local_bus,
      wd_control_standard_can_id(WD_ROBSTRIDE_COMM_WRITE_PARAM, motor_id),
      data,
      motor_index);
}

static int wd_control_send_motion_command(uint8_t local_bus,
                                          uint8_t motor_id,
                                          float velocity_radps,
                                          float kd,
                                          uint8_t motor_index) {
  uint32_t can_id;
  uint8_t data[8];

  if (wd_robstride_build_motion_command(motor_id,
                                        0.0f,
                                        0.0f,
                                        velocity_radps,
                                        0.0f,
                                        kd,
                                        &can_id,
                                        data) == 0) {
    return 0;
  }
  if (motor_index == WD_CAN_TX_TAG_NONE) {
    return wd_control_send_can_frame(local_bus, can_id, data);
  }
  return wd_control_send_can_frame_tagged(
      local_bus, can_id, data, motor_index);
}

static int wd_control_send_init(uint8_t local_bus, uint8_t motor_id) {
  return wd_control_send_device_query(local_bus, motor_id);
}

static int wd_control_send_enable(uint8_t local_bus, uint8_t motor_id) {
  uint8_t data[8] = {0};

  data[1] = 0xC4u;
  return wd_control_send_can_frame(
      local_bus,
      wd_control_standard_can_id(WD_ROBSTRIDE_COMM_ENABLE_DISABLE, motor_id),
      data);
}

static int wd_control_send_disable(uint8_t local_bus, uint8_t motor_id) {
  uint8_t data[8] = {0};

  return wd_control_send_can_frame(
      local_bus,
      wd_control_standard_can_id(WD_ROBSTRIDE_COMM_ENABLE_DISABLE, motor_id),
      data);
}

static int wd_control_send_start(uint8_t local_bus, uint8_t motor_id) {
  uint8_t data[8] = {0};

  return wd_control_send_can_frame(
      local_bus,
      wd_control_standard_can_id(WD_ROBSTRIDE_COMM_START, motor_id),
      data);
}
#endif

static int wd_control_send_read_param(uint8_t local_bus,
                                      uint8_t motor_id,
                                      uint16_t param_index) {
  CAN_TxHeaderTypeDef header;
  uint8_t data[8] = {0};
  uint32_t mailbox;
  CAN_HandleTypeDef *hcan;

  if (motor_id < 1u || motor_id > 4u) {
    return 0;
  }

  if (local_bus == 0u) {
    hcan = &hcan1;
  } else if (local_bus == 1u) {
    hcan = &hcan2;
  } else {
    return 0;
  }

  if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0u) {
    return 0;
  }

  header.StdId = 0u;
  header.ExtId = ((uint32_t)WD_ROBSTRIDE_COMM_READ_PARAM << 24) |
                 ((uint32_t)WD_ROBSTRIDE_HOST_ID << 8) |
                 (uint32_t)motor_id;
  header.IDE = CAN_ID_EXT;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8u;
  header.TransmitGlobalTime = DISABLE;

  data[0] = (uint8_t)(param_index & 0xFFu);
  data[1] = (uint8_t)((param_index >> 8) & 0xFFu);
  return (HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox) == HAL_OK) ? 1 : 0;
}

static void wd_control_abort_readonly_poll_tx(void) {
  HAL_CAN_AbortTxRequest(&hcan1,
                         CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
  HAL_CAN_AbortTxRequest(&hcan2,
                         CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
  g_wd.readonly_poll_running = 0u;
  g_wd.readonly_discovery_step = 0u;
  g_wd.readonly_discovery_bus = 0xFFu;
}

#if WD_ENABLE_CALIBRATION_READONLY
static void wd_control_service_readonly_poll(uint32_t now_ms) {
  uint32_t motor_id;
  uint32_t poll_step;
  uint32_t poll_motor_count;
  uint16_t param_index;
  uint8_t local_bus;
  uint8_t poll_both;

  if (g_wd.dry_run == 0u ||
      (g_wd.control_flags & WD_CONTROL_FLAG_READONLY_POLL) == 0u ||
      (g_wd.status_flags & (WD_STATUS_COMMAND_TIMEOUT | WD_STATUS_ESTOP)) != 0u) {
    if (g_wd.readonly_poll_running != 0u) {
      wd_control_abort_readonly_poll_tx();
    }
    return;
  }

  g_wd.readonly_poll_running = 1u;
  g_wd.status_flags |= WD_STATUS_READONLY_POLL;
  poll_both =
      ((g_wd.control_flags & WD_CONTROL_FLAG_CALIBRATION_READONLY) != 0u) ?
          1u : 0u;
  local_bus = (poll_both != 0u) ? 2u :
      (((g_wd.control_flags & WD_CONTROL_FLAG_READONLY_POLL_CAN2) != 0u) ?
          1u : 0u);
  poll_motor_count = (poll_both != 0u) ?
      WD_LIVE_LOCAL_MOTOR_COUNT : WD_READONLY_POLL_MOTOR_COUNT;
  if (g_wd.readonly_discovery_bus != local_bus) {
    g_wd.readonly_discovery_bus = local_bus;
    g_wd.readonly_discovery_step = 0u;
    g_wd.readonly_poll_step = 0u;
  }
  if ((now_ms - g_wd.last_readonly_poll_ms) < WD_READONLY_POLL_PERIOD_MS) {
    return;
  }
  g_wd.last_readonly_poll_ms = now_ms;

  if (g_wd.readonly_discovery_step < poll_motor_count) {
    poll_step = g_wd.readonly_discovery_step;
    if (poll_both != 0u) {
      local_bus = (uint8_t)(poll_step / WD_MOTORS_PER_CAN_BUS);
      poll_step %= WD_MOTORS_PER_CAN_BUS;
    }
    motor_id = poll_step + 1u;
    if (wd_control_send_device_query(local_bus, (uint8_t)motor_id) == 0) {
      ++g_wd.can_tx_errors;
      g_wd.status_flags |= WD_STATUS_CAN_TX_ERROR;
      return;
    }
    ++g_wd.readonly_discovery_step;
    return;
  }

  poll_step = g_wd.readonly_poll_step;
  if (poll_both != 0u) {
    local_bus = (uint8_t)(poll_step /
        (WD_MOTORS_PER_CAN_BUS * WD_READONLY_POLL_PARAM_COUNT));
    poll_step %= (WD_MOTORS_PER_CAN_BUS * WD_READONLY_POLL_PARAM_COUNT);
  }
  motor_id = (poll_step / WD_READONLY_POLL_PARAM_COUNT) + 1u;
  if ((poll_step % WD_READONLY_POLL_PARAM_COUNT) == 0u) {
    param_index = WD_ROBSTRIDE_PARAM_MECH_POS;
  } else {
    param_index = WD_ROBSTRIDE_PARAM_MECH_VEL;
  }
  ++g_wd.readonly_poll_step;
  if (g_wd.readonly_poll_step >=
      (poll_motor_count * WD_READONLY_POLL_PARAM_COUNT)) {
    g_wd.readonly_poll_step = 0u;
  }

  if (wd_control_send_read_param(local_bus,
                                 (uint8_t)motor_id,
                                 param_index) == 0) {
    ++g_wd.can_tx_errors;
    g_wd.status_flags |= WD_STATUS_CAN_TX_ERROR;
  }
}
#else
static void wd_control_service_readonly_poll(uint32_t now_ms) {
  uint32_t motor_id;
  uint16_t param_index;
  uint8_t local_bus;

  if (g_wd.dry_run == 0u ||
      (g_wd.control_flags & WD_CONTROL_FLAG_READONLY_POLL) == 0u ||
      (g_wd.status_flags & (WD_STATUS_COMMAND_TIMEOUT | WD_STATUS_ESTOP)) != 0u) {
    if (g_wd.readonly_poll_running != 0u) {
      wd_control_abort_readonly_poll_tx();
    }
    return;
  }

  g_wd.readonly_poll_running = 1u;
  g_wd.status_flags |= WD_STATUS_READONLY_POLL;
  local_bus = ((g_wd.control_flags & WD_CONTROL_FLAG_READONLY_POLL_CAN2) != 0u) ?
      1u : 0u;
  if (g_wd.readonly_discovery_bus != local_bus) {
    g_wd.readonly_discovery_bus = local_bus;
    g_wd.readonly_discovery_step = 0u;
    g_wd.readonly_poll_step = 0u;
  }
  if ((now_ms - g_wd.last_readonly_poll_ms) < WD_READONLY_POLL_PERIOD_MS) {
    return;
  }
  g_wd.last_readonly_poll_ms = now_ms;

  if (g_wd.readonly_discovery_step < WD_READONLY_POLL_MOTOR_COUNT) {
    motor_id = g_wd.readonly_discovery_step + 1u;
    if (wd_control_send_device_query(local_bus, (uint8_t)motor_id) == 0) {
      ++g_wd.can_tx_errors;
      g_wd.status_flags |= WD_STATUS_CAN_TX_ERROR;
      return;
    }
    ++g_wd.readonly_discovery_step;
    return;
  }

  motor_id = (g_wd.readonly_poll_step / WD_READONLY_POLL_PARAM_COUNT) + 1u;
  if ((g_wd.readonly_poll_step % WD_READONLY_POLL_PARAM_COUNT) == 0u) {
    param_index = WD_ROBSTRIDE_PARAM_MECH_POS;
  } else {
    param_index = WD_ROBSTRIDE_PARAM_MECH_VEL;
  }
  ++g_wd.readonly_poll_step;
  if (g_wd.readonly_poll_step >=
      (WD_READONLY_POLL_MOTOR_COUNT * WD_READONLY_POLL_PARAM_COUNT)) {
    g_wd.readonly_poll_step = 0u;
  }

  if (wd_control_send_read_param(local_bus,
                                 (uint8_t)motor_id,
                                 param_index) == 0) {
    ++g_wd.can_tx_errors;
    g_wd.status_flags |= WD_STATUS_CAN_TX_ERROR;
  }
}
#endif

static int wd_control_feedback_healthy(uint32_t index);
static void wd_control_zero_motor_command(uint32_t index);

#if WD_ALLOW_LIVE_CAN_CONTROL
static uint32_t wd_control_live_index_from_slot(uint32_t slot) {
  return WD_LOCAL_LOGICAL_FIRST_INDEX + slot;
}

static void wd_control_reset_fast_qualification(void) {
  uint32_t slot;

  g_wd.fast_feedback_armed = 0u;
  g_wd.live_stop_reason_flags = 0u;
  g_wd.live_stop_motor_mask = 0u;
  g_wd.live_stop_event_count = 0u;
  g_wd.live_stop_trigger_time_ms = 0u;
  g_wd.live_stop_fast_valid_mask = 0u;
  g_wd.live_stop_fault_mask = 0u;
  memset(g_wd.live_stop_fast_age_ms, 0, sizeof(g_wd.live_stop_fast_age_ms));
  memset(g_wd.live_stop_joint_torque_cmd_nm,
         0,
         sizeof(g_wd.live_stop_joint_torque_cmd_nm));
  for (slot = 0u; slot < WD_LIVE_LOCAL_MOTOR_COUNT; ++slot) {
    const uint32_t index = wd_control_live_index_from_slot(slot);
    WdFastFeedbackState *fast = &g_wd.fast_feedback[index];
    /*
     * Require a fresh rate window and fresh READ_PARAM comparisons on every
     * live entry. Keep only the observed excitation span for this MCU boot so
     * the first zero-command bench qualification can be followed by another
     * zero-command/live run without manually rotating all motors again.
     */
    fast->rate_window_started = 0u;
    fast->rate_valid = 0u;
    fast->measured_rate_hz = 0.0f;
    fast->last_operation_ms = 0u;
    fast->max_operation_gap_ms = 0u;
    fast->position_reference_ok = 0u;
    fast->velocity_reference_ok = 0u;
    fast->position_match_count = 0u;
    fast->velocity_match_count = 0u;
    fast->position_mismatch_count = 0u;
    fast->velocity_mismatch_count = 0u;
    g_wd.overspeed_consecutive_count[index] = 0u;
    g_wd.overspeed_last_operation_rx_count[index] =
        fast->operation_rx_count;
    g_wd.fast_feedback_generation[index] = 0u;
    g_wd.last_command_feedback_generation[index] = 0u;
    g_wd.last_command_compute_ms[index] = 0u;
    wd_control_zero_motor_command(index);
  }
}

static uint32_t wd_control_live_index_allowed(uint32_t index) {
  const WdJointSafetyConfig *joint = wd_safety_joint_config(index);

  if (
#if WD_CAN_BUS_BASE != 0
      index < WD_LOCAL_LOGICAL_FIRST_INDEX ||
#endif
      index >= WD_LOCAL_LOGICAL_END_INDEX ||
      index >= WD_MOTOR_COUNT ||
      joint == NULL) {
    return 0u;
  }
  if ((index % WD_MOTORS_PER_CAN_BUS) < 3u) {
    return (joint->mode == WD_ACTUATOR_POSITION_PD) ? 1u : 0u;
  }
  return (joint->mode == WD_ACTUATOR_VELOCITY) ? 1u : 0u;
}

/*
 * Wake/discover all eight local motors before requiring READ_PARAM feedback.
 * If any motor remains unhealthy, repeat the read-only discovery pass after a
 * bounded interval.  This avoids the former cold-start deadlock where INIT was
 * sent only after the feedback that INIT was needed to obtain.
 */
static uint32_t wd_control_service_live_discovery(uint32_t now_ms) {
  uint32_t index;
  uint8_t local_bus;
  uint8_t motor_id;

  if (g_wd.live_discovery_step >= WD_LIVE_LOCAL_MOTOR_COUNT) {
    if ((now_ms - g_wd.live_discovery_complete_ms) <
        WD_DEVICE_DISCOVERY_RETRY_MS) {
      return 1u;
    }
    g_wd.live_discovery_step = 0u;
  }
  if ((now_ms - g_wd.last_live_discovery_ms) <
      WD_READONLY_POLL_PERIOD_MS) {
    return 0u;
  }

  index = wd_control_live_index_from_slot(g_wd.live_discovery_step);
  if (wd_control_live_index_allowed(index) == 0u ||
      wd_control_local_bus_motor_from_index(index, &local_bus, &motor_id) == 0u ||
      wd_control_send_device_query(local_bus, motor_id) == 0) {
    /* A full bxCAN mailbox is a normal short deferral while both buses are
     * being discovered. Do not turn one missed setup query into a live-control
     * fault; this step is not advanced and is retried after the bounded poll
     * interval. A persistent failure is still caught by the PC setup timeout. */
    return 0u;
  }

  g_wd.last_live_discovery_ms = now_ms;
  ++g_wd.live_discovery_step;
  if (g_wd.live_discovery_step >= WD_LIVE_LOCAL_MOTOR_COUNT) {
    g_wd.live_discovery_complete_ms = now_ms;
    return 1u;
  }
  return 0u;
}

static void wd_control_service_live_precheck_poll(uint32_t now_ms) {
  uint32_t poll_slot;
  uint32_t index;
  uint16_t param_index;
  uint8_t local_bus;
  uint8_t motor_id;

  if ((now_ms - g_wd.last_live_precheck_poll_ms) < WD_READONLY_POLL_PERIOD_MS) {
    return;
  }
  g_wd.last_live_precheck_poll_ms = now_ms;

  poll_slot = g_wd.live_precheck_poll_step %
      (WD_LIVE_LOCAL_MOTOR_COUNT * WD_READONLY_POLL_PARAM_COUNT);
  index = wd_control_live_index_from_slot(poll_slot / WD_READONLY_POLL_PARAM_COUNT);
  param_index = ((poll_slot % WD_READONLY_POLL_PARAM_COUNT) == 0u) ?
      WD_ROBSTRIDE_PARAM_MECH_POS : WD_ROBSTRIDE_PARAM_MECH_VEL;
  g_wd.live_precheck_poll_step = poll_slot + 1u;

  if (wd_control_live_index_allowed(index) == 0u ||
      wd_control_local_bus_motor_from_index(index, &local_bus, &motor_id) == 0u ||
      wd_control_send_read_param(local_bus, motor_id, param_index) == 0) {
    /* READ_PARAM is an asynchronous cross-check, not the 500 Hz control
     * stream. Skip one poll when all mailboxes are busy. Fresh operation
     * status, rate qualification and the independent stale watchdog continue
     * to protect the active command path. */
    return;
  }
}

/*
 * VBUS is not part of the 500 Hz operation-status frame. Read it through the
 * drive's asynchronous parameter channel without putting it on the realtime
 * command path: one request is attempted every 5 ms until all eight local
 * motors have been sampled, then the next sweep waits three seconds. Busy CAN
 * mailboxes simply defer the request; motor commands retain priority.
 */
static void wd_control_service_motor_telemetry(uint32_t now_ms) {
  const uint32_t local_mask =
      ((1u << WD_LIVE_LOCAL_MOTOR_COUNT) - 1u) <<
      WD_LOCAL_LOGICAL_FIRST_INDEX;
  uint32_t index;
  uint8_t local_bus;
  uint8_t motor_id;

  if ((g_wd.status_flags & (WD_STATUS_LIVE_CONTROL_ACTIVE |
                            WD_STATUS_LIVE_ENABLE_READY)) !=
          (WD_STATUS_LIVE_CONTROL_ACTIVE | WD_STATUS_LIVE_ENABLE_READY) ||
      (g_wd.status_flags & (WD_STATUS_COMMAND_TIMEOUT | WD_STATUS_ESTOP |
                            WD_STATUS_LIVE_SAFETY_STOP)) != 0u) {
    g_wd.motor_telemetry_poll_cursor = 0u;
    g_wd.last_motor_telemetry_poll_ms = 0u;
    return;
  }

  if (g_wd.motor_telemetry_poll_cursor == 0u &&
      g_wd.last_motor_telemetry_poll_ms != 0u &&
      (now_ms - g_wd.last_motor_telemetry_poll_ms) <
          WD_MOTOR_TELEMETRY_POLL_PERIOD_MS) {
    return;
  }
  if ((now_ms - g_wd.last_motor_telemetry_attempt_ms) <
      WD_MOTOR_TELEMETRY_POLL_STEP_MS) {
    return;
  }
  g_wd.last_motor_telemetry_attempt_ms = now_ms;

  index = WD_LOCAL_LOGICAL_FIRST_INDEX +
          g_wd.motor_telemetry_poll_cursor;
  if (wd_control_local_bus_motor_from_index(index, &local_bus, &motor_id) == 0u ||
      wd_control_send_read_param(local_bus,
                                 motor_id,
                                 WD_ROBSTRIDE_PARAM_VBUS) == 0) {
    return;
  }

  if (g_wd.motor_telemetry_poll_cursor == 0u) {
    /* Do not present an old voltage indefinitely when a motor misses the new
     * three-second sweep. Each successful reply revalidates its own bit. */
    g_wd.supply_voltage_valid_mask &= ~local_mask;
  }
  ++g_wd.motor_telemetry_poll_cursor;
  if (g_wd.motor_telemetry_poll_cursor >= WD_LIVE_LOCAL_MOTOR_COUNT) {
    g_wd.motor_telemetry_poll_cursor = 0u;
    g_wd.last_motor_telemetry_poll_ms = now_ms;
  }
}

static uint32_t wd_control_live_all_feedback_healthy(void) {
  uint32_t slot;

  for (slot = 0u; slot < WD_LIVE_LOCAL_MOTOR_COUNT; ++slot) {
    const uint32_t index = wd_control_live_index_from_slot(slot);
    if (wd_control_live_index_allowed(index) == 0u ||
        wd_control_feedback_healthy(index) == 0) {
      return 0u;
    }
  }
  return 1u;
}

static uint32_t wd_control_live_all_fast_feedback_healthy(uint32_t now_ms) {
  uint32_t slot;

  for (slot = 0u; slot < WD_LIVE_LOCAL_MOTOR_COUNT; ++slot) {
    const uint32_t index = wd_control_live_index_from_slot(slot);
    if (wd_fast_feedback_ready(&g_wd.fast_feedback[index], now_ms) == 0) {
      return 0u;
    }
  }
  return 1u;
}

static uint32_t wd_control_live_fast_feedback_stop_reason(
    uint32_t now_ms, uint32_t *motor_mask) {
  uint32_t slot;
  uint32_t mask = 0u;
  uint32_t reason = 0u;

  if (motor_mask == NULL) {
    return WD_LIVE_STOP_REASON_FAST_FEEDBACK_LOST;
  }

  for (slot = 0u; slot < WD_LIVE_LOCAL_MOTOR_COUNT; ++slot) {
    const uint32_t index = wd_control_live_index_from_slot(slot);
    const WdFastFeedbackState *fast = &g_wd.fast_feedback[index];
    uint32_t local_reason = 0u;
    /*
     * Strict 5 ms freshness is handled in the command compute path: that
     * channel is immediately commanded to zero and no held sample is reused.
     * Escalate to a latched all-motor stop only when qualification itself is
     * lost or the feedback hole persists to the independent 300 ms stop
     * bound.
     */
    if (fast->operation_rx_count == 0u ||
        (now_ms - fast->last_operation_ms) >= WD_FAST_FEEDBACK_STOP_MS) {
      local_reason |= WD_LIVE_STOP_REASON_FAST_STALE;
    }
    if (fast->rate_valid == 0u) {
      local_reason |= WD_LIVE_STOP_REASON_FAST_RATE_LOST;
    }
    /*
     * Initial qualification must establish these latches before arming.
     * wd_fast_feedback.c deliberately never clears them from an asynchronous
     * READ_PARAM mismatch after arming; dynamic cross-check errors remain
     * telemetry only and cannot enter this stop path.
     */
    if (fast->position_match_count < WD_FAST_REFERENCE_MATCH_COUNT ||
        fast->velocity_match_count < WD_FAST_REFERENCE_MATCH_COUNT) {
      local_reason |= WD_LIVE_STOP_REASON_FAST_REFERENCE_LOST;
    }
    if (local_reason != 0u) {
      mask |= (1u << index);
      reason |= local_reason;
    }
  }
  *motor_mask = mask;
  return (reason != 0u) ?
      (reason | WD_LIVE_STOP_REASON_FAST_FEEDBACK_LOST) : 0u;
}

static uint32_t wd_control_live_overspeed_mask(uint32_t now_ms) {
  uint32_t slot;
  uint32_t mask = 0u;

  for (slot = 0u; slot < WD_LIVE_LOCAL_MOTOR_COUNT; ++slot) {
    const uint32_t index = wd_control_live_index_from_slot(slot);
    const WdFastFeedbackState *fast = &g_wd.fast_feedback[index];
    float measured_dq;

    /* Only act on a fresh 500 Hz operation-status sample. This makes the
     * feedback-speed trip independent of the slower asynchronous READ_PARAM
     * diagnostics and prevents a stale velocity value from causing a trip. */
    if (fast->operation_rx_count == 0u ||
        (now_ms - fast->last_operation_ms) > WD_FAST_FEEDBACK_STALE_MS) {
      g_wd.overspeed_consecutive_count[index] = 0u;
      g_wd.overspeed_last_operation_rx_count[index] =
          fast->operation_rx_count;
      continue;
    }
    /* The control service is 1 kHz but operation status is 500 Hz. Count each
     * decoded velocity generation once so one sample cannot be counted twice. */
    if (g_wd.overspeed_last_operation_rx_count[index] ==
        fast->operation_rx_count) {
      continue;
    }
    g_wd.overspeed_last_operation_rx_count[index] =
        fast->operation_rx_count;
    measured_dq = wd_motor_sign(index) * fast->operation_velocity_radps;
    if (wd_safety_update_overspeed_trip(
            index,
            measured_dq,
            &g_wd.runtime_config,
            &g_wd.overspeed_consecutive_count[index]) != 0) {
      mask |= (1u << index);
    }
  }
  return mask;
}

static uint32_t wd_control_live_command_fault_mask(void) {
  uint32_t slot;
  uint32_t mask = 0u;

  for (slot = 0u; slot < WD_LIVE_LOCAL_MOTOR_COUNT; ++slot) {
    const uint32_t index = wd_control_live_index_from_slot(slot);
    if (g_wd.live_enabled[index] == 0u ||
        g_wd.feedback[index].fault_bits != 0u) {
      mask |= (1u << index);
    }
  }
  return mask;
}

static void wd_control_latch_live_stop(uint32_t reason,
                                       uint32_t motor_mask,
                                       uint32_t now_ms) {
  uint32_t slot;
  const uint32_t new_reason = reason & ~g_wd.live_stop_reason_flags;
  const uint32_t new_motors = motor_mask & ~g_wd.live_stop_motor_mask;

  if (g_wd.live_stop_reason_flags == 0u) {
    g_wd.live_stop_trigger_time_ms = now_ms;
    for (slot = 0u; slot < WD_LIVE_LOCAL_MOTOR_COUNT; ++slot) {
      const uint32_t index = wd_control_live_index_from_slot(slot);
      uint32_t fast_age_ms = 0xFFu;
      if (wd_fast_feedback_ready(&g_wd.fast_feedback[index], now_ms) != 0) {
        g_wd.live_stop_fast_valid_mask |= (1u << index);
      }
      if (g_wd.fast_feedback[index].operation_rx_count != 0u) {
        fast_age_ms = now_ms - g_wd.fast_feedback[index].last_operation_ms;
        if (fast_age_ms > 0xFFu) {
          fast_age_ms = 0xFFu;
        }
      }
      g_wd.live_stop_fast_age_ms[index] = (uint8_t)fast_age_ms;
      g_wd.live_stop_joint_torque_cmd_nm[index] =
          g_wd.joint_torque_cmd_nm[index];
      if (g_wd.feedback[index].fault_bits != 0u) {
        g_wd.live_stop_fault_mask |= (1u << index);
      }
    }
  }
  g_wd.live_stop_reason_flags |= reason;
  g_wd.live_stop_motor_mask |= motor_mask;
  if (new_reason != 0u || new_motors != 0u) {
    ++g_wd.live_stop_event_count;
  }
}

static uint32_t wd_control_live_all_ready(void) {
  uint32_t slot;

  for (slot = 0u; slot < WD_LIVE_LOCAL_MOTOR_COUNT; ++slot) {
    const uint32_t index = wd_control_live_index_from_slot(slot);
    if (g_wd.live_enable_stage[index] != WD_LIVE_STAGE_READY ||
        g_wd.live_enabled[index] == 0u) {
      return 0u;
    }
  }
  return 1u;
}

static uint32_t wd_control_live_any_started(void) {
  uint32_t slot;

  for (slot = 0u; slot < WD_LIVE_LOCAL_MOTOR_COUNT; ++slot) {
    const uint32_t index = wd_control_live_index_from_slot(slot);
    if (g_wd.live_enable_stage[index] != WD_LIVE_STAGE_IDLE ||
        g_wd.live_enabled[index] != 0u ||
        g_wd.live_stop_stage[index] != WD_LIVE_STOP_NONE) {
      return 1u;
    }
  }
  return 0u;
}

static uint32_t wd_control_live_run_mode(uint32_t index) {
  const WdJointSafetyConfig *joint = wd_safety_joint_config(index);

  if (joint != NULL && joint->mode == WD_ACTUATOR_VELOCITY) {
    return WD_LIVE_RUN_MODE_MOTION;
  }
  return WD_LIVE_RUN_MODE_CURRENT;
}

static int wd_control_send_zero_live_command(uint32_t index) {
  uint8_t local_bus;
  uint8_t motor_id;
  const WdJointSafetyConfig *joint = wd_safety_joint_config(index);

  if (joint == NULL ||
      wd_control_local_bus_motor_from_index(index, &local_bus, &motor_id) == 0u) {
    return 0;
  }
  if (joint->mode == WD_ACTUATOR_VELOCITY) {
    /* Kd=0 is a true zero-torque command in RS01 motion-control mode. This is
     * used before disable and must not be confused with a normal flexible
     * zero-speed command, which carries Kd=0.4 from StandUp/RL. */
    return wd_control_send_motion_command(local_bus,
                                          motor_id,
                                          0.0f,
                                          0.0f,
                                          WD_CAN_TX_TAG_NONE);
  }
  return wd_control_send_write_param_f32(local_bus,
                                         motor_id,
                                         WD_ROBSTRIDE_PARAM_IQ_REF,
                                         0.0f);
}

static int wd_control_send_live_command(uint32_t index) {
  uint8_t local_bus;
  uint8_t motor_id;
  const WdJointSafetyConfig *joint = wd_safety_joint_config(index);
  const WdJointCommand *command = &g_wd.setpoint.joint[index];

  if (joint == NULL ||
      wd_control_local_bus_motor_from_index(index, &local_bus, &motor_id) == 0u) {
    return 0;
  }
  if (joint->mode == WD_ACTUATOR_VELOCITY) {
    const float velocity_radps =
        wd_safety_limit_velocity_command(index,
                                         g_wd.motor_velocity_cmd_radps[index],
                                         &g_wd.runtime_config,
                                         &g_wd.command_status_flags);
    return wd_control_send_motion_command(local_bus,
                                          motor_id,
                                          velocity_radps,
                                          command->kd,
                                          (uint8_t)index);
  }
  return wd_control_send_write_param_f32_tagged(
      local_bus,
      motor_id,
      WD_ROBSTRIDE_PARAM_IQ_REF,
      wd_rs06_torque_nm_to_iq_peak_a(g_wd.motor_torque_cmd_nm[index]),
      (uint8_t)index);
}

static int wd_control_send_live_enable_stage(uint32_t index) {
  uint8_t local_bus;
  uint8_t motor_id;
  uint8_t stage;

  if (wd_control_live_index_allowed(index) == 0u ||
      wd_control_local_bus_motor_from_index(index, &local_bus, &motor_id) == 0u) {
    return 0;
  }

  stage = g_wd.live_enable_stage[index];
  if (stage == WD_LIVE_STAGE_IDLE) {
    stage = WD_LIVE_STAGE_INIT;
    g_wd.live_enable_stage[index] = stage;
  }

  switch (stage) {
    case WD_LIVE_STAGE_INIT:
      return wd_control_send_init(local_bus, motor_id);
    case WD_LIVE_STAGE_STOP_BEFORE_MODE:
      /* The RS01 manual forbids switching run_mode while the actuator is
       * running. Make the precondition explicit instead of relying on the
       * previous PC/MCU session to have stopped cleanly. */
      return wd_control_send_disable(local_bus, motor_id);
    case WD_LIVE_STAGE_RUN_MODE:
      return wd_control_send_write_param_u32(
          local_bus,
          motor_id,
          WD_ROBSTRIDE_PARAM_RUN_MODE,
          wd_control_live_run_mode(index));
    case WD_LIVE_STAGE_COMMAND_LIMIT:
      if (wd_safety_joint_config(index)->mode == WD_ACTUATOR_VELOCITY) {
        return wd_control_send_write_param_f32(
            local_bus,
            motor_id,
            WD_ROBSTRIDE_PARAM_TORQUE_LIMIT,
            wd_safety_effective_torque_limit(index, &g_wd.runtime_config));
      }
      return wd_control_send_write_param_f32(
          local_bus,
          motor_id,
          WD_ROBSTRIDE_PARAM_I_LIMIT,
          wd_current_limit_peak_a(index));
    case WD_LIVE_STAGE_ENABLE:
      return wd_control_send_enable(local_bus, motor_id);
    case WD_LIVE_STAGE_START:
      return wd_control_send_start(local_bus, motor_id);
    case WD_LIVE_STAGE_ZERO_COMMAND:
      return wd_control_send_zero_live_command(index);
    default:
      return 1;
  }
}

static void wd_control_request_live_stop(void) {
  uint32_t slot;

  for (slot = 0u; slot < WD_LIVE_LOCAL_MOTOR_COUNT; ++slot) {
    const uint32_t index = wd_control_live_index_from_slot(slot);
    if ((g_wd.live_enable_stage[index] != WD_LIVE_STAGE_IDLE ||
         g_wd.live_enabled[index] != 0u) &&
        g_wd.live_stop_stage[index] == WD_LIVE_STOP_NONE) {
      g_wd.live_stop_stage[index] = WD_LIVE_STOP_ZERO_COMMAND;
    }
  }
  g_wd.live_command_scheduler.initialized = 0u;
}

static void wd_control_service_live_stop(uint32_t now_ms) {
  uint32_t attempt;

  if ((now_ms - g_wd.last_live_tx_ms) < WD_LIVE_TX_PERIOD_MS) {
    return;
  }

  for (attempt = 0u; attempt < WD_LIVE_LOCAL_MOTOR_COUNT; ++attempt) {
    uint32_t index = wd_control_live_index_from_slot(
        g_wd.live_stop_cursor % WD_LIVE_LOCAL_MOTOR_COUNT);
    uint8_t local_bus;
    uint8_t motor_id;
    int sent = 1;

    ++g_wd.live_stop_cursor;
    if (g_wd.live_stop_stage[index] == WD_LIVE_STOP_NONE) {
      continue;
    }

    if (g_wd.live_stop_stage[index] == WD_LIVE_STOP_ZERO_COMMAND) {
      sent = wd_control_send_zero_live_command(index);
      if (sent != 0) {
        g_wd.live_stop_stage[index] = WD_LIVE_STOP_DISABLE;
      }
    } else if (wd_control_local_bus_motor_from_index(index,
                                                      &local_bus,
                                                      &motor_id) != 0u) {
      sent = wd_control_send_disable(local_bus, motor_id);
      if (sent != 0) {
        g_wd.live_stop_stage[index] = WD_LIVE_STOP_NONE;
        g_wd.live_enable_stage[index] = WD_LIVE_STAGE_IDLE;
        g_wd.live_enabled[index] = 0u;
        g_wd.feedback[index].enabled = 0u;
      }
    } else {
      sent = 0;
    }

    g_wd.last_live_tx_ms = now_ms;
    if (sent == 0) {
      ++g_wd.can_tx_errors;
      g_wd.status_flags |= WD_STATUS_CAN_TX_ERROR;
    }
    return;
  }
}

static void wd_control_service_live_enable(uint32_t now_ms) {
  uint32_t attempt;

  if ((now_ms - g_wd.last_live_tx_ms) < WD_LIVE_ENABLE_STEP_MS) {
    return;
  }

  for (attempt = 0u; attempt < WD_LIVE_LOCAL_MOTOR_COUNT; ++attempt) {
    uint32_t index = wd_control_live_index_from_slot(
        g_wd.live_enable_cursor % WD_LIVE_LOCAL_MOTOR_COUNT);
    int sent;

    ++g_wd.live_enable_cursor;
    if (g_wd.live_enable_stage[index] == WD_LIVE_STAGE_READY) {
      continue;
    }
    if (g_wd.live_stop_stage[index] != WD_LIVE_STOP_NONE) {
      return;
    }
    if (wd_control_live_index_allowed(index) == 0u ||
        wd_control_feedback_healthy(index) == 0) {
      g_wd.status_flags |= WD_STATUS_LIVE_CONTROL_BLOCKED;
      return;
    }

    sent = wd_control_send_live_enable_stage(index);
    g_wd.last_live_tx_ms = now_ms;
    if (sent == 0) {
      /* Do not advance the stage. One occupied mailbox or failed setup frame
       * is retried 20 ms later; only a persistent inability to complete the
       * verified enable sequence reaches the host setup timeout. */
      return;
    }

    if (g_wd.live_enable_stage[index] < WD_LIVE_STAGE_READY) {
      ++g_wd.live_enable_stage[index];
      if (g_wd.live_enable_stage[index] == WD_LIVE_STAGE_READY) {
        g_wd.live_enabled[index] = 1u;
        g_wd.feedback[index].enabled = 1u;
      }
    }
    return;
  }
}

static void wd_control_service_live_commands(uint32_t now_ms) {
  uint8_t local_bus;

  if (g_wd.live_command_scheduler.initialized == 0u) {
    wd_can_scheduler_init(&g_wd.live_command_scheduler, now_ms);
  }

  for (local_bus = 0u; local_bus < WD_LOCAL_CAN_BUS_COUNT; ++local_bus) {
    uint8_t slots[WD_CAN_SCHEDULER_MOTORS_PER_BUS];
    uint32_t due = wd_can_scheduler_collect_due(
        &g_wd.live_command_scheduler, now_ms, local_bus, slots);
    uint32_t attempt;

    for (attempt = 0u; attempt < due; ++attempt) {
      const uint32_t local_slot = slots[attempt];
      const uint32_t index = wd_control_live_index_from_slot(local_slot);
      int sent;

      /*
       * Do not latch all motors from a transient slow READ_PARAM freshness
       * miss. Runtime PD already consumes only new <=5 ms operation-status
       * generations and zeroes an individual channel immediately when that
       * fast stream is stale. The independent fast-feedback watchdog then
       * escalates a continuously missing stream after 300 ms. A drive that is
       * actually disabled or reports a hardware fault remains an immediate
       * stop condition here.
       */
      if (g_wd.live_enabled[index] == 0u ||
          g_wd.feedback[index].fault_bits != 0u) {
        const uint32_t reason = (g_wd.live_enabled[index] == 0u) ?
            WD_LIVE_STOP_REASON_COMMAND_MOTOR_NOT_ENABLED :
            WD_LIVE_STOP_REASON_COMMAND_FEEDBACK_UNHEALTHY;
        uint32_t unhealthy_mask = wd_control_live_command_fault_mask();
        if (unhealthy_mask == 0u) {
          unhealthy_mask = (1u << index);
        }
        wd_control_latch_live_stop(reason, unhealthy_mask, now_ms);
        g_wd.status_flags |= WD_STATUS_LIVE_SAFETY_STOP |
                             WD_STATUS_LIVE_CONTROL_BLOCKED;
        wd_control_request_live_stop();
        return;
      }

      sent = wd_control_send_live_command(index);
      if (sent > 0) {
        ++g_wd.live_command_tx_queued[index];
        wd_can_scheduler_mark_queued(&g_wd.live_command_scheduler,
                                     (uint8_t)local_slot,
                                     now_ms);
      } else if (sent == 0) {
        ++g_wd.live_command_tx_deferred[index];
        if (wd_can_scheduler_failed_queue_is_deadline_miss(
                &g_wd.live_command_scheduler,
                (uint8_t)local_slot,
                now_ms) != 0) {
          g_wd.status_flags |= WD_STATUS_CAN_TX_DEADLINE_MISS;
        }
        /* No mailbox remains on this CAN controller in the current tick. */
        break;
      } else {
        ++g_wd.can_tx_errors;
        g_wd.status_flags |= WD_STATUS_CAN_TX_ERROR;
        break;
      }
    }
  }
}
#endif

static void wd_control_service_live_can(uint32_t now_ms) {
  uint32_t live_requested =
      (g_wd.dry_run == 0u &&
       (g_wd.control_flags & WD_CONTROL_FLAG_LIVE_CONTROL) != 0u &&
       (g_wd.control_flags & WD_CONTROL_FLAG_ENABLE_REQUEST) != 0u) ? 1u : 0u;

  if ((g_wd.control_flags & WD_CONTROL_FLAG_LIVE_CONTROL) != 0u) {
    g_wd.status_flags |= WD_STATUS_LIVE_CONTROL_REQUESTED;
  }

#if WD_ALLOW_LIVE_CAN_CONTROL
  uint32_t blocked =
      g_wd.status_flags & (WD_STATUS_COMMAND_TIMEOUT | WD_STATUS_ESTOP);

  if (blocked != 0u || live_requested == 0u) {
    g_wd.fast_feedback_armed = 0u;
    if (wd_control_live_any_started() != 0u) {
      g_wd.status_flags |= WD_STATUS_LIVE_SAFETY_STOP;
      wd_control_request_live_stop();
      wd_control_service_live_stop(now_ms);
    }
    if (wd_control_live_any_started() == 0u) {
      g_wd.fast_qualification_started = 0u;
      g_wd.live_discovery_step = 0u;
      g_wd.live_discovery_complete_ms = 0u;
    }
    g_wd.live_command_scheduler.initialized = 0u;
    return;
  }

  if (g_wd.fast_qualification_started == 0u) {
    wd_control_reset_fast_qualification();
    g_wd.fast_qualification_started = 1u;
  }

  /* A latched stop must be completed locally even if the PC remains alive and
   * keeps requesting live control. The old behavior waited for the next USB
   * feedback/FATAL round trip before zero/disable, leaving the previous motor
   * command active for an avoidable feedback period. */
  if (g_wd.live_stop_reason_flags != 0u) {
    g_wd.status_flags |= WD_STATUS_LIVE_SAFETY_STOP |
                         WD_STATUS_LIVE_CONTROL_BLOCKED;
    wd_control_request_live_stop();
    wd_control_service_live_stop(now_ms);
    return;
  }

  if (wd_control_live_any_started() == 0u &&
      wd_control_live_all_feedback_healthy() == 0u) {
    g_wd.status_flags |= WD_STATUS_LIVE_ENABLE_IN_PROGRESS;
    if (wd_control_service_live_discovery(now_ms) != 0u) {
      wd_control_service_live_precheck_poll(now_ms);
    }
    return;
  }

  if (wd_control_live_all_ready() == 0u) {
    g_wd.status_flags |= WD_STATUS_LIVE_ENABLE_IN_PROGRESS;
    /*
     * Keep READ_PARAM feedback fresh while the multi-motor enable sequence is
     * progressing. Enabling eight motors one stage at a time can otherwise
     * exceed the stale-feedback window before the last motor becomes ready.
     */
    wd_control_service_live_precheck_poll(now_ms);
    wd_control_service_live_enable(now_ms);
    return;
  }

  g_wd.status_flags |= WD_STATUS_LIVE_CONTROL_ACTIVE |
                       WD_STATUS_LIVE_ENABLE_READY;
  wd_control_service_live_precheck_poll(now_ms);

  {
    const uint32_t overspeed_mask = wd_control_live_overspeed_mask(now_ms);
    if (overspeed_mask != 0u) {
      wd_control_latch_live_stop(WD_LIVE_STOP_REASON_OVERSPEED,
                                 overspeed_mask,
                                 now_ms);
      g_wd.status_flags |= WD_STATUS_LIVE_SAFETY_STOP |
                           WD_STATUS_LIVE_CONTROL_BLOCKED;
      wd_control_request_live_stop();
      /* Start sending zero commands in this same 1 kHz service tick. */
      wd_control_service_live_stop(now_ms);
      return;
    }
  }

  if (wd_control_live_all_fast_feedback_healthy(now_ms) != 0u) {
    if (g_wd.fast_feedback_armed == 0u) {
      uint32_t slot;
      /* Drop enable/qualification gaps; subsequent maxima are runtime-only. */
      for (slot = 0u; slot < WD_LIVE_LOCAL_MOTOR_COUNT; ++slot) {
        const uint32_t index = wd_control_live_index_from_slot(slot);
        g_wd.fast_feedback[index].max_operation_gap_ms = 0u;
      }
    }
    g_wd.fast_feedback_armed = 1u;
    g_wd.status_flags |= WD_STATUS_FAST_FEEDBACK_READY;
  } else {
    g_wd.status_flags |= WD_STATUS_FAST_FEEDBACK_UNQUALIFIED;
    if (g_wd.fast_feedback_armed != 0u) {
      uint32_t stop_mask = 0u;
      const uint32_t stop_reason =
          wd_control_live_fast_feedback_stop_reason(now_ms, &stop_mask);
      if (stop_reason != 0u && stop_mask != 0u) {
        wd_control_latch_live_stop(stop_reason, stop_mask, now_ms);
        g_wd.status_flags |= WD_STATUS_LIVE_SAFETY_STOP |
                             WD_STATUS_LIVE_CONTROL_BLOCKED;
        wd_control_request_live_stop();
        return;
      }
    }
  }
  wd_control_service_live_commands(now_ms);
#else
  (void)now_ms;
  if (live_requested != 0u ||
      (g_wd.control_flags & WD_CONTROL_FLAG_LIVE_CONTROL) != 0u) {
    g_wd.status_flags |= WD_STATUS_LIVE_CONTROL_BLOCKED;
  }
#endif
}

static void wd_control_apply_read_param_feedback(const WdCanRxFrame *frame,
                                                 uint32_t now_ms) {
  uint32_t motor_id;
  uint32_t index;
  uint16_t param_index;
  float value;
  float sign;
#if WD_ENABLE_CALIBRATION_READONLY
  uint8_t calibration_readonly;
#endif
  WdJointFeedback *feedback;
  const WdJointSafetyConfig *joint;

  if (frame == NULL || frame->dlc < 8u) {
    ++g_wd.can_rx_bad_frames;
    return;
  }

  motor_id = (frame->can_id >> 8) & 0xFFu;
  if (wd_control_index_from_bus_motor(frame->bus_index, motor_id, &index) == 0u) {
    ++g_wd.can_rx_bad_frames;
    return;
  }

  param_index = wd_le_u16(&frame->data[0]);
  if (param_index != WD_ROBSTRIDE_PARAM_MECH_POS &&
      param_index != WD_ROBSTRIDE_PARAM_MECH_VEL &&
      param_index != WD_ROBSTRIDE_PARAM_VBUS) {
    return;
  }

  value = wd_le_f32(&frame->data[4]);
  if (wd_safety_is_finite(value) == 0) {
    ++g_wd.can_rx_bad_frames;
    return;
  }

  if (param_index == WD_ROBSTRIDE_PARAM_VBUS) {
    g_wd.supply_voltage_v[index] = value;
    g_wd.supply_voltage_valid_mask |= (1u << index);
    return;
  }

  sign = wd_motor_sign(index);
#if WD_ENABLE_CALIBRATION_READONLY
  calibration_readonly =
      (g_wd.dry_run != 0u &&
       (g_wd.control_flags & (WD_CONTROL_FLAG_READONLY_POLL |
                              WD_CONTROL_FLAG_CALIBRATION_READONLY)) ==
           (WD_CONTROL_FLAG_READONLY_POLL |
            WD_CONTROL_FLAG_CALIBRATION_READONLY)) ? 1u : 0u;
#endif
  feedback = &g_wd.feedback[index];
  joint = wd_safety_joint_config(index);

  if (param_index == WD_ROBSTRIDE_PARAM_MECH_POS) {
    if (g_wd.mech_position_valid[index] == 0u) {
      g_wd.reference_motor_position[index] = value;
      g_wd.continuous_motor_position[index] = value;
      g_wd.mech_position_valid[index] = 1u;
      g_wd.last_operation_position_raw[index] = 0.0f;
      g_wd.operation_position_valid[index] = 0u;
    }
    g_wd.aligned_raw_position[index] = wd_motor_align_raw(index, value);
    wd_fast_feedback_check_position(&g_wd.fast_feedback[index], now_ms, value);
#if WD_ENABLE_CALIBRATION_READONLY
    if (calibration_readonly != 0u) {
      feedback->q = value;
    } else
#endif
    if (wd_fast_feedback_ready(&g_wd.fast_feedback[index], now_ms) == 0) {
      if (joint != NULL && joint->mode == WD_ACTUATOR_POSITION_PD) {
        feedback->q = wd_motor_raw_to_joint(index, value);
      } else {
        feedback->q = value;
      }
    }
    g_wd.last_position_feedback_ms[index] = now_ms;
  } else {
    g_wd.mech_velocity_valid[index] = 1u;
    wd_fast_feedback_check_velocity(&g_wd.fast_feedback[index], now_ms, value);
#if WD_ENABLE_CALIBRATION_READONLY
    if (calibration_readonly != 0u) {
      feedback->dq = value;
    } else
#endif
    if (wd_fast_feedback_ready(&g_wd.fast_feedback[index], now_ms) == 0) {
      feedback->dq = sign * value;
    }
    g_wd.last_velocity_feedback_ms[index] = now_ms;
  }

  feedback->online = 1u;
  feedback->enabled = g_wd.live_enabled[index];
  feedback->actuator_mode = (uint8_t)(joint ? joint->mode : WD_ACTUATOR_POSITION_PD);
  g_wd.last_can_feedback_ms[index] = now_ms;
}

static void wd_control_apply_can_feedback(const WdCanRxFrame *frame,
                                          uint32_t now_ms) {
  uint32_t comm_type;
  uint32_t motor_id;
  uint32_t index;
  uint32_t fault_mode;
  uint32_t mode_status;
  float p_max;
  float v_max;
  float t_max;
  float raw_position;
  float raw_velocity;
  float raw_tau;
  float sign;
  WdJointFeedback *feedback;
  const WdJointSafetyConfig *joint;

  if (frame == NULL) {
    return;
  }
  ++g_wd.can_rx_frames;
  g_wd.last_can_id = frame->can_id;
  g_wd.last_can_data0_3 = wd_le_u32(&frame->data[0]);
  g_wd.last_can_data4_7 = wd_le_u32(&frame->data[4]);
  g_wd.last_can_meta = ((uint32_t)frame->bus_index & 0xFFu) |
                       (((uint32_t)frame->dlc & 0xFFu) << 8);

  comm_type = (frame->can_id >> 24) & 0x1Fu;
  if (comm_type == WD_ROBSTRIDE_COMM_READ_PARAM) {
    wd_control_apply_read_param_feedback(frame, now_ms);
    return;
  }
  if (comm_type != WD_ROBSTRIDE_COMM_OPERATION_STATUS) {
    return;
  }
  if (frame->dlc < 8u || frame->bus_index >= 4u) {
    ++g_wd.can_rx_bad_frames;
    return;
  }

  motor_id = (frame->can_id >> 8) & 0xFFu;
  if (wd_control_index_from_bus_motor(frame->bus_index, motor_id, &index) == 0u) {
    ++g_wd.can_rx_bad_frames;
    return;
  }

  joint = wd_safety_joint_config(index);
  wd_motor_limits(index, &p_max, &v_max, &t_max);
  raw_position = wd_signed_offset_to_float(wd_be_u16(&frame->data[0]), p_max);
  raw_velocity = wd_signed_offset_to_float(wd_be_u16(&frame->data[2]), v_max);
  raw_tau = wd_signed_offset_to_float(wd_be_u16(&frame->data[4]), t_max);
  sign = wd_motor_sign(index);

  fault_mode = (frame->can_id >> 16) & 0xFFFFu;
  mode_status = (fault_mode >> 6) & 0x03u;

  feedback = &g_wd.feedback[index];
  wd_fast_feedback_note_operation(&g_wd.fast_feedback[index],
                                  now_ms,
                                  raw_position,
                                  raw_velocity);
  if (g_wd.mech_position_valid[index] != 0u) {
    if (joint != NULL && joint->mode == WD_ACTUATOR_POSITION_PD) {
      g_wd.aligned_raw_position[index] =
          wd_motor_align_raw(index, raw_position);
      g_wd.operation_position_valid[index] = 1u;
    } else if (g_wd.operation_position_valid[index] == 0u) {
      /* The wheel is continuous. Anchor its first 0x02 sample to the slow
       * single-turn phase, then unwrap subsequent samples with the confirmed
       * 2*pi encoder period. The wider packet coding range is not a physical
       * wrap period. */
      g_wd.continuous_motor_position[index] =
          g_wd.reference_motor_position[index] +
          wd_motor_raw_wrap_delta(
              raw_position, g_wd.reference_motor_position[index]);
      g_wd.operation_position_valid[index] = 1u;
    } else {
      g_wd.continuous_motor_position[index] +=
          wd_motor_raw_wrap_delta(
              raw_position, g_wd.last_operation_position_raw[index]);
    }
    g_wd.last_operation_position_raw[index] = raw_position;
    if (wd_fast_feedback_ready(&g_wd.fast_feedback[index], now_ms) != 0) {
      if (joint != NULL && joint->mode == WD_ACTUATOR_POSITION_PD) {
        feedback->q = wd_motor_raw_to_joint(index, raw_position);
      } else {
        feedback->q = sign * g_wd.continuous_motor_position[index];
      }
      feedback->dq = sign * raw_velocity;
      ++g_wd.fast_feedback_generation[index];
    }
  }
  feedback->tau = sign * raw_tau;
  feedback->temperature_c = ((float)wd_be_u16(&frame->data[6])) * 0.1f;
  feedback->fault_bits = fault_mode & 0x3Fu;
  feedback->online = 1u;
  feedback->enabled = g_wd.live_enabled[index];
  feedback->actuator_mode = (uint8_t)(joint ? joint->mode : WD_ACTUATOR_POSITION_PD);
  feedback->reserved = (uint8_t)mode_status;
  g_wd.last_can_feedback_ms[index] = now_ms;
}

static void wd_control_drain_can_rx(uint32_t now_ms) {
  WdCanRxFrame frame;

  while (wd_can_rx_pop(&frame) != 0) {
    wd_control_apply_can_feedback(&frame, now_ms);
  }

  g_wd.can_rx_overflows = g_can_rx_overflows;
  if (g_wd.can_rx_overflows != 0u) {
    g_wd.status_flags |= WD_STATUS_CAN_RX_OVERFLOW;
  }
}

static void wd_control_update_feedback_freshness(uint32_t now_ms) {
  uint32_t i;

  for (i = 0u; i < WD_MOTOR_COUNT; ++i) {
    if (g_wd.feedback[i].online != 0u &&
        (now_ms - g_wd.last_can_feedback_ms[i]) > WD_CAN_FEEDBACK_STALE_MS) {
      g_wd.feedback[i].online = 0u;
      g_wd.feedback[i].enabled = 0u;
    }
  }
}

static void wd_control_zero_motor_command(uint32_t index) {
  if (index >= WD_MOTOR_COUNT) {
    return;
  }
  g_wd.joint_torque_cmd_nm[index] = 0.0f;
  g_wd.motor_torque_cmd_nm[index] = 0.0f;
  g_wd.motor_velocity_cmd_radps[index] = 0.0f;
}

static int wd_control_feedback_healthy(uint32_t index) {
  uint32_t now_ms;

  if (index >= WD_MOTOR_COUNT) {
    return 0;
  }
  now_ms = HAL_GetTick();
  return (g_wd.feedback[index].online != 0u &&
          g_wd.feedback[index].fault_bits == 0u &&
          g_wd.mech_position_valid[index] != 0u &&
          g_wd.mech_velocity_valid[index] != 0u &&
          (now_ms - g_wd.last_position_feedback_ms[index]) <=
              WD_CAN_FEEDBACK_STALE_MS &&
          (now_ms - g_wd.last_velocity_feedback_ms[index]) <=
              WD_CAN_FEEDBACK_STALE_MS) ? 1 : 0;
}

static int wd_control_fast_feedback_healthy(uint32_t index, uint32_t now_ms) {
  if (index >= WD_MOTOR_COUNT) {
    return 0;
  }
  return (g_wd.feedback[index].online != 0u &&
          g_wd.feedback[index].fault_bits == 0u &&
          g_wd.mech_position_valid[index] != 0u &&
          wd_fast_feedback_ready(&g_wd.fast_feedback[index], now_ms) != 0) ?
      1 : 0;
}

static float wd_control_absf(float value) {
  return (value < 0.0f) ? -value : value;
}

static float wd_control_apply_velocity_guard(uint32_t index, float tau_cmd) {
  float dq = g_wd.feedback[index].dq;
  const float velocity_limit =
      wd_safety_effective_velocity_limit(index, &g_wd.runtime_config);
  const uint32_t now_ms = HAL_GetTick();

  /*
   * Qualification deliberately runs before fast feedback is marked ready.
   * Still use a fresh operation-status velocity for the hard safety guard: it
   * arrives at 500 Hz, whereas the READ_PARAM cross-check is intentionally
   * much slower.  Taking the larger magnitude is conservative if the two
   * sources briefly disagree; qualification itself still requires them to
   * match before operation-status can drive PD.
   */
  if (index < WD_MOTOR_COUNT &&
      g_wd.fast_feedback[index].operation_rx_count != 0u &&
      (now_ms - g_wd.fast_feedback[index].last_operation_ms) <=
          WD_FAST_FEEDBACK_STALE_MS) {
    const float fast_dq =
        wd_motor_sign(index) *
        g_wd.fast_feedback[index].operation_velocity_radps;
    if (wd_control_absf(fast_dq) > wd_control_absf(dq)) {
      dq = fast_dq;
    }
  }

  if (velocity_limit > 0.0f) {
    const float guarded_tau = wd_safety_derate_torque_for_velocity(
        index, dq, tau_cmd, &g_wd.runtime_config);
    if (guarded_tau != tau_cmd) {
      g_wd.command_status_flags |= WD_STATUS_VELOCITY_GUARD;
    }
    return guarded_tau;
  }
  return tau_cmd;
}

static int wd_control_qualification_command(uint32_t *active_index) {
  uint32_t i;
  uint32_t active = WD_MOTOR_COUNT;

  if (active_index == NULL) {
    return 0;
  }
  for (i = 0u; i < WD_MOTOR_COUNT; ++i) {
    const WdJointSafetyConfig *joint = wd_safety_joint_config(i);
    const WdJointCommand *cmd = &g_wd.setpoint.joint[i];
    int nonzero = 0;

    if (joint == NULL) {
      return 0;
    }
    if (joint->mode == WD_ACTUATOR_VELOCITY) {
      if (wd_control_absf(cmd->dq_des) >
          WD_QUALIFICATION_VELOCITY_MAX_RADPS) {
        return 0;
      }
      nonzero = (wd_control_absf(cmd->dq_des) >
                 WD_QUALIFICATION_COMMAND_EPSILON) ? 1 : 0;
      if ((nonzero != 0 &&
           wd_control_absf(cmd->kd - WD_QUALIFICATION_WHEEL_KD) >
               WD_QUALIFICATION_COMMAND_EPSILON) ||
          (nonzero == 0 &&
           wd_control_absf(cmd->kd) > WD_QUALIFICATION_COMMAND_EPSILON)) {
        return 0;
      }
    } else {
      if (wd_control_absf(cmd->kp) > WD_QUALIFICATION_COMMAND_EPSILON ||
          wd_control_absf(cmd->q_des) > WD_QUALIFICATION_COMMAND_EPSILON ||
          wd_control_absf(cmd->kd) > WD_QUALIFICATION_COMMAND_EPSILON ||
          wd_control_absf(cmd->dq_des) > WD_QUALIFICATION_COMMAND_EPSILON ||
          wd_control_absf(cmd->tau_ff) > WD_QUALIFICATION_TORQUE_MAX_NM) {
        return 0;
      }
      nonzero = (wd_control_absf(cmd->tau_ff) >
                 WD_QUALIFICATION_COMMAND_EPSILON) ? 1 : 0;
    }

    if (nonzero != 0) {
      if (active != WD_MOTOR_COUNT) {
        return 0;
      }
      active = i;
    }
  }
  *active_index = active;
  return 1;
}

static void wd_control_compute_motor_commands(void) {
  uint32_t i;
  uint32_t qualification_active_index = WD_MOTOR_COUNT;
  const uint32_t now_ms = HAL_GetTick();
  const uint32_t blocked =
      (g_wd.status_flags & (WD_STATUS_COMMAND_TIMEOUT | WD_STATUS_ESTOP)) |
      ((g_wd.live_stop_reason_flags != 0u) ? 1u : 0u);
  const uint32_t qualification_requested =
      ((g_wd.control_flags &
        WD_CONTROL_FLAG_QUALIFICATION_EXCITATION) != 0u) ? 1u : 0u;
  const int qualification_valid = (qualification_requested != 0u) ?
      wd_control_qualification_command(&qualification_active_index) : 1;

  g_wd.command_status_flags = 0u;
  ++g_wd.command_compute_ticks;
  if (qualification_valid == 0) {
    g_wd.command_status_flags |= WD_STATUS_BAD_PACKET |
                                 WD_STATUS_LIVE_CONTROL_BLOCKED;
  }

  for (i = 0u; i < WD_MOTOR_COUNT; ++i) {
    const WdJointSafetyConfig *joint = wd_safety_joint_config(i);
    const WdJointCommand *cmd = &g_wd.setpoint.joint[i];
    const float sign = wd_motor_sign(i);

    if (blocked != 0u || joint == NULL) {
      wd_control_zero_motor_command(i);
      g_wd.last_command_feedback_generation[i] =
          g_wd.fast_feedback_generation[i];
      continue;
    }

    if (qualification_requested != 0u) {
      if (qualification_valid == 0 || i != qualification_active_index ||
          wd_control_feedback_healthy(i) == 0) {
        wd_control_zero_motor_command(i);
        continue;
      }
      if (joint->mode == WD_ACTUATOR_VELOCITY) {
        const float dq_cmd = wd_safety_limit_velocity_command(
            i, cmd->dq_des, &g_wd.runtime_config, &g_wd.command_status_flags);
        g_wd.joint_torque_cmd_nm[i] = 0.0f;
        g_wd.motor_torque_cmd_nm[i] = 0.0f;
        g_wd.motor_velocity_cmd_radps[i] = sign * dq_cmd;
      } else {
        WdSafetyRuntimeConfig enable_runtime = g_wd.runtime_config;
        enable_runtime.torque_slew_rate_nm_per_s =
            WD_ENABLE_TORQUE_SLEW_RATE_NM_PER_S;
        float tau_cmd = wd_safety_limit_torque_command(
            i,
            cmd->tau_ff,
            g_wd.joint_torque_cmd_nm[i],
            0.001f,
            &enable_runtime,
            &g_wd.command_status_flags);
        tau_cmd = wd_safety_enforce_raw_position_limit(
            i,
            g_wd.aligned_raw_position[i],
            g_wd.fast_feedback[i].operation_velocity_radps,
            tau_cmd,
            &g_wd.runtime_config,
            &g_wd.command_status_flags);
        tau_cmd = wd_control_apply_velocity_guard(i, tau_cmd);
        g_wd.joint_torque_cmd_nm[i] = tau_cmd;
        g_wd.motor_torque_cmd_nm[i] = sign * tau_cmd;
        g_wd.motor_velocity_cmd_radps[i] = 0.0f;
      }
      g_wd.last_command_compute_ms[i] = now_ms;
      continue;
    }

    if (g_wd.fast_feedback_armed == 0u ||
        wd_control_fast_feedback_healthy(i, now_ms) == 0) {
      wd_control_zero_motor_command(i);
      g_wd.last_command_feedback_generation[i] =
          g_wd.fast_feedback_generation[i];
      continue;
    }

    /* Never recompute from a held sample. Each command update consumes one
     * newly decoded, rate-qualified operation-status feedback generation. */
    if (g_wd.last_command_feedback_generation[i] ==
        g_wd.fast_feedback_generation[i]) {
      continue;
    }
    g_wd.last_command_feedback_generation[i] =
        g_wd.fast_feedback_generation[i];

    if (joint->mode == WD_ACTUATOR_VELOCITY) {
      float dt_seconds = 0.002f;
      float wheel_tau_cmd = 0.0f;
      float motion_velocity_target;
      if (g_wd.last_command_compute_ms[i] != 0u) {
        dt_seconds = (float)(now_ms - g_wd.last_command_compute_ms[i]) * 0.001f;
        if (dt_seconds < 0.001f) {
          dt_seconds = 0.001f;
        } else if (dt_seconds > 0.010f) {
          dt_seconds = 0.010f;
        }
      }
      motion_velocity_target = wd_safety_compute_velocity_motion_target(
          i,
          cmd->dq_des,
          g_wd.feedback[i].dq,
          cmd->kd,
          g_wd.joint_torque_cmd_nm[i],
          dt_seconds,
          &g_wd.runtime_config,
          &g_wd.command_status_flags,
          &wheel_tau_cmd);
      g_wd.joint_torque_cmd_nm[i] = wheel_tau_cmd;
      g_wd.motor_torque_cmd_nm[i] = sign * wheel_tau_cmd;
      g_wd.motor_velocity_cmd_radps[i] = sign * motion_velocity_target;
    } else {
      float dt_seconds = 0.002f;
      if (wd_safety_position_command_is_passive(cmd) != 0) {
        /* Idle deliberately sends kp=kd=tau_ff=0. Keep that request truly
         * zero-current: neither the joint-space nor raw-position limit wall
         * may energize a passive joint. Active commands retain both walls. */
        wd_control_zero_motor_command(i);
        g_wd.last_command_compute_ms[i] = now_ms;
        continue;
      }
      float tau_cmd =
          wd_safety_compute_pd_torque(i,
                                      g_wd.feedback[i].q,
                                      g_wd.feedback[i].dq,
                                      cmd,
                                      g_wd.limit_mode,
                                      &g_wd.runtime_config);
      if (g_wd.last_command_compute_ms[i] != 0u) {
        dt_seconds = (float)(now_ms - g_wd.last_command_compute_ms[i]) * 0.001f;
        if (dt_seconds < 0.001f) {
          dt_seconds = 0.001f;
        } else if (dt_seconds > 0.010f) {
          dt_seconds = 0.010f;
        }
      }
      tau_cmd = wd_safety_limit_torque_command_motion_aware(
          i,
          tau_cmd,
          g_wd.joint_torque_cmd_nm[i],
          g_wd.feedback[i].dq,
          dt_seconds,
          &g_wd.runtime_config,
          &g_wd.command_status_flags);
      tau_cmd = wd_safety_enforce_raw_position_limit(
          i,
          g_wd.aligned_raw_position[i],
          g_wd.fast_feedback[i].operation_velocity_radps,
          tau_cmd,
          &g_wd.runtime_config,
          &g_wd.command_status_flags);
      tau_cmd = wd_control_apply_velocity_guard(i, tau_cmd);
      g_wd.joint_torque_cmd_nm[i] = tau_cmd;
      g_wd.motor_torque_cmd_nm[i] = sign * tau_cmd;
      g_wd.motor_velocity_cmd_radps[i] = 0.0f;
    }
    g_wd.last_command_compute_ms[i] = now_ms;
  }
}

static void wd_control_capture_observation(uint32_t now_ms) {
  uint32_t i;
  uint16_t max_age_ms = 0u;
  uint16_t flags = WD_OBSERVATION_FLAG_VALID;

  ++g_wd.observation_lock;
  __DMB();
  for (i = 0u; i < WD_MOTOR_COUNT; ++i) {
    uint32_t age_ms = 0xFFFFu;
    g_wd.observation_feedback[i] = g_wd.feedback[i];
    if (g_wd.fast_feedback[i].operation_rx_count != 0u) {
      age_ms = now_ms - g_wd.fast_feedback[i].last_operation_ms;
      if (age_ms > 0xFFFFu) {
        age_ms = 0xFFFFu;
      }
    }
    g_wd.observation_sample_age_ms[i] = (uint16_t)age_ms;

    if (
#if WD_CAN_BUS_BASE != 0
        i >= WD_LOCAL_LOGICAL_FIRST_INDEX &&
#endif
        i < WD_LOCAL_LOGICAL_END_INDEX) {
      if (age_ms > max_age_ms) {
        max_age_ms = (uint16_t)age_ms;
      }
      if (age_ms <= WD_FAST_FEEDBACK_STALE_MS &&
          wd_fast_feedback_ready(&g_wd.fast_feedback[i], now_ms) != 0) {
        g_wd.observation_feedback[i].q +=
            g_wd.observation_feedback[i].dq *
            ((float)age_ms * 0.001f);
        if (age_ms != 0u) {
          flags |= WD_OBSERVATION_FLAG_POSITION_EXTRAPOLATED;
        }
      }
    }
  }
  g_wd.observation_seq = g_wd.last_setpoint_seq;
  g_wd.observation_time_ms = now_ms;
  g_wd.observation_max_sample_age_ms = max_age_ms;
  g_wd.observation_flags = flags;
  g_wd.observation_tx_pending = 1u;
  __DMB();
  ++g_wd.observation_lock;
}

void wd_control_service_1khz(void) {
  uint32_t now_ms;
  uint32_t age_ms;
  int setpoint_applied;

  if (g_wd.initialized == 0u) {
    wd_control_init();
  }

  ++g_wd.control_ticks;
  now_ms = HAL_GetTick();
  wd_control_update_hz(now_ms);
  wd_control_drain_can_rx(now_ms);
  wd_control_update_feedback_freshness(now_ms);
  setpoint_applied = wd_control_apply_pending_setpoint();
  if (setpoint_applied != 0) {
    wd_control_capture_observation(now_ms);
  }

  age_ms = now_ms - g_wd.last_setpoint_ms;
  if (age_ms > g_wd.command_timeout_ms) {
    g_wd.status_flags |= WD_STATUS_COMMAND_TIMEOUT;
  } else {
    g_wd.status_flags &= ~WD_STATUS_COMMAND_TIMEOUT;
  }

  g_wd.status_flags &= ~(WD_STATUS_TORQUE_SLEW_LIMITED |
                         WD_STATUS_VELOCITY_GUARD |
                         WD_STATUS_COMMAND_CLIPPED |
                         WD_STATUS_READONLY_POLL |
                         WD_STATUS_LIVE_CONTROL_REQUESTED |
                         WD_STATUS_LIVE_CONTROL_ACTIVE |
                         WD_STATUS_LIVE_ENABLE_IN_PROGRESS |
                         WD_STATUS_LIVE_ENABLE_READY |
                         WD_STATUS_LIVE_SAFETY_STOP |
                         WD_STATUS_FAST_FEEDBACK_READY |
                         WD_STATUS_FAST_FEEDBACK_UNQUALIFIED |
                         WD_STATUS_QUALIFICATION_EXCITATION |
                         WD_STATUS_RAW_POSITION_LIMIT);
  wd_control_service_readonly_poll(now_ms);
  wd_control_compute_motor_commands();
  g_wd.status_flags |= g_wd.command_status_flags;
  wd_control_service_live_can(now_ms);
#if WD_ALLOW_LIVE_CAN_CONTROL
  wd_control_service_motor_telemetry(now_ms);
#endif

  if (g_wd.dry_run != 0u) {
    g_wd.status_flags |= WD_STATUS_DRY_RUN;
  } else {
    g_wd.status_flags &= ~WD_STATUS_DRY_RUN;
  }
  g_wd.status_flags &= ~WD_STATUS_BENCH_RELATIVE_LIMITS;
  if ((g_wd.control_flags &
       WD_CONTROL_FLAG_QUALIFICATION_EXCITATION) != 0u) {
    g_wd.status_flags |= WD_STATUS_QUALIFICATION_EXCITATION;
  }
#if WD_BENCH_REMAP_CAN2_TO_LOGICAL0
  g_wd.status_flags |= WD_STATUS_BENCH_CAN2_REMAP;
#endif
#if WD_CAN_BUS_BASE == 2
  g_wd.status_flags |= WD_STATUS_MCU_CAN_BASE_2;
#endif
  if (g_wd.active != 0u) {
    g_wd.status_flags |= WD_STATUS_ACTIVE;
  } else {
    g_wd.status_flags &= ~WD_STATUS_ACTIVE;
  }
  g_wd.status_flags |= g_wd.sanitize_status_flags;
}

static int wd_control_copy_observation(WdFeedbackPayload *payload) {
  uint32_t attempt;

  if (payload == NULL) {
    return 0;
  }
  for (attempt = 0u; attempt < 4u; ++attempt) {
    uint32_t i;
    const uint32_t begin = g_wd.observation_lock;
    uint32_t end;
    if ((begin & 1u) != 0u) {
      continue;
    }
    for (i = 0u; i < WD_MOTOR_COUNT; ++i) {
      payload->joint[i] = g_wd.observation_feedback[i];
      payload->observation_sample_age_ms[i] =
          g_wd.observation_sample_age_ms[i];
    }
    payload->observation_seq = g_wd.observation_seq;
    payload->observation_time_ms = g_wd.observation_time_ms;
    payload->observation_max_sample_age_ms =
        g_wd.observation_max_sample_age_ms;
    payload->observation_flags = g_wd.observation_flags;
    __DMB();
    end = g_wd.observation_lock;
    if (begin == end && (end & 1u) == 0u &&
        (payload->observation_flags & WD_OBSERVATION_FLAG_VALID) != 0u) {
      return 1;
    }
  }
  return 0;
}

static void wd_control_fill_feedback_payload(WdFeedbackPayload *payload) {
  uint32_t i;
  uint32_t now_ms = HAL_GetTick();
  uint32_t age_ms = now_ms - g_wd.last_setpoint_ms;
  int observation_copied;

  memset(payload, 0, sizeof(*payload));
  payload->status_flags = g_wd.status_flags;
  payload->control_flags = g_wd.control_flags |
      ((g_wd.dry_run != 0u) ? WD_CONTROL_FLAG_DRY_RUN : 0u);
  payload->feedback_time_ms = now_ms;
  payload->last_rx_seq = g_wd.last_rx_seq;
  payload->last_setpoint_seq = g_wd.last_setpoint_seq;
  payload->rx_packets = g_wd.parser.packets;
  payload->rx_crc_errors = g_wd.parser.crc_errors;
  payload->rx_bad_packets = g_wd.parser.bad_packets;
  payload->control_ticks = g_wd.control_ticks;
  payload->setpoint_age_ms = age_ms;
  payload->command_timeout_ms = g_wd.command_timeout_ms;
  payload->actual_control_hz = g_wd.actual_control_hz;
  payload->limit_mode = (uint8_t)g_wd.limit_mode;
  payload->can_rx_frames = g_wd.can_rx_frames;
  payload->can_rx_bad_frames = g_wd.can_rx_bad_frames;
  payload->can_rx_overflows = g_wd.can_rx_overflows;
  payload->can_tx_errors = g_wd.can_tx_errors;
  payload->last_can_id = g_wd.last_can_id;
  payload->last_can_data0_3 = g_wd.last_can_data0_3;
  payload->last_can_data4_7 = g_wd.last_can_data4_7;
  payload->last_can_meta = g_wd.last_can_meta;
  payload->live_stop_reason_flags = g_wd.live_stop_reason_flags;
  payload->live_stop_motor_mask = g_wd.live_stop_motor_mask;
  payload->live_stop_event_count = g_wd.live_stop_event_count;
  payload->live_stop_trigger_time_ms = g_wd.live_stop_trigger_time_ms;
  payload->live_stop_fast_valid_mask = g_wd.live_stop_fast_valid_mask;
  payload->live_stop_fault_mask = g_wd.live_stop_fault_mask;
#if WD_ENABLE_CALIBRATION_READONLY
  if ((g_wd.control_flags & WD_CONTROL_FLAG_CALIBRATION_READONLY) != 0u) {
    /* Setpoint-latched observations precede the read replies; calibration
     * needs the newest raw 0x7019 values instead. */
    observation_copied = 0;
  } else {
    observation_copied = wd_control_copy_observation(payload);
  }
#else
  observation_copied = wd_control_copy_observation(payload);
#endif

  for (i = 0u; i < WD_MOTOR_COUNT; ++i) {
    if (observation_copied == 0) {
      payload->joint[i] = g_wd.feedback[i];
    }
    if (payload->joint[i].online != 0u) {
      payload->online_mask |= (1u << i);
    }
    if (payload->joint[i].enabled != 0u) {
      payload->enabled_mask |= (1u << i);
    }
    if (payload->joint[i].fault_bits != 0u) {
      payload->fault_mask |= (1u << i);
    }
    payload->live_command_tx_queued[i] = g_wd.live_command_tx_queued[i];
    payload->live_command_tx_completed[i] = g_live_command_tx_completed[i];
    payload->live_command_tx_deferred[i] = g_wd.live_command_tx_deferred[i];
    payload->operation_status_rx_count[i] =
        g_wd.fast_feedback[i].operation_rx_count;
    payload->fast_feedback_rate_hz[i] =
        g_wd.fast_feedback[i].measured_rate_hz;
    payload->fast_position_error_rad[i] =
        g_wd.fast_feedback[i].position_error_rad;
    payload->fast_velocity_error_radps[i] =
        g_wd.fast_feedback[i].velocity_error_radps;
    payload->operation_status_max_gap_ms[i] =
        (uint8_t)((g_wd.fast_feedback[i].max_operation_gap_ms > 0xFFu) ?
                      0xFFu : g_wd.fast_feedback[i].max_operation_gap_ms);
    payload->live_stop_fast_age_ms[i] = g_wd.live_stop_fast_age_ms[i];
    payload->final_joint_torque_cmd_nm[i] =
        (g_wd.live_stop_reason_flags != 0u) ?
            g_wd.live_stop_joint_torque_cmd_nm[i] :
            g_wd.joint_torque_cmd_nm[i];
    payload->supply_voltage_v[i] = g_wd.supply_voltage_v[i];
    if (wd_fast_feedback_ready(&g_wd.fast_feedback[i], now_ms) != 0) {
      payload->fast_feedback_valid_mask |= (1u << i);
    }
    if (g_wd.fast_feedback[i].position_reference_seen != 0u &&
        (g_wd.fast_feedback[i].reference_position_max_rad -
         g_wd.fast_feedback[i].reference_position_min_rad) >=
            WD_FAST_POSITION_EXCITATION_RAD) {
      payload->fast_position_excited_mask |= (1u << i);
    }
    if (g_wd.fast_feedback[i].reference_velocity_max_abs_radps >=
        WD_FAST_VELOCITY_EXCITATION_RADPS) {
      payload->fast_velocity_excited_mask |= (1u << i);
    }
  }
  payload->supply_voltage_valid_mask = g_wd.supply_voltage_valid_mask;
}

int wd_control_usb_tx_poll(void) {
  WdFeedbackPayload *payload = &g_feedback_payload;
  uint16_t packet_size = 0u;
  uint32_t now_ms;

  if (g_wd.initialized == 0u) {
    wd_control_init();
  }
  if (g_wd.active == 0u) {
    return 0;
  }

  now_ms = HAL_GetTick();
  if (g_wd.observation_tx_pending == 0u &&
      (now_ms - g_wd.last_feedback_tx_ms) < WD_FEEDBACK_PERIOD_MS) {
    return 1;
  }

  wd_control_fill_feedback_payload(payload);
  if (wd_protocol_build_packet(WD_PACKET_FEEDBACK,
                               ++g_wd.tx_seq,
                               payload,
                               sizeof(*payload),
                               g_tx_buffer,
                               sizeof(g_tx_buffer),
                               &packet_size) != 0) {
    return 1;
  }

  if (CDC_Transmit_FS(g_tx_buffer, packet_size) == USBD_OK) {
    g_wd.last_feedback_tx_ms = now_ms;
    if (payload->observation_seq == g_wd.observation_seq) {
      g_wd.observation_tx_pending = 0u;
    }
  }
  return 1;
}
