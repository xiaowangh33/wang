#ifndef INC_WD_PROTOCOL_H_
#define INC_WD_PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WD_PROTOCOL_MAGIC (0x34504457u) /* "WDP4" little-endian */
#define WD_PROTOCOL_VERSION (1u)
#define WD_PROTOCOL_MAX_PAYLOAD (1104u)
#define WD_PROTOCOL_MAX_PACKET (WD_PROTOCOL_MAX_PAYLOAD + 16u)
#define WD_MOTOR_COUNT (16u)

typedef enum {
  WD_PACKET_HELLO = 1,
  WD_PACKET_SETPOINT = 2,
  WD_PACKET_CONFIG = 3,
  WD_PACKET_ESTOP = 4,
  WD_PACKET_FEEDBACK = 0x81,
  WD_PACKET_STATUS = 0x82,
} WdPacketType;

typedef enum {
  WD_ACTUATOR_POSITION_PD = 0,
  WD_ACTUATOR_VELOCITY = 1,
} WdActuatorMode;

typedef enum {
  /* Deprecated wire value. Firmware coerces it to calibrated absolute mode. */
  WD_LIMIT_STARTUP_RELATIVE = 0,
  /* Calibrated raw encoder zero, sign, URDF coordinates and hard boundaries. */
  WD_LIMIT_CALIBRATED_ABSOLUTE = 1,
} WdLimitMode;

typedef enum {
  WD_CONTROL_FLAG_ENABLE_REQUEST = 1u << 0,
  WD_CONTROL_FLAG_ESTOP = 1u << 1,
  WD_CONTROL_FLAG_DRY_RUN = 1u << 2,
  WD_CONTROL_FLAG_READONLY_POLL = 1u << 3,
  WD_CONTROL_FLAG_READONLY_POLL_CAN2 = 1u << 4,
  WD_CONTROL_FLAG_LIVE_CONTROL = 1u << 5,
  /* Explicit bench-only, one-motor excitation; never used by deployment. */
  WD_CONTROL_FLAG_QUALIFICATION_EXCITATION = 1u << 6,
  /*
   * Calibration-only dry-run mode. Poll both local CAN buses and expose the
   * unmodified 0x7019 mech_pos value in feedback.q. This flag never enables a
   * drive and is ignored unless READONLY_POLL is also set.
   */
  WD_CONTROL_FLAG_CALIBRATION_READONLY = 1u << 7,
} WdControlFlags;

typedef enum {
  WD_STATUS_DRY_RUN = 1u << 0,
  WD_STATUS_ACTIVE = 1u << 1,
  WD_STATUS_COMMAND_TIMEOUT = 1u << 2,
  WD_STATUS_ESTOP = 1u << 3,
  WD_STATUS_CRC_ERROR = 1u << 4,
  WD_STATUS_BAD_PACKET = 1u << 5,
  WD_STATUS_ABSOLUTE_LIMITS_NOT_CALIBRATED = 1u << 6,
  WD_STATUS_CONTROL_TASK_FALLBACK = 1u << 7,
  WD_STATUS_CAN_RX_OVERFLOW = 1u << 8,
  WD_STATUS_SETPOINT_CLIPPED = 1u << 9,
  WD_STATUS_ENABLE_BLOCKED_DRY_RUN = 1u << 10,
  WD_STATUS_TORQUE_SLEW_LIMITED = 1u << 11,
  WD_STATUS_VELOCITY_GUARD = 1u << 12,
  WD_STATUS_COMMAND_CLIPPED = 1u << 13,
  WD_STATUS_READONLY_POLL = 1u << 14,
  WD_STATUS_CAN_TX_ERROR = 1u << 15,
  WD_STATUS_BENCH_CAN2_REMAP = 1u << 16,
  WD_STATUS_LIVE_CONTROL_REQUESTED = 1u << 17,
  WD_STATUS_LIVE_CONTROL_ACTIVE = 1u << 18,
  WD_STATUS_LIVE_ENABLE_IN_PROGRESS = 1u << 19,
  WD_STATUS_LIVE_ENABLE_READY = 1u << 20,
  WD_STATUS_LIVE_CONTROL_BLOCKED = 1u << 21,
  WD_STATUS_LIVE_SAFETY_STOP = 1u << 22,
  WD_STATUS_MCU_CAN_BASE_2 = 1u << 23,
  WD_STATUS_CAN_TX_DEADLINE_MISS = 1u << 24,
  /* All eight local motors have fresh, rate-qualified and cross-checked 0x02. */
  WD_STATUS_FAST_FEEDBACK_READY = 1u << 25,
  /* Live is enabled, but operation-status feedback has not qualified. */
  WD_STATUS_FAST_FEEDBACK_UNQUALIFIED = 1u << 26,
  /* Deprecated diagnostic bit retained for wire compatibility; never set. */
  WD_STATUS_BENCH_RELATIVE_LIMITS = 1u << 27,
  WD_STATUS_QUALIFICATION_EXCITATION = 1u << 28,
  /* Raw encoder is inside its inward soft wall or at a mechanical boundary. */
  WD_STATUS_RAW_POSITION_LIMIT = 1u << 29,
} WdStatusFlags;

typedef enum {
  WD_LIVE_STOP_REASON_FAST_FEEDBACK_LOST = 1u << 0,
  WD_LIVE_STOP_REASON_COMMAND_FEEDBACK_UNHEALTHY = 1u << 1,
  WD_LIVE_STOP_REASON_COMMAND_MOTOR_NOT_ENABLED = 1u << 2,
  WD_LIVE_STOP_REASON_FAST_STALE = 1u << 3,
  WD_LIVE_STOP_REASON_FAST_RATE_LOST = 1u << 4,
  WD_LIVE_STOP_REASON_FAST_REFERENCE_LOST = 1u << 5,
  /* Fresh measured motor velocity exceeded its hard feedback limit. */
  WD_LIVE_STOP_REASON_OVERSPEED = 1u << 6,
} WdLiveStopReasonFlags;

typedef enum {
  WD_OBSERVATION_FLAG_VALID = 1u << 0,
  WD_OBSERVATION_FLAG_POSITION_EXTRAPOLATED = 1u << 1,
} WdObservationFlags;

#pragma pack(push, 1)
typedef struct {
  uint32_t magic;
  uint8_t version;
  uint8_t type;
  uint16_t payload_size;
  uint32_t seq;
  uint16_t crc16;
  uint16_t reserved;
} WdPacketHeader;

typedef struct {
  float kp;
  float q_des;
  float kd;
  float dq_des;
  float tau_ff;
} WdJointCommand;

typedef struct {
  uint32_t control_flags;
  uint32_t command_time_ms;
  uint32_t command_timeout_ms;
  uint8_t limit_mode;
  uint8_t actuator_mode[WD_MOTOR_COUNT];
  uint8_t reserved[3];
  WdJointCommand joint[WD_MOTOR_COUNT];
} WdSetpointPayload;

typedef struct {
  float q;
  float dq;
  float tau;
  float temperature_c;
  uint32_t fault_bits;
  uint8_t online;
  uint8_t enabled;
  uint8_t actuator_mode;
  uint8_t reserved;
} WdJointFeedback;

typedef struct {
  uint32_t status_flags;
  uint32_t control_flags;
  uint32_t feedback_time_ms;
  uint32_t last_rx_seq;
  uint32_t last_setpoint_seq;
  uint32_t rx_packets;
  uint32_t rx_crc_errors;
  uint32_t rx_bad_packets;
  uint32_t control_ticks;
  uint32_t setpoint_age_ms;
  uint32_t command_timeout_ms;
  uint32_t online_mask;
  uint32_t enabled_mask;
  uint32_t fault_mask;
  float actual_control_hz;
  uint8_t limit_mode;
  uint8_t reserved[3];
  WdJointFeedback joint[WD_MOTOR_COUNT];
  uint32_t can_rx_frames;
  uint32_t can_rx_bad_frames;
  uint32_t can_rx_overflows;
  uint32_t can_tx_errors;
  uint32_t last_can_id;
  uint32_t last_can_data0_3;
  uint32_t last_can_data4_7;
  uint32_t last_can_meta;
  /*
   * Per-motor live-command diagnostics. "queued" means accepted by a bxCAN
   * TX mailbox; "completed" is incremented by the TX-complete ISR; "deferred"
   * means a 2 ms command deadline found no free mailbox.
   */
  uint32_t live_command_tx_queued[WD_MOTOR_COUNT];
  uint32_t live_command_tx_completed[WD_MOTOR_COUNT];
  uint32_t live_command_tx_deferred[WD_MOTOR_COUNT];
  /*
   * Fast-feedback qualification diagnostics. operation_status_rx_count is
   * measured independently from CAN TX completion, so the PC can prove that
   * every motor supplies fresh feedback near 500 Hz. The mask is dynamic and
   * local to this MCU's eight logical motors.
   */
  uint32_t operation_status_rx_count[WD_MOTOR_COUNT];
  uint32_t fast_feedback_valid_mask;
  uint32_t fast_position_excited_mask;
  uint32_t fast_velocity_excited_mask;
  float fast_feedback_rate_hz[WD_MOTOR_COUNT];
  float fast_position_error_rad[WD_MOTOR_COUNT];
  float fast_velocity_error_radps[WD_MOTOR_COUNT];
  /* Coherent 50 Hz observation latched when last_setpoint_seq is applied. */
  uint32_t observation_seq;
  uint32_t observation_time_ms;
  uint16_t observation_max_sample_age_ms;
  uint16_t observation_flags;
  uint16_t observation_sample_age_ms[WD_MOTOR_COUNT];
  /* Latched at the first post-enable safety stop; cleared on next live entry. */
  uint32_t live_stop_reason_flags;
  uint32_t live_stop_motor_mask;
  uint32_t live_stop_event_count;
  uint32_t live_stop_trigger_time_ms;
  uint32_t live_stop_fast_valid_mask;
  uint32_t live_stop_fault_mask;
  /* Saturating millisecond diagnostics; appended for parser compatibility. */
  uint8_t operation_status_max_gap_ms[WD_MOTOR_COUNT];
  uint8_t live_stop_fast_age_ms[WD_MOTOR_COUNT];
  /*
   * Latest logical-joint torque command after the hard torque ceiling,
   * raw-position wall and leg measured-speed guard. Live RS06/RS01 control
   * has no torque slew limiter.
   * Position-PD channels send this value through the joint-torque/current
   * conversion. Velocity channels report the equivalent torque encoded into
   * the RS01 motion-mode velocity target and Kd. At the first live safety stop,
   * the pre-zero command is latched so the fault packet preserves the command
   * that was active at the trigger instead of reporting cleanup zeroes.
   */
  float final_joint_torque_cmd_nm[WD_MOTOR_COUNT];
} WdFeedbackPayload;
#pragma pack(pop)

typedef void (*WdProtocolPacketCallback)(uint8_t type,
                                         uint32_t seq,
                                         const uint8_t *payload,
                                         uint16_t payload_size,
                                         void *context);

typedef struct {
  uint8_t buffer[WD_PROTOCOL_MAX_PACKET];
  uint16_t size;
  uint32_t packets;
  uint32_t crc_errors;
  uint32_t bad_packets;
} WdProtocolParser;

void wd_protocol_parser_init(WdProtocolParser *parser);
int wd_protocol_parser_has_pending(const WdProtocolParser *parser);
int wd_protocol_buffer_starts_like_packet(const uint8_t *data, uint32_t len);
uint16_t wd_protocol_crc16_ccitt(const uint8_t *data, size_t len);
int wd_protocol_build_packet(uint8_t type,
                             uint32_t seq,
                             const void *payload,
                             uint16_t payload_size,
                             uint8_t *out,
                             uint16_t out_capacity,
                             uint16_t *out_size);
int wd_protocol_feed(WdProtocolParser *parser,
                     const uint8_t *data,
                     uint32_t len,
                     WdProtocolPacketCallback callback,
                     void *context);

#ifdef __cplusplus
}
#endif

#endif /* INC_WD_PROTOCOL_H_ */
