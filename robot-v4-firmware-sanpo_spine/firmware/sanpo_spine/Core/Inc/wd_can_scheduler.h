#ifndef INC_WD_CAN_SCHEDULER_H_
#define INC_WD_CAN_SCHEDULER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WD_CAN_SCHEDULER_BUS_COUNT (2u)
#define WD_CAN_SCHEDULER_MOTORS_PER_BUS (4u)
#define WD_CAN_SCHEDULER_MOTOR_COUNT \
  (WD_CAN_SCHEDULER_BUS_COUNT * WD_CAN_SCHEDULER_MOTORS_PER_BUS)
#define WD_CAN_SCHEDULER_COMMAND_PERIOD_MS (2u)
/* Flat-ground feasibility profile: tolerate short bxCAN mailbox jitter for up
 * to 5 ms relative to the planned slot. Fresh feedback is still never reused
 * after WD_FAST_FEEDBACK_STALE_MS, so this does not authorize stale-state PD. */
#define WD_CAN_SCHEDULER_DEFER_GRACE_MS (5u)

typedef struct {
  uint32_t next_due_ms[WD_CAN_SCHEDULER_MOTOR_COUNT];
  uint8_t initialized;
} WdCanScheduler;

void wd_can_scheduler_init(WdCanScheduler *scheduler, uint32_t now_ms);
uint32_t wd_can_scheduler_collect_due(const WdCanScheduler *scheduler,
                                      uint32_t now_ms,
                                      uint8_t local_bus,
                                      uint8_t slots[WD_CAN_SCHEDULER_MOTORS_PER_BUS]);
void wd_can_scheduler_mark_queued(WdCanScheduler *scheduler,
                                  uint8_t slot,
                                  uint32_t now_ms);
int wd_can_scheduler_failed_queue_is_deadline_miss(
    const WdCanScheduler *scheduler,
    uint8_t slot,
    uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* INC_WD_CAN_SCHEDULER_H_ */
