#include "wd_can_scheduler.h"

#include <stddef.h>

static int wd_can_scheduler_time_due(uint32_t now_ms, uint32_t due_ms) {
  return ((int32_t)(now_ms - due_ms) >= 0) ? 1 : 0;
}

void wd_can_scheduler_init(WdCanScheduler *scheduler, uint32_t now_ms) {
  uint32_t bus;
  uint32_t motor;

  if (scheduler == NULL) {
    return;
  }

  for (bus = 0u; bus < WD_CAN_SCHEDULER_BUS_COUNT; ++bus) {
    for (motor = 0u; motor < WD_CAN_SCHEDULER_MOTORS_PER_BUS; ++motor) {
      const uint32_t slot =
          bus * WD_CAN_SCHEDULER_MOTORS_PER_BUS + motor;
      /*
       * Stagger two motors per bus on alternating 1 ms ticks. Every motor is
       * therefore due once per 2 ms (500 Hz), while each CAN controller only
       * needs to accept two normal command frames per tick.
       */
      scheduler->next_due_ms[slot] = now_ms + (motor & 1u);
    }
  }
  scheduler->initialized = 1u;
}

uint32_t wd_can_scheduler_collect_due(const WdCanScheduler *scheduler,
                                      uint32_t now_ms,
                                      uint8_t local_bus,
                                      uint8_t slots[WD_CAN_SCHEDULER_MOTORS_PER_BUS]) {
  uint32_t count = 0u;
  uint32_t motor;
  uint32_t first;

  if (scheduler == NULL || slots == NULL || scheduler->initialized == 0u ||
      local_bus >= WD_CAN_SCHEDULER_BUS_COUNT) {
    return 0u;
  }

  first = (uint32_t)local_bus * WD_CAN_SCHEDULER_MOTORS_PER_BUS;
  for (motor = 0u; motor < WD_CAN_SCHEDULER_MOTORS_PER_BUS; ++motor) {
    const uint32_t slot = first + motor;
    if (wd_can_scheduler_time_due(now_ms, scheduler->next_due_ms[slot]) != 0) {
      slots[count++] = (uint8_t)slot;
    }
  }
  return count;
}

void wd_can_scheduler_mark_queued(WdCanScheduler *scheduler,
                                  uint8_t slot,
                                  uint32_t now_ms) {
  uint32_t next_due;

  if (scheduler == NULL || scheduler->initialized == 0u ||
      slot >= WD_CAN_SCHEDULER_MOTOR_COUNT) {
    return;
  }

  next_due = scheduler->next_due_ms[slot] +
             WD_CAN_SCHEDULER_COMMAND_PERIOD_MS;
  /*
   * Keep the absolute schedule during a one-tick mailbox delay so the third
   * bxCAN mailbox can catch up. If the loop was stalled for a long time, skip
   * obsolete deadlines instead of bursting an unbounded number of old frames.
   */
  if ((int32_t)(now_ms - next_due) >=
      (int32_t)(4u * WD_CAN_SCHEDULER_COMMAND_PERIOD_MS)) {
    next_due = now_ms + WD_CAN_SCHEDULER_COMMAND_PERIOD_MS;
  }
  scheduler->next_due_ms[slot] = next_due;
}

int wd_can_scheduler_failed_queue_is_deadline_miss(
    const WdCanScheduler *scheduler,
    uint8_t slot,
    uint32_t now_ms) {
  if (scheduler == NULL || scheduler->initialized == 0u ||
      slot >= WD_CAN_SCHEDULER_MOTOR_COUNT) {
    return 1;
  }
  return ((int32_t)(now_ms - scheduler->next_due_ms[slot]) >=
          (int32_t)WD_CAN_SCHEDULER_DEFER_GRACE_MS) ? 1 : 0;
}
