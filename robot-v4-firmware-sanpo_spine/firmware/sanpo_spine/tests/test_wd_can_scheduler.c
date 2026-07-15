#include "wd_can_scheduler.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static void test_nominal_500_hz(void) {
  WdCanScheduler scheduler = {0};
  uint32_t count[WD_CAN_SCHEDULER_MOTOR_COUNT] = {0};
  uint32_t now_ms;

  wd_can_scheduler_init(&scheduler, 0u);
  for (now_ms = 0u; now_ms < 1000u; ++now_ms) {
    uint8_t bus;
    for (bus = 0u; bus < WD_CAN_SCHEDULER_BUS_COUNT; ++bus) {
      uint8_t slots[WD_CAN_SCHEDULER_MOTORS_PER_BUS];
      uint32_t due = wd_can_scheduler_collect_due(&scheduler, now_ms, bus, slots);
      uint32_t i;
      assert(due == 2u);
      for (i = 0u; i < due; ++i) {
        ++count[slots[i]];
        wd_can_scheduler_mark_queued(&scheduler, slots[i], now_ms);
      }
    }
  }

  for (now_ms = 0u; now_ms < WD_CAN_SCHEDULER_MOTOR_COUNT; ++now_ms) {
    assert(count[now_ms] == 500u);
  }
}

static void test_one_tick_mailbox_catchup(void) {
  WdCanScheduler scheduler = {0};
  uint8_t slots[WD_CAN_SCHEDULER_MOTORS_PER_BUS];
  uint32_t due;
  uint32_t i;

  wd_can_scheduler_init(&scheduler, 100u);
  due = wd_can_scheduler_collect_due(&scheduler, 100u, 0u, slots);
  assert(due == 2u);

  /* Simulate one accepted frame and one full mailbox. */
  wd_can_scheduler_mark_queued(&scheduler, slots[0], 100u);
  assert(wd_can_scheduler_failed_queue_is_deadline_miss(
             &scheduler, slots[1], 100u) == 0);
  due = wd_can_scheduler_collect_due(&scheduler, 101u, 0u, slots);
  assert(due == 3u);
  for (i = 0u; i < due; ++i) {
    wd_can_scheduler_mark_queued(&scheduler, slots[i], 101u);
  }
  due = wd_can_scheduler_collect_due(&scheduler, 102u, 0u, slots);
  assert(due == 2u);
}

static void test_five_ms_mailbox_deferral_grace(void) {
  WdCanScheduler scheduler = {0};
  uint8_t slots[WD_CAN_SCHEDULER_MOTORS_PER_BUS];
  uint32_t due;
  uint8_t deferred_slot;

  wd_can_scheduler_init(&scheduler, 200u);
  due = wd_can_scheduler_collect_due(&scheduler, 200u, 1u, slots);
  assert(due == 2u);
  deferred_slot = slots[1];
  assert(wd_can_scheduler_failed_queue_is_deadline_miss(
             &scheduler, deferred_slot, 200u) == 0);
  assert(wd_can_scheduler_failed_queue_is_deadline_miss(
             &scheduler, deferred_slot, 204u) == 0);
  assert(wd_can_scheduler_failed_queue_is_deadline_miss(
             &scheduler, deferred_slot, 205u) != 0);
}

static void test_tick_wrap(void) {
  WdCanScheduler scheduler = {0};
  uint8_t slots[WD_CAN_SCHEDULER_MOTORS_PER_BUS];
  uint32_t due;
  uint32_t now_ms = UINT32_MAX;

  wd_can_scheduler_init(&scheduler, now_ms);
  due = wd_can_scheduler_collect_due(&scheduler, now_ms, 1u, slots);
  assert(due == 2u);
  wd_can_scheduler_mark_queued(&scheduler, slots[0], now_ms);
  wd_can_scheduler_mark_queued(&scheduler, slots[1], now_ms);
  due = wd_can_scheduler_collect_due(&scheduler, 0u, 1u, slots);
  assert(due == 2u);
}

int main(void) {
  test_nominal_500_hz();
  test_one_tick_mailbox_catchup();
  test_five_ms_mailbox_deferral_grace();
  test_tick_wrap();
  puts("wd_can_scheduler: all tests passed");
  return 0;
}
