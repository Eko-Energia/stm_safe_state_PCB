/**
 * @file app_sync_tick_test.h
 * @brief TEMPORARY: broadcast SafeState_SyncTick for LED sync testing.
 *
 * This board normally acts as a slave. This module temporarily acts as the
 * sync-tick master so other ECUs' led_driver can sync without SafeState.
 *
 * Remove this file and its call site in app.c after testing.
 */
#ifndef APP_SYNC_TICK_TEST_H
#define APP_SYNC_TICK_TEST_H

#include "can_driver.h"

void SyncTickTest_Init(struct CAN_scheduledMsgList *scheduler);

#endif /* APP_SYNC_TICK_TEST_H */
