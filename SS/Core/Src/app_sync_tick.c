/**
 * @file app_sync_tick_test.c
 * @brief TEMPORARY: broadcast SafeState_SyncTick (CAN ID 30) every 10 s.
 *
 * DBC: BO_ 30 SafeState_SyncTick, SG_ SyncTick : 0|32@1+ (ms)
 * Remove after testing.
 */
#include <app_sync_tick.h>

#define SYNC_TICK_FRAME_ID   (30u)
#define SYNC_TICK_PERIOD_MS  (10000u)
#define SYNC_TICK_DLC        (8u)

static void SyncTickTest_GetData(uint8_t *data, void *context)
{
    (void)context;
    const uint32_t tick = HAL_GetTick();
    data[0] = GET_BYTE(tick, 0);
    data[1] = GET_BYTE(tick, 1);
    data[2] = GET_BYTE(tick, 2);
    data[3] = GET_BYTE(tick, 3);
}

void SyncTickTest_Init(struct CAN_scheduledMsgList *scheduler)
{
    struct CAN_scheduledMsg msg = {
        .header = {
            .StdId = SYNC_TICK_FRAME_ID,
            .ExtId = 0,
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = SYNC_TICK_DLC,
            .TransmitGlobalTime = DISABLE,
        },
        .periodMs = SYNC_TICK_PERIOD_MS,
        .lastTick = 0,
        .getData = SyncTickTest_GetData,
        .context = NULL,
    };

    (void)CAN_addScheduledMessage(msg, scheduler);
}
