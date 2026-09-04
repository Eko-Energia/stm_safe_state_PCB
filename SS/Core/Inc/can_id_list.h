/*
 * can_id_list.h
 */

#ifndef INC_CAN_ID_LIST_H_
#define INC_CAN_ID_LIST_H_

/*
 * SafeState - ID zgodne z CAN_DB.dbc (repo CAN-DATABASE)
 */

#define SAFE_STATE_ACTIV_ID    1    /* BO_ 1  SafeState_Activ    - RX, "frame from all PCBs" */
#define SAFE_STATE_ACCVAL_ID   2    /* BO_ 2  SafeState_AccVal   - cykl 10000 ms */
#define SAFE_STATE_NODE_ID     3    /* BO_ 3  SafeState_NODE     - TX heartbeat, cykl 5000 ms */
#define SAFE_STATE_SYNC_ID     30   /* BO_ 30 SafeState_SyncTick - cykl 10000 ms */
#define SAFE_STATE_END_ID      31   /* BO_ 31 SafeState_END */

/*
 * JETSON - komendy throttle do sterownikow silnikow (CAN_DB.dbc)
 * Sygnal throttle: 0|16@1- czyli int16 little-endian, zakres -32768..32767
 */

#define JETSON_ENGINE_LEFT_RPDO1_ID   550  /* BO_ 550 JETSON_STATIC_EngineLeft_RPDO1  */
#define JETSON_ENGINE_RIGHT_RPDO1_ID  551  /* BO_ 551 JETSON_STATIC_EngineRight_RPDO1 */

/*
 * RCD
 */

#define RCD_ERROR_ID 192
#define RCD_CONVERTER_COMMS_ID 403105268

#endif /* INC_CAN_ID_LIST_H_ */
