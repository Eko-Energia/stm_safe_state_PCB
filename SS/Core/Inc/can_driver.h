/**
 * @file can_driver.h
 * @brief CAN bus driver for PERLA
 * @author AGH EKO-ENERGIA
 * @author Kacper Lasota
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "main.h"
#include "can_id_list.h"
#include <stdio.h>

/**
 * Defines
 */

#define CAN_MAX_DLC (8)
#define CAN_MAX_MSG (32)

/** Najwieksze dopuszczalne standardowe (11-bitowe) ID */
#define CAN_STD_ID_MAX (0x7FFu)

/** Liczba ID mieszczacych sie w jednym banku filtra w trybie IDLIST + 16BIT */
#define CAN_FILTER_IDS_PER_BANK (4u)

/** Ostatni bank filtra dostepny dla ukladu z pojedynczym CAN (banki 0..13) */
#define CAN_FILTER_BANK_MAX (13u)

/**
 * @brief Buduje zawartosc 16-bitowego slotu filtra dla standardowego ID
 *
 * Uklad polowki rejestru filtra w skali 16-bitowej:
 * [15:5] STID[10:0] | [4] RTR | [3] IDE | [2:0] EXID[17:15]
 *
 * Standardowa ramka danych ma RTR = 0 i IDE = 0, wiec sprowadza sie to do
 * przesuniecia ID o 5 bitow w lewo. Efekt uboczny (pozadany): ramki zdalne
 * (RTR = 1) o tym samym ID nie zostana przyjete.
 */
#define CAN_STD_ID(id) ((uint16_t)(((uint32_t)(id) & CAN_STD_ID_MAX) << 5u))

/**
 * @brief Generic macro to swap endianness based on variable type.~
 * 
 * Endiannes should be handlend in GetData function of every 
 * * usage: 
 * uint32_t val = 0x12345678;
 * val = SWAP_ENDIANNESS(val); // Becomes 0x78563412
 */
#define SWAP_ENDIANNESS(x) _Generic((x),       \
    uint8_t:  (x),                             \
    int8_t:   (x),                             \
    uint16_t: __builtin_bswap16(x),                  \
    int16_t:  __builtin_bswap16(x),                  \
    uint32_t: __builtin_bswap32(x),                  \
    int32_t:  __builtin_bswap32(x),                  \
    uint64_t: __builtin_bswap64(x),                  \
    int64_t:  __builtin_bswap64(x)                   \
)

/**
 * @brief Extracts the n-th byte from variable x.
 * @warning Do not pass expressions with side effects (e.g., x++) as arguments,
 * as they may be evaluated multiple times.
 * @param x The source variable (uint8_t, uint16_t, or uint32_t).
 * @param n The byte index (0 for LSB).
 */
#define GET_BYTE(x, n) ((uint8_t)(((x) >> ((n) * 8u)) & 0xFFu))

/**
 * Periodic CAN message
 */
struct CAN_scheduledMsg
{
	CAN_TxHeaderTypeDef header;     // frame header
	uint32_t periodMs;              // period of this message
	uint32_t lastTick;              // time stamp of the last message
	void (*getData)(uint8_t *data, void *context); // fetches data
	void *context;                  // user callback context
};

/**
 * Periodic CAN message list used for automation
 */
struct CAN_scheduledMsgList
{
	struct CAN_scheduledMsg list[CAN_MAX_MSG];
	uint8_t size;
	uint32_t txMailbox;
};

/**
 * Setup functions
 */

/**
 * @brief Initialize CAN
 *
 * Configures the RX filters, activates RX and bus-error notifications
 * (error warning / passive / bus-off) and starts the peripheral. The
 * application should implement HAL_CAN_ErrorCallback() to react to bus
 * errors.
 *
 * @param hcan            Pointer to CAN handle
 * @param acceptedStdIds  Array of standard IDs the node accepts. Frames with
 *                        any other ID are dropped by hardware and never reach
 *                        HAL_CAN_RxFifo0MsgPendingCallback(). Pass NULL to
 *                        accept every frame on the bus (promiscuous mode).
 * @param idCount         Number of entries in @p acceptedStdIds (0 with NULL).
 *                        Each 4 IDs consume one filter bank.
 * @return HAL_OK on success, HAL_ERROR otherwise
 */
HAL_StatusTypeDef CAN_init(CAN_HandleTypeDef *hcan,
                           const uint16_t *acceptedStdIds,
                           uint8_t idCount);

/**
 * @brief Configure one filter bank as a list of accepted standard IDs
 *
 * Uses IDLIST + 16-bit scale, so a single bank holds up to four IDs. Unused
 * slots are filled with a duplicate of the last ID, because an empty slot
 * reads as 0x0000 and would silently accept the frame with ID 0.
 *
 * @param hcan    Pointer to CAN handle
 * @param bank    Filter bank index (0..CAN_FILTER_BANK_MAX)
 * @param stdIds  Array of standard IDs to accept
 * @param count   Number of IDs, 1..CAN_FILTER_IDS_PER_BANK
 * @return HAL_OK on success, HAL_ERROR on invalid arguments or HAL failure
 */
HAL_StatusTypeDef CAN_setStdIdListFilter(CAN_HandleTypeDef *hcan,
                                         uint8_t bank,
                                         const uint16_t *stdIds,
                                         uint8_t count);

/**
 * @brief Get configured Node ID
 * @return Current Node ID
 */
uint32_t CAN_getNodeId(void);

/**
 * @brief Process all scheduled CAN messages (call in main loop)
 */
void CAN_handleScheduled(CAN_HandleTypeDef *hcanPtr, struct CAN_scheduledMsgList *scheduler);

/**
 * @brief Send a single standard data frame immediately (event-driven)
 *
 * For one-shot frames that are not part of the periodic scheduler, e.g. a
 * safe-state broadcast triggered by a sensor reading.
 *
 * @param hcan   Pointer to CAN handle
 * @param stdId  Standard (11-bit) identifier
 * @param data   Payload, may be NULL when @p dlc is 0
 * @param dlc    Payload length, 0..CAN_MAX_DLC
 * @retval HAL_OK     frame handed over to a free TX mailbox
 * @retval HAL_BUSY   all three mailboxes occupied - retry on the next pass
 * @retval HAL_ERROR  invalid arguments or HAL failure
 */
HAL_StatusTypeDef CAN_sendStdFrame(CAN_HandleTypeDef *hcan, uint16_t stdId,
                                   const uint8_t *data, uint8_t dlc);

/**
 * Functions for scheduled messages
 */
HAL_StatusTypeDef CAN_addScheduledMessage(struct CAN_scheduledMsg msg, struct CAN_scheduledMsgList *buffer);

HAL_StatusTypeDef CAN_removeScheduledMessage(uint32_t id, struct CAN_scheduledMsgList *buffer);

/* ---- RX dispatch ---- */
typedef void (*CAN_RxCallback)(uint8_t *data, uint8_t dlc, void *context);

struct CAN_rxHandler {
    uint32_t id;               // ID ramki do obsługi
    uint8_t  ide;              // CAN_ID_STD lub CAN_ID_EXT
    CAN_RxCallback callback;   // funkcja wywoływana po odebraniu
    void *context;             // opcjonalny kontekst (może być NULL)
};

struct CAN_rxHandlerList {
    struct CAN_rxHandler list[CAN_MAX_MSG];
    uint8_t size;
};

/* Funkcje RX */
HAL_StatusTypeDef CAN_addRxHandler(struct CAN_rxHandler handler,
                                   struct CAN_rxHandlerList *table);

void CAN_dispatchRx(CAN_HandleTypeDef *hcanPtr,
                    struct CAN_rxHandlerList *table);


#endif /* INC_CAN_DRIVER_H_ */
