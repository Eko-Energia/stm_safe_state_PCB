/**
  * @file can_driver.c
  * @brief CAN bus driver for PERLA
  * @author AGH EKO-ENERGIA
  * @author Kacper Lasota
  */

/*
 * TODO
 *
 * Error handling both on bus and generic error messages
 * Filter configuration
 * Received messages handling
 *
 */
#include "can_driver.h"

/* Include error handler if available */
#if __has_include("error_handler.h")
#include "error_handler.h"
#define ERROR_HANDLER_AVAILABLE (1)
#else
#define ERROR_HANDLER_AVAILABLE (0)
#endif

/**
 * @brief Configure one filter bank as a list of accepted standard IDs
 */
HAL_StatusTypeDef CAN_setStdIdListFilter(CAN_HandleTypeDef *hcanPtr,
                                         uint8_t bank,
                                         const uint16_t *stdIds,
                                         uint8_t count)
{
	if (hcanPtr == NULL || stdIds == NULL ||
		count == 0U || count > CAN_FILTER_IDS_PER_BANK ||
		bank > CAN_FILTER_BANK_MAX)
	{
		return HAL_ERROR;
	}

	uint16_t slot[CAN_FILTER_IDS_PER_BANK];
	for (uint8_t i = 0U; i < CAN_FILTER_IDS_PER_BANK; i++)
	{
		/* Wolne sloty duplikuja ostatnie ID - slot pozostawiony jako 0x0000
		 * akceptowalby ramke o ID 0 */
		uint16_t id = (i < count) ? stdIds[i] : stdIds[count - 1U];

		if (id > CAN_STD_ID_MAX)
		{
			return HAL_ERROR;
		}
		slot[i] = CAN_STD_ID(id);
	}

	CAN_FilterTypeDef filterConfig = {0};

	filterConfig.FilterBank = bank;
	filterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	filterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	filterConfig.FilterIdHigh = slot[0];
	filterConfig.FilterIdLow = slot[1];
	filterConfig.FilterMaskIdHigh = slot[2];
	filterConfig.FilterMaskIdLow = slot[3];
	filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filterConfig.FilterActivation = CAN_FILTER_ENABLE;
	filterConfig.SlaveStartFilterBank = 14;

	return HAL_CAN_ConfigFilter(hcanPtr, &filterConfig);
}

/**
 * @brief Configure bank 0 to accept every frame on the bus
 */
static HAL_StatusTypeDef CAN_setAcceptAllFilter(CAN_HandleTypeDef *hcanPtr)
{
	CAN_FilterTypeDef filterConfig = {0};

	/* Maska = 0 -> zaden bit ID nie jest porownywany */
	filterConfig.FilterBank = 0;
	filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	filterConfig.FilterIdHigh = 0x0000;
	filterConfig.FilterIdLow = 0x0000;
	filterConfig.FilterMaskIdHigh = 0x0000;
	filterConfig.FilterMaskIdLow = 0x0000;
	filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filterConfig.FilterActivation = CAN_FILTER_ENABLE;
	filterConfig.SlaveStartFilterBank = 14;

	return HAL_CAN_ConfigFilter(hcanPtr, &filterConfig);
}

/**
 * @brief Initialize CAN: filters, notifications (RX + bus errors), start
 */
HAL_StatusTypeDef CAN_init(CAN_HandleTypeDef *hcanPtr,
                           const uint16_t *acceptedStdIds,
                           uint8_t idCount)
{
	if (hcanPtr == NULL)
	{
		return HAL_ERROR;
	}

	if (acceptedStdIds == NULL || idCount == 0U)
	{
		if (CAN_setAcceptAllFilter(hcanPtr) != HAL_OK)
		{
			return HAL_ERROR;
		}
	}
	else
	{
		/* Kazde 4 ID zajmuja jeden bank filtra */
		uint8_t bank = 0U;
		for (uint8_t i = 0U; i < idCount; i += CAN_FILTER_IDS_PER_BANK)
		{
			uint8_t chunk = idCount - i;
			if (chunk > CAN_FILTER_IDS_PER_BANK)
			{
				chunk = CAN_FILTER_IDS_PER_BANK;
			}

			if (CAN_setStdIdListFilter(hcanPtr, bank, &acceptedStdIds[i], chunk) != HAL_OK)
			{
				return HAL_ERROR;
			}
			bank++;
		}
	}

	/* CAN_IT_ERROR (ERRIE) musi byc aktywne, by przerwania
	 * warning/passive/bus-off w ogole byly generowane */
	if (HAL_CAN_ActivateNotification(hcanPtr,
			CAN_IT_RX_FIFO0_MSG_PENDING |
			CAN_IT_ERROR |
			CAN_IT_ERROR_WARNING |
			CAN_IT_ERROR_PASSIVE |
			CAN_IT_BUSOFF) != HAL_OK)
	{
		return HAL_ERROR;
	}

	return HAL_CAN_Start(hcanPtr);
}

/**
 * @brief Add new message to the periodic buffer
 */
HAL_StatusTypeDef CAN_addScheduledMessage(struct CAN_scheduledMsg msg, struct CAN_scheduledMsgList *buffer)
{
	// basic error checking
	if (buffer == NULL || buffer->size >= CAN_MAX_MSG)
	{
		return HAL_ERROR;
	}
	if (msg.periodMs == 0 || msg.header.DLC > CAN_MAX_DLC)
	{
		return HAL_ERROR;
	}

	msg.lastTick = HAL_GetTick();

	// check if id already exists in the buffer
	for (int i = 0; i < buffer->size; i++)
	{
		if ((buffer->list[i].header.IDE == CAN_ID_STD && buffer->list[i].header.StdId == msg.header.StdId) ||
			(buffer->list[i].header.IDE == CAN_ID_EXT && buffer->list[i].header.ExtId == msg.header.ExtId))
		{
			return HAL_ERROR;
		}
	}

	buffer->list[buffer->size] = msg;
	buffer->size++;
	return HAL_OK;
}

/*
 * @brief Remove message from the periodic buffer
 */
HAL_StatusTypeDef CAN_removeScheduledMessage(uint32_t id, struct CAN_scheduledMsgList *buffer)
{
	if (buffer == NULL)
	{
		return HAL_ERROR;
	}

	for (uint8_t i = 0; i < buffer->size; i++)
	{
		if ((buffer->list[i].header.IDE == CAN_ID_STD && buffer->list[i].header.StdId == id) ||
			(buffer->list[i].header.IDE == CAN_ID_EXT && buffer->list[i].header.ExtId == id))
		{
			for (uint8_t j = i; j + 1 < buffer->size; j++)
			{
				buffer->list[j] = buffer->list[j + 1];
			}
			buffer->size--;
			return HAL_OK;
		}
	}

	return HAL_ERROR;
}

/**
 * @brief Process all scheduled CAN messages (call in main loop)
 */
void CAN_handleScheduled(CAN_HandleTypeDef *hcanPtr, struct CAN_scheduledMsgList *scheduler)
{
	if (hcanPtr == NULL || scheduler == NULL)
	{
		return;
	}

	uint32_t currentTick = HAL_GetTick();
	for (uint8_t i = 0; i < scheduler->size; i++)
	{
		struct CAN_scheduledMsg *msg = &scheduler->list[i];

		/* Odejmowanie unsigned - odporne na przepelnienie HAL_GetTick() */
		if ((uint32_t)(currentTick - msg->lastTick) < msg->periodMs)
		{
			continue;
		}

		/* Wszystkie 3 mailboxy TX zajete (np. error passive / brak ACK) -
		 * nie probujemy dalej, wyslemy w nastepnym obiegu petli */
		if (HAL_CAN_GetTxMailboxesFreeLevel(hcanPtr) == 0U)
		{
			return;
		}

		uint8_t data[CAN_MAX_DLC] = {0};
		if (msg->getData != NULL)
		{
			msg->getData(data, msg->context);
		}

		if (HAL_CAN_AddTxMessage(hcanPtr, &msg->header, data, &scheduler->txMailbox) != HAL_OK)
		{
			return;
		}

		msg->lastTick = currentTick;
	}
}
/**
 * @brief Send a single standard data frame immediately (event-driven)
 */
HAL_StatusTypeDef CAN_sendStdFrame(CAN_HandleTypeDef *hcanPtr, uint16_t stdId,
                                   const uint8_t *data, uint8_t dlc)
{
	if (hcanPtr == NULL || stdId > CAN_STD_ID_MAX || dlc > CAN_MAX_DLC)
	{
		return HAL_ERROR;
	}
	if (dlc > 0U && data == NULL)
	{
		return HAL_ERROR;
	}

	/* Brak wolnej skrzynki - niech wolajacy sprobuje w kolejnym obiegu petli */
	if (HAL_CAN_GetTxMailboxesFreeLevel(hcanPtr) == 0U)
	{
		return HAL_BUSY;
	}

	CAN_TxHeaderTypeDef header = {0};
	header.StdId = stdId;
	header.ExtId = 0;
	header.IDE = CAN_ID_STD;
	header.RTR = CAN_RTR_DATA;
	header.DLC = dlc;
	header.TransmitGlobalTime = DISABLE;

	/* Kopia lokalna - HAL_CAN_AddTxMessage przyjmuje wskaznik bez const */
	uint8_t payload[CAN_MAX_DLC] = {0};
	for (uint8_t i = 0U; i < dlc; i++)
	{
		payload[i] = data[i];
	}

	uint32_t txMailbox;
	return HAL_CAN_AddTxMessage(hcanPtr, &header, payload, &txMailbox);
}

/**
 * @brief Rejestruje handler dla danego ID ramki
 */
HAL_StatusTypeDef CAN_addRxHandler(struct CAN_rxHandler handler,
                                   struct CAN_rxHandlerList *table)
{
    if (table == NULL || table->size >= CAN_MAX_MSG) {
        return HAL_ERROR;
    }

    /* Sprawdź duplikat ID */
    for (uint8_t i = 0; i < table->size; i++) {
        if (table->list[i].ide == handler.ide &&
            table->list[i].id  == handler.id) {
            return HAL_ERROR;  /* już zarejestrowany */
        }
    }

    table->list[table->size] = handler;
    table->size++;
    return HAL_OK;
}

/**
 * @brief Czyta ramkę z FIFO i wywołuje odpowiedni callback
 *        Wywołuj z HAL_CAN_RxFifo0MsgPendingCallback !
 */
void CAN_dispatchRx(CAN_HandleTypeDef *hcanPtr,
                    struct CAN_rxHandlerList *table)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxMessage(hcanPtr, CAN_RX_FIFO0,
                             &rxHeader, rxData) != HAL_OK) {
        return;
    }

    uint32_t rxId  = (rxHeader.IDE == CAN_ID_STD)
                     ? rxHeader.StdId : rxHeader.ExtId;

    for (uint8_t i = 0; i < table->size; i++) {
        if (table->list[i].ide == rxHeader.IDE &&
            table->list[i].id  == rxId) {
            if (table->list[i].callback != NULL) {
                table->list[i].callback(rxData, rxHeader.DLC,
                                        table->list[i].context);
            }
            return;  /* jeden handler na ID — wychodzimy */
        }
    }
    /* Brak handlera — ramka zignorowana (możesz tu dodać log) */
}
