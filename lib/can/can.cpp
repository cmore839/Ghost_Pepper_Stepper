#include "can.h"
#include <string.h>
#include <SimpleFOC.h>

static FDCAN_HandleTypeDef hfdcan1;
static FDCAN_TxHeaderTypeDef TxHeader;

// Helper function to convert byte count to the required HAL DLC enum
static uint32_t bytesToDLC(uint8_t bytes) {
    switch (bytes) {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        case 8: return FDCAN_DLC_BYTES_8;
        default: return FDCAN_DLC_BYTES_8; // Default to 8 for safety
    }
}

void CAN_Init(uint8_t can_id) {
    SIMPLEFOC_DEBUG("CAN: Initializing FDCAN peripheral...");
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = ENABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;

    hfdcan1.Init.NominalPrescaler = 1;
    hfdcan1.Init.NominalSyncJumpWidth = 4;
    hfdcan1.Init.NominalTimeSeg1 = 15;
    hfdcan1.Init.NominalTimeSeg2 = 8;
    hfdcan1.Init.DataPrescaler = 1;
    hfdcan1.Init.DataSyncJumpWidth = 4;
    hfdcan1.Init.DataTimeSeg1 = 15;
    hfdcan1.Init.DataTimeSeg2 = 8;

    hfdcan1.Init.StdFiltersNbr = 2; 
    hfdcan1.Init.ExtFiltersNbr = 0;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) Error_Handler();
    SIMPLEFOC_DEBUG("CAN: FDCAN peripheral initialized.");

    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = CAN_ID_COMMAND_BASE + can_id;
    sFilterConfig.FilterID2 = CAN_ID_SCAN_BROADCAST;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) Error_Handler();

    sFilterConfig.FilterIndex = 1;
    sFilterConfig.FilterID1 = CAN_ID_MOTION_COMMAND_BASE + can_id;
    sFilterConfig.FilterID2 = CAN_ID_SYNC;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) Error_Handler();

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) Error_Handler();
    
    SIMPLEFOC_DEBUG("CAN: Starting FDCAN...");
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) Error_Handler();
    SIMPLEFOC_DEBUG("CAN: FDCAN Started.");
}

bool CAN_Poll(FDCAN_RxHeaderTypeDef* rxHeader, uint8_t* rxData) {
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, rxHeader, rxData) == HAL_OK) {
            return true;
        }
    }
    return false;
}

bool CAN_Send(uint16_t id, uint8_t* data, uint8_t dlc_bytes) {
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {
        return false;
    }

    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = bytesToDLC(dlc_bytes); // Use the helper function
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK) {
        return false;
    }

    return true;
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hfdcan->Instance == FDCAN1) {
        __HAL_RCC_FDCAN_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan) {
    if (hfdcan->Instance == FDCAN1) {
        __HAL_RCC_FDCAN_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
    }
}