#include "can.h"
#include <string.h>

<<<<<<< HEAD
static FDCAN_HandleTypeDef hfdcan1;
static FDCAN_TxHeaderTypeDef TxHeader;

void CAN_Init(uint8_t can_id) {
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
=======
// Module-level variables
static FDCAN_HandleTypeDef hfdcan1;
static FDCAN_FilterTypeDef sFilterConfig;
static FDCAN_TxHeaderTypeDef TxHeader;

volatile uint8_t TxData[8];

// Helper function to convert DLC enum to an integer length
static uint8_t dlc_to_len(uint32_t dlc) {
    switch (dlc) {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
        default: return 8;
    }
}

void CAN_Init(uint8_t can_id) {
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
>>>>>>> 81eea26098ce949d4d77528ed42bd88f5d28f7a7
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = ENABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;

<<<<<<< HEAD
=======
    // FIX: Timing parameters updated for 12MHz Clock -> 500kbit/s Speed
>>>>>>> 81eea26098ce949d4d77528ed42bd88f5d28f7a7
    hfdcan1.Init.NominalPrescaler = 1;
    hfdcan1.Init.NominalSyncJumpWidth = 4;
    hfdcan1.Init.NominalTimeSeg1 = 15;
    hfdcan1.Init.NominalTimeSeg2 = 8;
    hfdcan1.Init.DataPrescaler = 1;
    hfdcan1.Init.DataSyncJumpWidth = 4;
    hfdcan1.Init.DataTimeSeg1 = 15;
    hfdcan1.Init.DataTimeSeg2 = 8;

    hfdcan1.Init.StdFiltersNbr = 1;
    hfdcan1.Init.ExtFiltersNbr = 0;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) Error_Handler();

<<<<<<< HEAD
    FDCAN_FilterTypeDef sFilterConfig;
=======
>>>>>>> 81eea26098ce949d4d77528ed42bd88f5d28f7a7
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = can_id;
    sFilterConfig.FilterID2 = 0x7FF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) Error_Handler();
<<<<<<< HEAD
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) Error_Handler();
=======

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) Error_Handler();

>>>>>>> 81eea26098ce949d4d77528ed42bd88f5d28f7a7
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) Error_Handler();
}

bool CAN_Poll(FDCAN_RxHeaderTypeDef* rxHeader, uint8_t* rxData) {
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, rxHeader, rxData) == HAL_OK) {
            return true;
        }
    }
    return false;
}

void CAN_Send(uint16_t id, uint8_t* data, uint32_t dlc) {
    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = dlc;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
<<<<<<< HEAD
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK) {
        // In this minimal version, we don't freeze on a send error.
    }
}

=======
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    
    uint8_t len = dlc_to_len(dlc);
    memcpy((void*)TxData, data, len);

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, (uint8_t*)TxData) != HAL_OK) {
        Error_Handler();
    }
}

uint8_t CAN_FindUniqueID(void) {
    FDCAN_TxHeaderTypeDef remoteTxHeader;
    remoteTxHeader.IdType = FDCAN_STANDARD_ID;
    remoteTxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
    remoteTxHeader.DataLength = FDCAN_DLC_BYTES_0;
    remoteTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    remoteTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    remoteTxHeader.FDFormat = FDCAN_FD_CAN;
    remoteTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    remoteTxHeader.MessageMarker = 0;

    for (uint8_t id = 1; id < 128; id++) {
        remoteTxHeader.Identifier = id;
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &remoteTxHeader, NULL) != HAL_OK) Error_Handler();
        delay(10); 
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0) {
            return id;
        }
        FDCAN_RxHeaderTypeDef dummyHeader;
        uint8_t dummyData[8];
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &dummyHeader, dummyData);
    }
    return 0;
}

>>>>>>> 81eea26098ce949d4d77528ed42bd88f5d28f7a7
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
<<<<<<< HEAD
}
=======
}
>>>>>>> 81eea26098ce949d4d77528ed42bd88f5d28f7a7
