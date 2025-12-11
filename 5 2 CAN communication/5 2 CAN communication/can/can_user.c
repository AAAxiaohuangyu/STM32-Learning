#include <can_user.h>
#include <string.h>

void CAN_Filter_Config_Mode(void)
{
    HAL_CAN_Stop(&hcan);
    CAN_FilterTypeDef filter;
    
    /* 过滤器配置结构体初始化 */
    memset(&filter, 0, sizeof(filter));
    
    filter.FilterBank = 0;                         // 使用过滤器组0
    filter.FilterMode = CAN_FILTERMODE_IDLIST;     // 列表模式
    filter.FilterScale = CAN_FILTERSCALE_32BIT;    // 32位模式
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;    // 使用FIFO0接收
    filter.FilterActivation = ENABLE;              // 启用过滤器
    uint32_t target_id = 0x22;  // 目标ID
    filter.FilterIdHigh = ((target_id << 5) & 0xFFFF);  // 高16位
    filter.FilterIdLow = 0x0000;                       // 低16位


    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
    {
        Error_Handler();
    }


    memset(&filter, 0, sizeof(filter));

    filter.FilterBank = 1;                         // 使用过滤器组1
    filter.FilterMode = CAN_FILTERMODE_IDMASK;     // 掩码模式
    filter.FilterScale = CAN_FILTERSCALE_16BIT;    // 16位模式
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;    // 使用FIFO0接收
    filter.FilterActivation = ENABLE;              // 启用过滤器
    target_id = 0x10;  // 目标ID
    filter.FilterIdHigh = ((target_id << 5) & 0xFFFF);  // 高16位
    filter.FilterIdLow = ((target_id << 5) & 0xFFFF); // 低16位
    filter.FilterMaskIdHigh = ((0xfff0 <<5) & 0xFFFF);
    filter.FilterMaskIdLow = ((0xfff0 <<5) & 0xFFFF);
    
    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_CAN_Start(&hcan);
}

void CAN_Send(uint32_t id, uint8_t* data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t mailbox;
    
    /* 参数检查 */
    if (len > 8 || data == NULL || id > 0x7FF) {
        return;
    }
    /* 配置标准帧 */
    tx_header.StdId = id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;      // 标准帧
    tx_header.RTR = CAN_RTR_DATA;    // 数据帧
    tx_header.DLC = len;             // 数据长度
    tx_header.TransmitGlobalTime = DISABLE;
    
    /* 发送 */
    HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &mailbox);
}

uint8_t CAN_Receive(uint32_t *id, uint8_t *data, uint8_t *len)
{
    CAN_RxHeaderTypeDef rx_header;
    
    
    /* 读取数据 */
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rx_header, data) != HAL_OK)
    {
        return 0;  // 读取失败
    }
    
    *id = rx_header.StdId;
    *len = rx_header.DLC;
    return 1;  // 成功接收
}
