#include <can.h>
void CAN_Filter_Config_ListMode(void);
void CAN_Send(uint32_t id, uint8_t* data, uint8_t len);
uint8_t CAN_Receive(uint32_t *id, uint8_t *data, uint8_t *len);
