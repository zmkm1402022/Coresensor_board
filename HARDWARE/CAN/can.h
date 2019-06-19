#ifdef CAN_GLOBALS 
#define CAN_EXT
#else
#define CAN_EXT extern 
#endif 

/// This will be changed when the support for CAN FD is added

//#define CANARD_CAN_FRAME_MAX_DATA_LEN      136U
#include  "stdint.h"
#include "canard.h"

CAN_EXT uint8_t CAN_rx_buf[150];
CAN_EXT uint8_t CAN_tx_buf[150];
CAN_EXT uint8_t UAVCAN_NODE_ID_SELF_ID;


void SYSTEM_RESET_CMD(uint16_t delay);

CAN_EXT uint8_t CAN_Rx_Succ;
CAN_EXT uint8_t CAN_ID;  //定义本机CAN_ID。 
CAN_EXT uint8_t AutoReturnDataFlag;

CAN_EXT void CAN_Configuration(void);                              //CAN配置函数
CAN_EXT void UavcanFun(void);
CAN_EXT void Uavcan_Broadcast(void);
CAN_EXT void Uavcan_Broadcast_Distance(void);
CAN_EXT void UavcanErr(void);
void CAN_NodeID_Update(void);
/******************************************************************************************/
