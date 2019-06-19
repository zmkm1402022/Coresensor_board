#include <string.h>
#include <stdint.h>
#include "header.h"
#include "can.h"
#include "canard.h"
#include "stm32f10x.h"
#include "ds18b20.h"
#include "FreeRTOS.h"				
#include "task.h"
#include "semphr.h"	  
/*

*/
CanTxMsg cantxframe;

extern SemaphoreHandle_t BinarySemaphore;

const TDeviceInfo ThisVersion = {
									"SensorBoard_v1.0",
									1,  // FW VERSION
									0,
									1,  // HW Version
									0,
									0x2018010312345678
								};	

uint8_t AutoReturnDataFlag = 0;
uint8_t UAVCAN_NODE_ID_SELF_ID;
uint8_t CAN_rx_buf[150];
uint8_t CAN_tx_buf[150];
u16 CAN_RX_LEN;
uint8_t RX_FLAG = 0;

uint8_t IAP_BUFFER[40];
uint32_t flash_hex_baseaddr = 0;

uint8_t is_xdigit(uint8_t data);
uint8_t ascii_to_xdigit(uint8_t data);
uint8_t IAP_FRAM_CHECK(uint8_t Data[],uint8_t *const pResult);
uint8_t IAP_Handle(uint8_t data[]);
uint8_t  iap_flash_write(uint32_t baseaddr,uint8_t data[]);
uint32_t iap_flash_get_baseaddr(uint8_t data[]);
uint8_t iap_flash_set_jumpaddr(void);
//void FLASH_EraseAllNextZONE(uint32_t Page_Address,u16 size);
uint8_t FLASH_EraseAllNextZONE(uint32_t Page_Address,u16 size);
/*
 * Some useful constants defined by the UAVCAN specification.
 * Data type signature values can be easily obtained with the script show_data_type_info.py
 */
#define UAVCAN_NODE_ID_MASTER_ID                                    0x01   //������ַ
//#define UAVCAN_NODE_ID_SELF_ID                                      0x08   //������ַ

 
#define UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID                      1
#define UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE               0x0b2a812620a11d40
#define UAVCAN_NODE_ID_ALLOCATION_RANDOM_TIMEOUT_RANGE_USEC         400000UL
#define UAVCAN_NODE_ID_ALLOCATION_REQUEST_DELAY_OFFSET_USEC         600000UL

//�������----��������
#define UAVCAN_SERVICE_ID_PING                                  0xC9  //PING�ź�
#define UAVCAN_SERVICE_ID_INFO                                  0xCA  //��ȡ��Ϣ
#define UAVCAN_SERVICE_ID_RESET                                 0xCB  //RESET
#define UAVCAN_SERVICE_ID_START                                 0xCC  //START
#define UAVCAN_SERVICE_ID_STOP                                  0xCD  //STOP 

//�������----Powercore˽�з���
#define UAVCAN_SERVICE_ID_REALCLOCK                           	0xD8
#define UAVCAN_SERVICE_ID_READRTC																0xD9
#define UAVCAN_SERVICE_ID_US_ONOFF		                        	0xDA
#define UAVCAN_SERVICE_ID_IR_ONOFF		                        	0xDB


//�������----���÷���246-256
#define UAVCAN_SERVICE_FIRMWAWR_UPDATE							0xFE  //�̼�����
#define UAVCAN_SERVICE_GET_ZONE									0xFF  //��ȡ��ǰ�̼�����

//SIGNATURE
#define UAVCAN_SERVICE_DEFAULT_SIGNATURE										0x00
#define UAVCAN_SERVICE_PING_SIGNATURE                        0xD9DECB8B9DD64068
#define UAVCAN_SERVICE_NODE_INFO_SIGNATURE                   0x6AB3E1F001ECE006
#define UAVCAN_SERVICE_RESET_SIGNATURE                       0x7E087F0F029578EC
#define UAVCAN_SERVICE_START_SIGNATURE                       0xEC7F28DAC87C4379
#define UAVCAN_SERVICE_STOP_SIGNATURE                        0x644C8832B567A1DC
#define UAVCAN_SERVICE_ID_REALCLOCK_SIGNATURE                   0x644C8832B567A1DC
#define UAVCAN_SERVICE_ID_READRTC_SIGNATURE                  0x00
#define UAVCAN_SERVICE_ID_USONOFF_SIGNATURE                  0x00
#define UAVCAN_SERVICE_ID_IRONOFF_SIGNATURE										0x00
//#define UAVCAN_SERVICE_ID_SWITCH_24VOUT_SIGNATURE							0x00
//#define UAVCAN_SERVICE_ID_SWITCH_POWER_SIGNATURE								0x00
#define UAVCAN_SERVICE_ID_CURRENT_SIGNATURE									 0x00
#define UAVCAN_SERVICE_SET_PARAM_SIGNATURE                   0x00
#define UAVCAN_SERVICE_FIRMWAWR_SIGNATURE                    0x812EBFD0290C78A1
#define UAVCAN_SERVICE_GET_ZONE_SIGNATURE                    0xD1D00F451131DBE2
#define UAVCAN_SERVICE_ID_ERRSTATUS_SIGNATURE								 0x00

#define UAVCAN_SERVICE_ID_BROADCAST_IMU_SIGNATURE 							 0x15C93ADF28B04B9E
#define UAVCAN_SERVICE_ID_BROADCAST_DISTANCE_SIGNATURE 							 0x15C93ADF28B04B9E
#define UAVCAN_SERVICE_ID_BROADCAST_TIME_SIGNATURE 							 0x15C93ADF28B04B9E
//FLASH ADDRESS
#define BOOTLOADER_ADDRESS				(uint32_t)0x08000000	  //bootloader   32k
#define FUNCTION_A_ADDRESS				(uint32_t)0x08008000    //����A��  96K   
#define FUNCTION_B_ADDRESS				(uint32_t)0x08020000   //����B��  96K
#define APP_PARAM_ADDRESS				  (uint32_t)0x08038000    //�û������� 24K
#define BOOT_PARAM_ADDRESS				(uint32_t)0x0803E000    //BOOT������ 6K,������Ϊ�û�������
#define FUNCTION_JUMP_ADDRESS			(uint32_t)0x0803F800    //��ת��ַ�洢 2K
#define IAP_LENTH                     (uint16_t)0xC000     //IAP����������


//#define UAVCAN_NODE_STATUS_MESSAGE_SIZE                             7
//#define UAVCAN_NODE_STATUS_DATA_TYPE_ID                             341
//#define UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE                      0x0f0868d0c1a7c6f1

#define UAVCAN_NODE_HEALTH_OK                                       0
#define UAVCAN_NODE_HEALTH_WARNING                                  1
#define UAVCAN_NODE_HEALTH_ERROR                                    2
#define UAVCAN_NODE_HEALTH_CRITICAL                                 3

#define UAVCAN_NODE_MODE_OPERATIONAL                                0
#define UAVCAN_NODE_MODE_INITIALIZATION                             1

#define UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE                      ((3015 + 7) / 8)
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE                    0xee468a8121c46a9e
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_ID                           1

#define UNIQUE_ID_LENGTH_BYTES                                      16

//�㲥���͵���----��������5080-50BF

#define UAVCAN_TOPIC_ID_USONIC_TDATA											0x508A //�㲥����
#define UAVCAN_TOPIC_ID_INFRAR_TDATA											0x5087
#define UAVCAN_TOPIC_ID_TEMPERATURE_TDATA									0x508C
#define UAVCAN_TOPIC_ID_IMU_TDATA													0x508D
#define UAVCAN_TOPIC_ID_REALCLOCK_TDATA										0x508E 

#define UAVCAN_TOPIC_ID_US_ONOFF													0x5082
#define UAVCAN_TOPIC_ID_IR_ONOFF													0x5083
#define UAVCAN_TOPIC_ID_IMU_ONOFF													0x5084

#define UAVCAN_TOPIC_ID_REALCLOCK													0x5085
//#define UAVCAN_TOPIC_ID_LED_SET										0x5081 		//LED mode set;
//#define UAVCAN_TOPIC_ID_LOWMHZ_SET_CMD								0x5082 		//433MHZ mode set cmd;
//#define UAVCAN_TOPIC_ID_LOWMHZ_TRANSPARENT_TX						0x5083 		//433MHZ ͸������ ����; ----����CAN��--MCU���ڷ���--433Mģ��--���߷���
//#define UAVCAN_TOPIC_ID_LOWMHZ_TRANSPARENT_RX						0x5084 		//433MHZ ͸������ ����; ----���߽���--433Mģ��--MCU���ڽ���--����CAN����
//#define UAVCAN_TOPIC_ID_BLE_SET_CMD									0x5086 		//���� mode set cmd;
//#define UAVCAN_TOPIC_ID_BLE_TRANSPARENT_TX							0x5087 		//���� ͸������ ����; ----����CAN��--MCU���ڷ���--BLEģ��--��������
//#define UAVCAN_TOPIC_ID_BLE_TRANSPARENT_RX							0x5088 		//���� ͸������ ����; ----��������--BLEģ��--MCU���ڽ���--����CAN����


/*
 * Library instance.
 * In simple applications it makes sense to make it static, but it is not necessary.
 */
CanardInstance canard;                       ///< The library instance
uint8_t canard_memory_pool[1024];            ///< Arena for memory allocation, used by the library
uint8_t transfer_id;
/*
 * Variables used for dynamic node ID allocation.
 * RTFM at http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
 */
//static uint64_t send_next_node_id_allocation_request_at;    ///< When the next node ID allocation request should be sent
//static uint8_t node_id_allocation_unique_id_offset;         ///< Depends on the stage of the next request

/*
 * Node status variables
 */
//static uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
//static uint8_t node_mode   = UAVCAN_NODE_MODE_INITIALIZATION;
static CanardRxTransfer RxTransfer;

/**
 * This callback is invoked by the library when a new message or request or response is received.
 */
static void onTransferReceived(CanardInstance* ins,
                               CanardRxTransfer* transfer)
{
	uint8_t i=0;
	if(RX_FLAG == 1)
	{
		return;   //����û�д�����������������
	}
	if(transfer->payload_len <= 7)
	{
		memcpy(CAN_rx_buf,transfer->payload_head, transfer->payload_len); //������֡����
	}
	else if (transfer->payload_len <= 12)
	{                                                                     //���ƶ�֡����
		memcpy(CAN_rx_buf,transfer->payload_head, 6); //������֡ͷ����
		i += 6;    
		memcpy(&CAN_rx_buf[i],transfer->payload_tail, transfer->payload_len-6); //������֡ͷ����
	}
	else
	{   
		int16_t len = transfer->payload_len;
		memcpy(CAN_rx_buf,transfer->payload_head, 6); //������֡ͷ����
		i += 6;
		len -=5;
		CanardBufferBlock *pbuffer =transfer->payload_middle;
		while( pbuffer != NULL)
		{
			if (len>CANARD_BUFFER_BLOCK_DATA_SIZE)
			{
				memcpy(&CAN_rx_buf[i],pbuffer->data,CANARD_BUFFER_BLOCK_DATA_SIZE); //������֡ͷ����
			}
			else
			{
				memcpy(&CAN_rx_buf[i],pbuffer->data,len); //������֡ͷ����
			}
			pbuffer =pbuffer->next;
			len += CANARD_BUFFER_BLOCK_DATA_SIZE;
			i += CANARD_BUFFER_BLOCK_DATA_SIZE;
		}
		memcpy(&CAN_rx_buf[i],transfer->payload_tail, len);//������֡ͷ����  
	}
	CAN_RX_LEN = transfer->payload_len;		//���յ����ݳ���
	memcpy(&RxTransfer,transfer, sizeof(CanardRxTransfer)); //������֡ͷ���� 
//	canardReleaseRxTransferPayload(ins, transfer);
	RX_FLAG = 1;
}


static bool shouldAcceptTransfer(const CanardInstance* ins,
                                 uint64_t* out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
	   
	//(void)source_node_id;

//    if (source_node_id == UAVCAN_NODE_ID_MASTER_ID)
//    {
        /*
         * If we're in the process of allocation of dynamic node ID, accept only relevant transfers.
         */
        if (transfer_type == CanardTransferTypeBroadcast)
        {
					switch (data_type_id)
          {
						//*out_data_type_signature = 
					}
            *out_data_type_signature = UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE;
            return true;
        }
//    }
    if (transfer_type == CanardTransferTypeRequest)
    {
//            *out_data_type_signature = UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE;
//            return true;
		switch (data_type_id)
		{
			case UAVCAN_SERVICE_ID_PING:
			    *out_data_type_signature = UAVCAN_SERVICE_PING_SIGNATURE;	 /*����ͨѶ�Ƿ�����*/
			break;
			case UAVCAN_SERVICE_ID_INFO:
				*out_data_type_signature = UAVCAN_SERVICE_NODE_INFO_SIGNATURE;	/*�ڵ���Ϣ*/
			break;
			case UAVCAN_SERVICE_ID_RESET:
				*out_data_type_signature = UAVCAN_SERVICE_RESET_SIGNATURE; /*��������*/
			break;
			case UAVCAN_SERVICE_ID_START:	
			  *out_data_type_signature = UAVCAN_SERVICE_START_SIGNATURE;
			break;
			case UAVCAN_SERVICE_ID_STOP:	
			  *out_data_type_signature = UAVCAN_SERVICE_STOP_SIGNATURE;	
			break;
			case UAVCAN_SERVICE_ID_REALCLOCK:
			  *out_data_type_signature = UAVCAN_SERVICE_ID_REALCLOCK_SIGNATURE;
			break;
			case UAVCAN_SERVICE_ID_READRTC:
				*out_data_type_signature = UAVCAN_SERVICE_ID_READRTC_SIGNATURE;
			break;
//			case UAVCAN_SERVICE_ID_READ_PARAM:
//				*out_data_type_signature = UAVCAN_SERVICE_READ_PARAM_SIGNATURE;
//			break;
//			case UAVCAN_SERVICE_ID_SET_RARAM:
//				*out_data_type_signature = UAVCAN_SERVICE_SET_PARAM_SIGNATURE;
//			break;
			case UAVCAN_SERVICE_FIRMWAWR_UPDATE:
			   *out_data_type_signature = UAVCAN_SERVICE_FIRMWAWR_SIGNATURE;				
			break;
			case UAVCAN_SERVICE_GET_ZONE:
			   *out_data_type_signature = UAVCAN_SERVICE_GET_ZONE_SIGNATURE;				
			break;
			default:
		       *out_data_type_signature = 0;	
		}
		return true;
    }
	  return false;
}


void CAN_NodeID_Update(void)
{
	gSensor.CanID.m_ID = gSensor.CanID.m_ID*81/100; //unit mV
	if(gSensor.CanID.m_ID < 800){
		UAVCAN_NODE_ID_SELF_ID = 0;
		printf("Error: fail to set the CAN ID.\r\n");
	}
	else if (gSensor.CanID.m_ID < 1100)
		UAVCAN_NODE_ID_SELF_ID = 0x24;
	else if (gSensor.CanID.m_ID < 1600)
		UAVCAN_NODE_ID_SELF_ID = 0x25;	
	else if (gSensor.CanID.m_ID < 2000)
		UAVCAN_NODE_ID_SELF_ID = 0x26;
	else if (gSensor.CanID.m_ID < 3200)
		UAVCAN_NODE_ID_SELF_ID = 0x27;	
	else
		UAVCAN_NODE_ID_SELF_ID = 0;
}

/********************************************************************************
* ��    �ƣ�
* ��    �ܣ����������
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
*********************************************************************************/
void CAN_Configuration(void)                              //CAN���ú���
{
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    u32 mask =0;
	  u32 ext_id = 0;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);	   //ʹ��CAN1����ʱ��
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	GPIO_PinRemapConfig(GPIO_Remap_CAN_PortA,ENABLE);

    /***********CAN IO��ʼ��**************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;              //CAN rx: PINA 11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;           //��������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;              //CAN TX : PINA12
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         //�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);	//ʹ��CAN1����ʱ��
                             
    CAN_InitStructure.CAN_TTCM = DISABLE;    //��ֹʱ�䴥��ͨ��ģʽ
    //������CAN_MCR�Ĵ�����INRQλ������1�����0��һ��Ӳ����⵽128��11λ������    //����λ�����˳�����״̬��
    CAN_InitStructure.CAN_ABOM = DISABLE;    
    //˯��ģʽͨ�����CAN_MCR�Ĵ�����SLEEPλ������������
    CAN_InitStructure.CAN_AWUM = DISABLE;   
    CAN_InitStructure.CAN_NART = ENABLE;     //DISABLE;CAN����ֻ������1�Σ����ܷ��͵Ľ����Σ��ɹ����������ٲö�ʧ��
    CAN_InitStructure.CAN_RFLM = DISABLE;    //�ڽ������ʱFIFOδ��������������FIFO�ı���δ����������һ���յ��ı��ĻḲ��ԭ�еı���
    CAN_InitStructure.CAN_TXFP = ENABLE;    //����FIFO���ȼ��ɱ��ĵı�ʶ�������� CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //CANӲ������������ģʽ  CAN_Mode_LoopBack
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;  //����ͬ����Ծ����1��ʱ�䵥λ
    CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;  //ʱ���1Ϊ8��ʱ�䵥λ
    CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;  //ʱ���2Ϊ7��ʱ�䵥λ

    CAN_InitStructure.CAN_Prescaler = 6;   

	/*
	f = 24M/4/(6+5+1)=500K
	*/
    CAN_Init(CAN1,&CAN_InitStructure);
    /**********************************************����֡�˲�******************************/                             
/*    CAN_FilterInitStructure.CAN_FilterNumber=0;                 
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;  
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;    	         
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;  
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;   
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;    
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;             
    CAN_FilterInit(&CAN_FilterInitStructure); */
    ext_id = (UAVCAN_NODE_ID_SELF_ID<<8) + 0x80;   //ֻ���յ�ַΪ������ַ�ķ���֡
		mask =   0x00007F80;                           
    
    CAN_FilterInitStructure.CAN_FilterNumber=0;                 
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	  CAN_FilterInitStructure.CAN_FilterIdHigh=((ext_id<<3) >>16) &0xffff;  
    CAN_FilterInitStructure.CAN_FilterIdLow=(u16)(ext_id<<3) | CAN_ID_EXT;    	         
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=((mask<<3) >>16)&0xffff;    
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=(u16)(mask<<3) | CAN_ID_EXT;   
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;    
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;             
    CAN_FilterInit(&CAN_FilterInitStructure); 
		
	 /**********************************�㲥֡�˲�*ϵͳʹ��*****************************/
    /*ext_id = 0x00800000;   //ͨ��0x8000-0xffff
		mask =   0x00800080;        	
	  CAN_FilterInitStructure.CAN_FilterNumber=1;             
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	  CAN_FilterInitStructure.CAN_FilterIdHigh=((ext_id<<3) >>16) &0xffff;  
    CAN_FilterInitStructure.CAN_FilterIdLow=(u16)(ext_id<<3) | CAN_ID_EXT;   	          
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=((mask<<3) >>16)&0xffff;  
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=(u16)(mask<<3) | CAN_ID_EXT;   
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;    
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;             
    CAN_FilterInit(&CAN_FilterInitStructure);*/
		
	 /**********************************�㲥֡�˲�*ϵͳʹ��*****************************/
    /*ext_id = 0x00000000;   //ͨ��0x0000-0x3fff
		mask =   0x00C00080;        	
	  CAN_FilterInitStructure.CAN_FilterNumber=2;             
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	  CAN_FilterInitStructure.CAN_FilterIdHigh=((ext_id<<3) >>16) &0xffff;   
    CAN_FilterInitStructure.CAN_FilterIdLow=(u16)(ext_id<<3) | CAN_ID_EXT;    	          
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=((mask<<3) >>16)&0xffff;  
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=(u16)(mask<<3) | CAN_ID_EXT;    
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;    
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;             
    CAN_FilterInit(&CAN_FilterInitStructure);	*/	
		

	 /**********************************�㲥֡�˲�*ϵͳʹ��*****************************/
    /*ext_id = 0x00400000;;   //ͨ��0x4000-0x4fff
		mask =   0x00f00080;        	
	  CAN_FilterInitStructure.CAN_FilterNumber=3;             
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	  CAN_FilterInitStructure.CAN_FilterIdHigh=((ext_id<<3) >>16) &0xffff; 
    CAN_FilterInitStructure.CAN_FilterIdLow=(u16)(ext_id<<3) | CAN_ID_EXT; 	          
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=((mask<<3) >>16)&0xffff; 
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=(u16)(mask<<3) | CAN_ID_EXT;    
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;    
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;             
    CAN_FilterInit(&CAN_FilterInitStructure);	*/	

	 /**********************************�㲥֡�˲�*ϵͳʹ��*****************************/
    /*ext_id = 0x00700000;;   //ͨ��0x6000-0x7fff
		mask =   0x00E00080;        	
	  CAN_FilterInitStructure.CAN_FilterNumber=4;             
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	  CAN_FilterInitStructure.CAN_FilterIdHigh=((ext_id<<3) >>16) &0xffff;  
    CAN_FilterInitStructure.CAN_FilterIdLow=(u16)(ext_id<<3) | CAN_ID_EXT;   	          
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=((mask<<3) >>16)&0xffff;  
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=(u16)(mask<<3) | CAN_ID_EXT;    
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;    
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;             
    CAN_FilterInit(&CAN_FilterInitStructure);*/		

	 /**********************************�㲥֡�˲�*ϵͳʹ��*****************************/
 /*   ext_id = 0x00580000;   //ͨ��0x5800-0x5FFF
		mask =   0x00F80080;        	
	  CAN_FilterInitStructure.CAN_FilterNumber=5;             
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	  CAN_FilterInitStructure.CAN_FilterIdHigh=((ext_id<<3) >>16) &0xffff;  
    CAN_FilterInitStructure.CAN_FilterIdLow=(u16)(ext_id<<3) | CAN_ID_EXT;    	          
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=((mask<<3) >>16)&0xffff;   
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=(u16)(mask<<3) | CAN_ID_EXT;    
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;    
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;             
    CAN_FilterInit(&CAN_FilterInitStructure);	*/

    /**********************************�㲥֡�˲�*ϵͳʹ��*****************************/
    /*ext_id = 0x00540000;;   //ͨ��0x5400-0x57FF
		mask =   0x00FC0080;        	
	  CAN_FilterInitStructure.CAN_FilterNumber=6;             
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	  CAN_FilterInitStructure.CAN_FilterIdHigh=((ext_id<<3) >>16) &0xffff;   
    CAN_FilterInitStructure.CAN_FilterIdLow=(u16)(ext_id<<3) | CAN_ID_EXT;   	          
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=((mask<<3) >>16)&0xffff;  
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=(u16)(mask<<3) | CAN_ID_EXT;   
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;    
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;             
    CAN_FilterInit(&CAN_FilterInitStructure);	*/	

    /**********************************�㲥֡�˲�*ϵͳʹ��*****************************/
    /*ext_id = 0x00520000;;   //ͨ��0x5200-0x53FF
		mask =   0x00FE0080;        	
	  CAN_FilterInitStructure.CAN_FilterNumber=7;             
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	  CAN_FilterInitStructure.CAN_FilterIdHigh=((ext_id<<3) >>16) &0xffff;  
    CAN_FilterInitStructure.CAN_FilterIdLow=(u16)(ext_id<<3) | CAN_ID_EXT;   	          
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=((mask<<3) >>16)&0xffff;  
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=(u16)(mask<<3) | CAN_ID_EXT;   
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;    
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;             
    CAN_FilterInit(&CAN_FilterInitStructure);	*/


    /**********************************�㲥֡�˲�*��������*****************************/
    ext_id = 0x00508000;;   //ͨ��0x5080-0x50BF
		mask =   0x00ffc080;        	
	  CAN_FilterInitStructure.CAN_FilterNumber=8;             
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	  CAN_FilterInitStructure.CAN_FilterIdHigh=((ext_id<<3) >>16) &0xffff;  
    CAN_FilterInitStructure.CAN_FilterIdLow=(u16)(ext_id<<3) | CAN_ID_EXT;   	          
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=((mask<<3) >>16)&0xffff;  
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=(u16)(mask<<3) | CAN_ID_EXT;     
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;    
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;             
    CAN_FilterInit(&CAN_FilterInitStructure);	


//    
//	    /**************************************�㲥֡����㲥******************************/
//    ext_id = 0x00503F6F;   //ͨ��0x500040-0x503F7F
//		mask =   0x00ffc0c0;        	
//	  CAN_FilterInitStructure.CAN_FilterNumber=8;             
//    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
//	  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
//	  CAN_FilterInitStructure.CAN_FilterIdHigh=((ext_id<<3) >>16) &0xffff;  
//    CAN_FilterInitStructure.CAN_FilterIdLow=(u16)(ext_id<<3) | CAN_ID_EXT;    	          
//    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=((mask<<3) >>16)&0xffff;  
//    CAN_FilterInitStructure.CAN_FilterMaskIdLow=(u16)(mask<<3) | CAN_ID_EXT;    
//    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;    
//    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;             
//    CAN_FilterInit(&CAN_FilterInitStructure);	

//		
//	   /**************************************�㲥֡����㲥******************************/
/*    ext_id = 0x00500210;   //ͨ��0x5000-0-0x503F0F
		mask =   0x00FFFFFF;        	
	  CAN_FilterInitStructure.CAN_FilterNumber=9;             
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	  CAN_FilterInitStructure.CAN_FilterIdHigh=((ext_id<<3) >>16) &0xffff;   
    CAN_FilterInitStructure.CAN_FilterIdLow=(u16)(ext_id<<3) | CAN_ID_EXT;   	          
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=((mask<<3) >>16)&0xffff; 
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=(u16)(mask<<3) | 0xffff;    
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;    
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;             
    CAN_FilterInit(&CAN_FilterInitStructure);	  */
//		
//		
//			
//	   /**************************************�㲥֡����㲥******************************/
//    ext_id = 0x00503F2F;   //ͨ��0x5000-0-0x503F0F
//		mask =   0x00ffc0E0;
//	  CAN_FilterInitStructure.CAN_FilterNumber=10;             
//    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
//	  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
//	  CAN_FilterInitStructure.CAN_FilterIdHigh=((ext_id<<3) >>16) &0xffff;   
//    CAN_FilterInitStructure.CAN_FilterIdLow=(u16)(ext_id<<3) | CAN_ID_EXT;    	          
//    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=((mask<<3) >>16)&0xffff;  
//    CAN_FilterInitStructure.CAN_FilterMaskIdLow=(u16)(mask<<3) | CAN_ID_EXT;    
//    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;    
//    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;             
//    CAN_FilterInit(&CAN_FilterInitStructure);		
			
    CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);    //ʹ��ָ����CAN�ж�
		NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;			//��ռʽ���ȼ�(0~1)
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				//��Ӧ���ȼ�(0~7)
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
		
		canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool), onTransferReceived, shouldAcceptTransfer, NULL);
    canardSetLocalNodeID(&canard, UAVCAN_NODE_ID_SELF_ID);
}

/****************************************************************************
* ��    �ƣ�Uavcan_Broadcast
* ��    �ܣ�CAN���߹㲥���Զ��ϴ���
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
/*
1. �ϱ���������������������
2. �ϱ����������ഫ����������
3. �ϱ��¶ȵ�����
*/
void Uavcan_Broadcast_Distance(void)
{
	if (gSensor.SonarEnable == 1)
	{
		memset(CAN_tx_buf, 0, 8);
		CAN_tx_buf[0] = UlsResultVerification(gSensor.Sonar01.m_Distance);
		CAN_tx_buf[1] = UlsResultVerification(gSensor.Sonar02.m_Distance);
		CAN_tx_buf[2] = UlsResultVerification(gSensor.Sonar03.m_Distance);
		CAN_tx_buf[3] = UlsResultVerification(gSensor.Sonar04.m_Distance);
		CAN_tx_buf[4] = UlsResultVerification(gSensor.Sonar05.m_Distance);
		CAN_tx_buf[5] = UlsResultVerification(gSensor.Sonar06.m_Distance);
		CAN_tx_buf[6] = 0xFF;
		CAN_tx_buf[7] = 0xFF;	
		canardBroadcast(&canard, UAVCAN_SERVICE_ID_BROADCAST_DISTANCE_SIGNATURE, UAVCAN_TOPIC_ID_USONIC_TDATA, &transfer_id,10,CAN_tx_buf, 8);
		
		
	}
	else{
		gSensor.Sonar06.m_Distance =0;
		gSensor.Sonar05.m_Distance =0;
		gSensor.Sonar04.m_Distance =0;
		gSensor.Sonar03.m_Distance =0;
		gSensor.Sonar02.m_Distance =0;
		gSensor.Sonar01.m_Distance =0;
//		canardBroadcast(&canard, UAVCAN_SERVICE_ID_BROADCAST_DISTANCE_SIGNATURE, UAVCAN_TOPIC_ID_USONIC_TDATA, &transfer_id,10,CAN_tx_buf, 8);
	}
	
	
	if(gSensor.IREnable == 1)
	{
		memset(CAN_tx_buf, 0, 8);
		CAN_tx_buf[0] = Get_IR_DIS_Value(gSensor.InfraredSensor.Infra01.m_Distance);
		CAN_tx_buf[1] = Get_IR_DIS_Value(gSensor.InfraredSensor.Infra02.m_Distance);
		canardBroadcast(&canard, UAVCAN_SERVICE_ID_BROADCAST_DISTANCE_SIGNATURE, UAVCAN_TOPIC_ID_INFRAR_TDATA, &transfer_id,10,CAN_tx_buf, 8);
	}

	if(1)
	{
		CAN_tx_buf[0] = gSensor.Temperaturer.m_Temperaturer & 0xFF;
		CAN_tx_buf[1] = (gSensor.Temperaturer.m_Temperaturer>>8) & 0xFF;	
		canardBroadcast(&canard, UAVCAN_SERVICE_ID_BROADCAST_DISTANCE_SIGNATURE, UAVCAN_TOPIC_ID_TEMPERATURE_TDATA, &transfer_id,10,CAN_tx_buf, 2);
	
	}

	for(const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;)  //���ݷ���
  {
		cantxframe.ExtId = txf->id;
		cantxframe.DLC = txf->data_len;
		cantxframe.IDE = CAN_ID_EXT;         // �趨��Ϣ��ʶ��������
		cantxframe.RTR = CAN_RTR_DATA;       // �趨��������Ϣ��֡����
		memcpy(cantxframe.Data,txf->data, txf->data_len); //��������
		const uint8_t TransmitMailbox = CAN_Transmit(CAN1,&cantxframe);

		if(TransmitMailbox != CAN_TxStatus_NoMailBox)   
	  {
			canardPopTxQueue(&canard);        
		}
	}

}

/*
1. �ϱ����������
2. �ϱ�timestamp������
*/
void Uavcan_Broadcast(void)
{
	CAN_tx_buf[0] = gSensor.imuData.gyro[0] & 0xFF;
	CAN_tx_buf[1] = (gSensor.imuData.gyro[0]>>8) & 0xFF;	
	
	CAN_tx_buf[2] = gSensor.imuData.gyro[1] & 0xFF;
	CAN_tx_buf[3] = (gSensor.imuData.gyro[1]>>8) & 0xFF;	
	
	CAN_tx_buf[4] = gSensor.imuData.gyro[2] & 0xFF;
	CAN_tx_buf[5] = (gSensor.imuData.gyro[2]>>8) & 0xFF;	
	
	CAN_tx_buf[6] = gSensor.imuData.accel_short[0] & 0xFF;
	CAN_tx_buf[7] = (gSensor.imuData.accel_short[0]>>8) & 0xFF;	
	
	CAN_tx_buf[8] = gSensor.imuData.accel_short[1] & 0xFF;
	CAN_tx_buf[9] = (gSensor.imuData.accel_short[1]>>8) & 0xFF;
	
	CAN_tx_buf[10] = gSensor.imuData.accel_short[2] & 0xFF;
	CAN_tx_buf[11] = (gSensor.imuData.accel_short[2]>>8) & 0xFF;

	CAN_tx_buf[12] = gSensor.imuData.mag[0] & 0xFF;
	CAN_tx_buf[13] = (gSensor.imuData.mag[0]>>8) & 0xFF;
	
	CAN_tx_buf[14] = gSensor.imuData.mag[1] & 0xFF;
	CAN_tx_buf[15] = (gSensor.imuData.mag[1]>>8) & 0xFF;

	CAN_tx_buf[16] = gSensor.imuData.mag[2] & 0xFF;
	CAN_tx_buf[17] = (gSensor.imuData.mag[2]>>8) & 0xFF;

	canardBroadcast(&canard, UAVCAN_SERVICE_ID_BROADCAST_IMU_SIGNATURE, UAVCAN_TOPIC_ID_IMU_TDATA, &transfer_id,1,CAN_tx_buf, 18);
	for(const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;)  //���ݷ���
  {
		cantxframe.ExtId = txf->id;
		cantxframe.DLC = txf->data_len;
		cantxframe.IDE = CAN_ID_EXT;         // �趨��Ϣ��ʶ��������
		cantxframe.RTR = CAN_RTR_DATA;       // �趨��������Ϣ��֡����
		memcpy(cantxframe.Data,txf->data, txf->data_len); //��������
		const uint8_t TransmitMailbox = CAN_Transmit(CAN1,&cantxframe);

		if(TransmitMailbox != CAN_TxStatus_NoMailBox)   
	  {
			canardPopTxQueue(&canard);        
		}
	}
}

/****************************************************************************
* ��    �ƣ�UavcanFun 
* ��    �ܣ�CAN���������շ�
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/
void UavcanFun(void)
{
	 
	uint8_t error = 0;
	if( RX_FLAG == 1)
	{
		if(RxTransfer.transfer_type == CanardTransferTypeBroadcast)
		{
			if(RxTransfer.data_type_id == UAVCAN_TOPIC_ID_US_ONOFF)  // ������࿪��
			{
				gSensor.SonarEnable = CAN_rx_buf[0];
			}
			
			if(RxTransfer.data_type_id == UAVCAN_TOPIC_ID_IR_ONOFF)
			{
				gSensor.IREnable = CAN_rx_buf[0];
			}
			
			if(RxTransfer.data_type_id == UAVCAN_TOPIC_ID_IMU_ONOFF)
			{
				gSensor.IMUEnable = CAN_rx_buf[0];
			}		
			
			if(RxTransfer.data_type_id == UAVCAN_TOPIC_ID_REALCLOCK_TDATA)
			{
				Readburst(CAN_tx_buf);
				if(DEBUG == 1)
					printf("DS1302 is set at Time:20%d��%d��%d��%dʱ%d��%d��\r\n",	CAN_tx_buf[6],
																												CAN_tx_buf[4],
																												CAN_tx_buf[3],
																												CAN_tx_buf[2],
																												CAN_tx_buf[1],
																												CAN_tx_buf[0]);
				canardBroadcast(&canard, UAVCAN_SERVICE_ID_READRTC_SIGNATURE, UAVCAN_TOPIC_ID_REALCLOCK_TDATA, &transfer_id,1,CAN_tx_buf, 7);				
			}	
			
			if(RxTransfer.data_type_id == UAVCAN_TOPIC_ID_REALCLOCK)
			{
					if(CAN_rx_buf[0]>0x99 || CAN_rx_buf[0]<0x19)  //year
					{

						error = 1;
					}
					else if(CAN_rx_buf[1] >7)   //day
						error = 1;
					else if(CAN_rx_buf[2] > 0x12)  //month
						error = 1;
					else if(CAN_rx_buf[3] > 0x31)   //date 
						error = 1;
					else if (CAN_rx_buf[4] >0x24)   //hour
						error = 1;
					else if(CAN_rx_buf[5]> 0x59 || CAN_rx_buf[6]> 0x59) //minute __ seconds
						error =1;
					if(error ==1)
					{
						CAN_tx_buf[0] = 0x02;   //������������
						canardBroadcast(&canard, UAVCAN_SERVICE_ID_REALCLOCK_SIGNATURE, UAVCAN_TOPIC_ID_REALCLOCK, &transfer_id,1,CAN_tx_buf, 1);
					}
					else
					{
						DS1302_DateSetup(CAN_rx_buf);
						Readburst(CAN_tx_buf);
						if(DEBUG == 1)
							printf("DS1302 is set at Time:20%d��%d��%d��%dʱ%d��%d��\r\n",	CAN_tx_buf[6],
																														CAN_tx_buf[4],
																														CAN_tx_buf[3],
																														CAN_tx_buf[2],
																														CAN_tx_buf[1],
																														CAN_tx_buf[0]);
						if (CAN_tx_buf[6] == CAN_rx_buf[0] && CAN_tx_buf[1] == CAN_rx_buf[5])
						{
							CAN_tx_buf[0] = 0x00;   //�������óɹ�
							canardBroadcast(&canard, UAVCAN_SERVICE_ID_REALCLOCK_SIGNATURE, UAVCAN_TOPIC_ID_REALCLOCK, &transfer_id,1,CAN_tx_buf, 1);
						}			
						else
						{
							CAN_tx_buf[0] = 0x01;   //��������ʧ��
							canardBroadcast(&canard, UAVCAN_SERVICE_ID_REALCLOCK_SIGNATURE, UAVCAN_TOPIC_ID_REALCLOCK, &transfer_id,1,CAN_tx_buf, 1);
						}								
					}				
			}				
		}

		else if (RxTransfer.transfer_type == CanardTransferTypeRequest)
		{				   
			switch(RxTransfer.data_type_id)
			{
				case UAVCAN_SERVICE_ID_PING:					//PING		
					CAN_tx_buf[0] = 1;
					CAN_tx_buf[1] = UAVCAN_NODE_ID_SELF_ID;
					canardRequestOrRespond(&canard,
					RxTransfer.source_node_id,
					UAVCAN_SERVICE_PING_SIGNATURE,
					UAVCAN_SERVICE_ID_PING,
					&RxTransfer.transfer_id,
					RxTransfer.priority,
					CanardResponse,
					CAN_tx_buf,
					2);
				break;
				case UAVCAN_SERVICE_ID_INFO:					//READ Info��ȡ��Ϣ
					canardRequestOrRespond(&canard,
					RxTransfer.source_node_id,
					UAVCAN_SERVICE_NODE_INFO_SIGNATURE,
					UAVCAN_SERVICE_ID_INFO,
					&RxTransfer.transfer_id,
					RxTransfer.priority,
					CanardResponse,
					&ThisVersion,
					sizeof(ThisVersion));
					break;
				case UAVCAN_SERVICE_ID_RESET:					//RESET ��λ
					
					gSensor.RESETEnable = 1;
					break;				
				case UAVCAN_SERVICE_ID_START:					//START ������������

					break;				
				case UAVCAN_SERVICE_ID_STOP:					//STOP ֹͣ��������
					CAN_tx_buf[0] = 1;
					AutoReturnDataFlag &= ~(1<<2);
					canardRequestOrRespond(&canard,
					RxTransfer.source_node_id,
					UAVCAN_SERVICE_STOP_SIGNATURE,
					UAVCAN_SERVICE_ID_STOP,
					&RxTransfer.transfer_id,
					RxTransfer.priority,
					CanardResponse,
					CAN_tx_buf,
					1);
					break;
				
				case UAVCAN_SERVICE_FIRMWAWR_UPDATE:				
					if(IAP_FRAM_CHECK(CAN_rx_buf, IAP_BUFFER) == 0x01)
					{
						CAN_tx_buf[0] = IAP_Handle(IAP_BUFFER);  //���ݴ���	
					}
					else
					{
						CAN_tx_buf[0] = 0x01;  //���ݽ��մ���
					}
					canardRequestOrRespond(&canard,
																	RxTransfer.source_node_id,
																	UAVCAN_SERVICE_FIRMWAWR_SIGNATURE,
																	UAVCAN_SERVICE_FIRMWAWR_UPDATE,
																	&RxTransfer.transfer_id,
																	RxTransfer.priority,
																	CanardResponse,
																	CAN_tx_buf,
																	1);											
					break;	
					
				case UAVCAN_SERVICE_GET_ZONE:			
					if( (uint32_t)&ThisVersion >= FUNCTION_B_ADDRESS)
					{
						CAN_tx_buf[0] = 0x02;  //��ǰʹ��������ΪA��
					}
					else if((uint32_t)&ThisVersion >= FUNCTION_A_ADDRESS)
					{
						CAN_tx_buf[0] = 0x01;  //��ǰʹ��������ΪB��
					}
					else
					{
						CAN_tx_buf[0] = 0x00;  //��ǰʹ�����ݲ�����
					}
					canardRequestOrRespond(&canard,
																	RxTransfer.source_node_id,
																	UAVCAN_SERVICE_GET_ZONE_SIGNATURE,
																	UAVCAN_SERVICE_GET_ZONE,
																	&RxTransfer.transfer_id,
																	RxTransfer.priority,
																	CanardResponse,
																	CAN_tx_buf,
																	1);											
					break;		
				

			}
		}
		else if (RxTransfer.transfer_type == CanardTransferTypeResponse)
		{
		}
		RX_FLAG = 0;
	}
		
	for(const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;)  //���ݷ���
  {
		cantxframe.ExtId = txf->id;
		cantxframe.DLC = txf->data_len;
		cantxframe.IDE = CAN_ID_EXT;         // �趨��Ϣ��ʶ��������
		cantxframe.RTR = CAN_RTR_DATA;       // �趨��������Ϣ��֡����
		memcpy(cantxframe.Data,txf->data, txf->data_len); //��������
		const uint8_t TransmitMailbox = CAN_Transmit(CAN1,&cantxframe);

		if(TransmitMailbox != CAN_TxStatus_NoMailBox)   
	  {
			canardPopTxQueue(&canard);        
		}
	}
}





uint8_t IAP_FRAM_CHECK(uint8_t Data[], uint8_t * const pResult)
{
	uint8_t i;
	uint8_t *pdata;
	uint8_t checksum=0;
	uint8_t len;
	pdata = pResult;
	
	if (Data[0] != 0x3A)
	{
		return 0; 
	}
	i=1;
	while (Data[i] != 0x0D)
	{
		 if(!is_xdigit(Data[i]))
        {
            return 0;
        }
		if(i%2)
		{
            *pdata =  ascii_to_xdigit(Data[i])<<4;
		}
		else
		{
			*pdata +=  ascii_to_xdigit(Data[i]); 
			 pdata++;
		}
		i++;
		if(i > 2*(*pResult+6))
		{
		    return 0;
		}
		
	}
    pdata = pResult;
	len = *pdata + 5;
    for (i=0; i < len - 1; i++)
    {
       checksum += *pdata;
       pdata++; 
    }
	checksum = ~checksum;
	checksum += 1;
    if (checksum == *pdata)
    {
        return 1;
    }  
    return 0;	
}


uint8_t is_xdigit(uint8_t data)
{
    if((data >= 0x30) && (data <= 0x39))
    {
        return 1;
    }
    
    if((data >= 0x41) && (data <= 0x46))
    {
       return 1;  
    } 
    return 0;  
    
}

uint8_t ascii_to_xdigit(uint8_t data)
{
    if((data >= 0x30) && (data <= 0x39))
    {
        return data - 0x30;
    }
    
    if((data >= 0x41) && (data <= 0x46))
    {
       return data - 0x37;  
    }
    return 0xff;   
}

uint8_t IAP_Handle(uint8_t data[])
{
	uint8_t err;
	switch(data[3])
	{
		case 0x00:
		    err = iap_flash_write(flash_hex_baseaddr,data);                     
		    break;
		case 0x01:         //�ļ���д����
			err = iap_flash_set_jumpaddr();
			break;
		case 0x02:         //���ƫ�Ƶ�ַ
			err = 0x00;
			break;
		case 0x03:
			err = 0x00;
			break;
		case 0x04:
			flash_hex_baseaddr = iap_flash_get_baseaddr(data);
		    if( flash_hex_baseaddr == 0)
			{
			    err = 0x02;
			}
			else
			{
				err = 0x00;
			}
			break;
		case 0x05:
			err = 0x00;
			break;
		default:
			err = 0x00;
	}
	return err;
}
/*******************************************************************************
*****************************************************************************/
uint8_t iap_flash_write(uint32_t baseaddr,uint8_t data[])
{
   uint32_t addr;
   uint8_t len;
   uint8_t *pdata;
   uint8_t err;
	uint8_t ErasePageStatus;
	uint8_t ErasePageCnt;
   addr = baseaddr + (data[1]<<8) + data[2];  //����д���ַ
   len = data[0];
   pdata = &data[4];
   if (baseaddr != 0x08000000 && baseaddr != 0x08010000 && baseaddr != 0x08020000 &&baseaddr != 0x08030000)
   {
	
		
			return 0x03;
		
   }
	 

		
   if ((addr%0x4000) == 0)
   {
	   u8 cnt_page;
		 u32 addrs;
		 addrs = addr;
		 FLASH_Unlock();
	   FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	   for (cnt_page=0; cnt_page<16;cnt_page++){
		 ErasePageStatus = FLASH_ErasePage(addrs);
	   if(ErasePageStatus != FLASH_COMPLETE)
	   {
		   ErasePageCnt++;
		   if(ErasePageCnt >= 2)
		   {
			   err = 0x03;
			   return err;
		   }
		   else
		   {
				FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
				ErasePageStatus = FLASH_ErasePage(addrs);
		   }
	   }
		 addrs += 0x400;
		 
	 if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_9)==TRUE)
		GPIO_ResetBits(GPIOB,GPIO_Pin_9);
	 else
		GPIO_SetBits(GPIOB,GPIO_Pin_9);
	 }
	   FLASH_Lock();
   }
   
   FLASH_Unlock();
   while(len)
   {
		 
		if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_9)==TRUE)
			GPIO_ResetBits(GPIOB,GPIO_Pin_9);
		else
			GPIO_SetBits(GPIOB,GPIO_Pin_9);
		
		 if (len/4)
        {
			err = FLASH_ProgramWord(addr, *((uint32_t*)pdata));
            addr += 4;
            len -= 4; 
            pdata +=4;
        }
        else if (len/2)
        {
			err = FLASH_ProgramHalfWord(addr, *((uint16_t*)pdata));
            addr += 2;
            len -=2;
            pdata += 2;
        }
        else
        {
			err = FLASH_ProgramOptionByteData(addr, *pdata);
            addr += 1;
            len -= 1;
            pdata += 1;
        }
		if (err != FLASH_COMPLETE)
		{
			break;
		}		 
   }
   FLASH_Lock();
   if(err != FLASH_COMPLETE)
   {
	   return 0x03;
   }
   return 0x00;
}

uint32_t iap_flash_get_baseaddr(uint8_t data[])
{
	uint32_t addr;
	addr = (data[4]<<8) + data[5];
	if ((addr == 0x0800) || (addr == 0x0801)|| (addr == 0x0802)|| (addr == 0x0803))
	{
		return addr<<16;
	}
	return 0;
	
}

uint8_t iap_flash_set_jumpaddr(void)
{
	uint8_t err = 0x03;
	if ((uint32_t)&ThisVersion >= FUNCTION_B_ADDRESS)
	{
		FLASH_Unlock();
	    FLASH_ErasePage(FUNCTION_JUMP_ADDRESS);
	    FLASH_Lock();
		FLASH_Unlock();
		FLASH_ProgramWord(FUNCTION_JUMP_ADDRESS, FUNCTION_A_ADDRESS);
		FLASH_Lock();
		
		if ( *(uint32_t *)FUNCTION_JUMP_ADDRESS == FUNCTION_A_ADDRESS)
		{
			err = 0x00;
		}
		
	}
	else if ( (uint32_t)&ThisVersion >= FUNCTION_A_ADDRESS)
	{
		FLASH_Unlock();
	    FLASH_ErasePage(FUNCTION_JUMP_ADDRESS);
	    FLASH_Lock();
	    FLASH_Unlock();
		FLASH_ProgramWord(FUNCTION_JUMP_ADDRESS, FUNCTION_B_ADDRESS);
		FLASH_Lock();
		
		if ( *(uint32_t *)FUNCTION_JUMP_ADDRESS == FUNCTION_B_ADDRESS)
		{
			err = 0x00;
		}
	}
	return err;
  
}

//uint8_t FLASH_EraseAllNextZONE(uint32_t Page_Address,u16 size)
//{
//	u16 i;
//	uint32_t newerasepage;
//	uint8_t ErasePageStatus;
//	
//	newerasepage = Page_Address;
//	i = size/1024;
//	while((i--)&& (ErasePageStatus == FLASH_COMPLETE))
//	{
//		FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
//		ErasePageStatus = FLASH_ErasePage(newerasepage);
//		newerasepage = newerasepage + 0x400;
//	}
//	return ErasePageStatus;
//}


/*******************************************************************************
* Function Name  : USB_LP_CAN_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
CanRxMsg RxMessage;
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  //CanRxMsg RxMessage;
	BaseType_t xHigherPriorityTaskWoken, Result;
  CanardCANFrame rx_frame;
	xHigherPriorityTaskWoken = pdFALSE;
  CAN_Receive(CAN1,CAN_FIFO0,&RxMessage);//��ȡ����	
  rx_frame.id = RxMessage.ExtId;
  rx_frame.data_len = RxMessage.DLC;

  memcpy(rx_frame.data,RxMessage.Data,RxMessage.DLC); //��������	 
	canardHandleRxFrame(&canard, &rx_frame, 1000);
	Result = xSemaphoreGiveFromISR( BinarySemaphore, &xHigherPriorityTaskWoken );
	if(Result == pdTRUE)
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/******************************************************************************/