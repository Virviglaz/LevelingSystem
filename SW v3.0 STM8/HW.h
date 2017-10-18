#define RF_CE_PIN       GPIOD, GPIO_PIN_2
#define RF_CSN_PIN      GPIOD, GPIO_PIN_3
#define RF_IRQ_PIN      GPIOD, GPIO_PIN_4

#define One_Wire_RX     GPIOC, GPIO_PIN_4
#define One_Wire_TX     GPIOC, GPIO_PIN_3

#define IR              GPIOA, GPIO_PIN_3

#define MAX_CommandLen  30
#define MAX_Sensors     12
#define EEPROM_ADDRESS0 0x4000
#define NRF_NOERROR     0x0E
#define OutDataLen      100
#define Pipe0Len        1
#define Pipe1Len        16
#define RF_KEY_EE_ADD   100

#define REG_NEW_SENSOR  0xAA    //      0 - 0xAA
                                //      1 - SensNum[0..200]
                                //      2..10 - Serial Number
                                //      10 - CRC8Dallas
#define PROTECT_FW              0xAE
#define DELETE_SENSOR           0xDE
#define CONV_RESULT             0xEE
#define RF_DATA_RES             0xFA
#define SENSOR_SAVED            0x0F
#define SEARCH_SENSORS          0x34
#define GET_INSTALLED_SENSORS   0x57
#define INSTALL_NEY_KEY         0x72
#define BOOTLOADER_STM8         0x73
#define IR_COMMAND              0x80

typedef struct 
{
  char Data[16];
  u8 Error;
  u8 PipeNum;
}RF_TypeDef;

typedef struct
{
  u16 HighDataDelay; //1 & 2
  u16 LowDataDelay;  //3 & 4
  u8 DataLen;        //5   
}IR_TypeDefStruct;
                                          