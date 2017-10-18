#define Time_Pulse_Delay_Low	        10      //10
#define Time_Pulse_Delay_High	        40      //40
#define Time_Reset_Low			500
#define Time_After_Reset		350
#define Time_Hold_Down			10

#define One_Wire_Read_ROM		0x33
#define One_Wire_Skip_ROM		0xCC
#define One_Wire_Search_ROM		0xF0
#define One_Wire_Match_ROM		0x55

#define One_Wire_Success		0x00
#define One_Wire_Error_No_Echo	        0x01
#define One_Wire_Bus_Low_Error	        0x02
#define One_Wire_Device_Busy	        0x03
#define One_Wire_CRC_Error		0x04

unsigned int One_Wire_Reset(void);
void One_Wire_Write_Byte(unsigned char Byte);
unsigned char One_Wire_Read_Byte(void);
unsigned char One_Wire_Read_Bit (void);
void One_Wire_Write_Bit (unsigned char Bit);
