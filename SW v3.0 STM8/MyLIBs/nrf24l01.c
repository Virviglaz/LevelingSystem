#include "nrf24l01.h"
/* VERSION 2.11 */
/*  31.01.2014  */

#include "HW.h"

unsigned char 	RF_Read_Cmd(unsigned char adrs);
unsigned char   RF_Send_Cmd(unsigned char adrs, unsigned char cmd);
void RF_Send_Adrs(unsigned char adrs, unsigned char cmd[5]);
void RF_IRQ_CLEAR (unsigned char CMD);
void RF_Flush (unsigned char CMD);

#ifndef RF_CE_PIN
	#error NO RF Chip Enable PIN Defined in main.h!
#endif
#ifndef RF_CSN_PIN
	#error NO RF Chip Select PIN Defined in main.h!
#endif
#ifndef RF_IRQ_PIN
	#error NO RF IRQ PIN Defined in main.h!
#endif

/**
  * @brief  Initialise NRF24L01 
  * @param  RF_InitStruct: pointer to an RF_InitTypeDef structure that contains
  *   the configuration information for the specified parameters.
  * @retval STATUS REG content (0x0E by default, 0x00 or 0xFF are impossible values)
  */
unsigned char RF_Init(RF_InitTypeDef* RF_InitStruct)		  
{	
    unsigned char rftmp=RF_Send_Cmd(CONFIG_REG, RF_InitStruct->RF_Config|RF_InitStruct->RF_Power_State|RF_InitStruct->RF_CRC_Mode|RF_InitStruct->RF_Mode);															
    RF_Send_Cmd(EN_AA_REG,RF_InitStruct->RF_Pipe_Auto_Ack);										//EN_AA 		REG
    RF_Send_Cmd(EN_RXADDR_REG,RF_InitStruct->RF_Enable_Pipe);									//EX_RXADDR		REG
    RF_Send_Cmd(SETUP_AW_REG,RF_InitStruct->RF_Setup);											//SETUP_AW		REG
    RF_Send_Cmd(SETUP_RETR_REG,(RF_InitStruct->RF_Auto_Retransmit_Count&0x0F)|(((RF_InitStruct->RF_Auto_Retransmit_Delay)&0x0F)<<4));
    RF_Send_Cmd(RF_CH_REG,(RF_InitStruct->RF_Channel&0x7F));									//RF Chanel		REG
    RF_Send_Cmd(RF_SETUP_REG,(0x01|RF_InitStruct->RF_TX_Power|RF_InitStruct->RF_Data_Rate));	//RF_SETUP		REG
                
    /*checking ZERO configuration*/
    if (RF_InitStruct->RF_RX_Adress_Pipe0[0])
        RF_Send_Adrs(RX_ADDR_P0_REG,RF_InitStruct->RF_RX_Adress_Pipe0);								//RX adress for PIPE0
    if (RF_InitStruct->RF_RX_Adress_Pipe1[0])
        RF_Send_Adrs(RX_ADDR_P1_REG,RF_InitStruct->RF_RX_Adress_Pipe1);								//RX adress for PIPE1
    if (RF_InitStruct->RF_RX_Adress_Pipe2)
        RF_Send_Cmd(RX_ADDR_P2_REG,RF_InitStruct->RF_RX_Adress_Pipe2);								//RX adress for PIPE2 (only LSB) 
    if (RF_InitStruct->RF_RX_Adress_Pipe3)
	RF_Send_Cmd(RX_ADDR_P3_REG,RF_InitStruct->RF_RX_Adress_Pipe3);								//RX adress for PIPE3 (only LSB)
    if (RF_InitStruct->RF_RX_Adress_Pipe4)
	RF_Send_Cmd(RX_ADDR_P4_REG,RF_InitStruct->RF_RX_Adress_Pipe4);								//RX adress for PIPE4 (only LSB)
    if (RF_InitStruct->RF_RX_Adress_Pipe5)
	RF_Send_Cmd(RX_ADDR_P5_REG,RF_InitStruct->RF_RX_Adress_Pipe5);								//RX adress for PIPE5 (only LSB)
    if (RF_InitStruct->RF_TX_Adress[0])
	RF_Send_Adrs(TX_ADDR_REG,RF_InitStruct->RF_TX_Adress);
    if (RF_InitStruct->RF_Payload_Size_Pipe0)
	RF_Send_Cmd(RX_PW_P0_REG,RF_InitStruct->RF_Payload_Size_Pipe0);
    if (RF_InitStruct->RF_Payload_Size_Pipe1)
      	RF_Send_Cmd(RX_PW_P1_REG,RF_InitStruct->RF_Payload_Size_Pipe1);
    if (RF_InitStruct->RF_Payload_Size_Pipe2)
       	RF_Send_Cmd(RX_PW_P2_REG,RF_InitStruct->RF_Payload_Size_Pipe2);
    if (RF_InitStruct->RF_Payload_Size_Pipe3)
	RF_Send_Cmd(RX_PW_P3_REG,RF_InitStruct->RF_Payload_Size_Pipe3);
    if (RF_InitStruct->RF_Payload_Size_Pipe4)
    	RF_Send_Cmd(RX_PW_P4_REG,RF_InitStruct->RF_Payload_Size_Pipe4);
    if (RF_InitStruct->RF_Payload_Size_Pipe5)
	RF_Send_Cmd(RX_PW_P5_REG,RF_InitStruct->RF_Payload_Size_Pipe5);
    PIN_ON(RF_CE_PIN);
    
    return rftmp;
    //return RF_Read_Cmd(CONFIG_REG);			 
}

/**
  * @brief  Send Payload via NRF24L01 
  * @param  unsigned char * data: pointer to first data memory location
  * @param  unsigned char DataLen: lenth of data array [1..32] for NRF24L01
  * @retval RF_SUCCESS or RF_ERROR_CHIP_NOT_RESPONDING if data send time overlimit
  */
unsigned char RF_SendPayload (char * data, unsigned char DataLen)
{
  unsigned long DS_Delay=Payload_send_delay;
  PIN_OFF(RF_CSN_PIN); 
  SPI_SendByte(RF_SendPayload_CMD);
  while(DataLen) 
  {
    SPI_SendByte(*data++);
    DataLen--;
  }
  PIN_ON(RF_CSN_PIN);
  PIN_ON(RF_CE_PIN);
  while((DS_Delay!=0)&(PIN_SYG(RF_IRQ_PIN)!=0)) DS_Delay--;
  PIN_OFF(RF_CE_PIN);
  RF_IRQ_CLEAR(RF_TX_DS_IRQ_CLEAR);
  RF_Flush(RF_Flush_TX_CMD);
  if(DS_Delay==0) return RF_ERROR_CHIP_NOT_RESPONDING;
  return RF_SUCCESS;
}

/**
  * @brief  Send Payload via NRF24L01 with Acknowledge
  * @param  unsigned char * data: pointer to first data memory location
  * @param  unsigned char DataLen: lenth of data array [1..32] for NRF24L01
  * @retval RF_DATA_SEND_NO_ACK_RECEIVED if data send ok but no Acknowledge received 
	    RF_DATA_SEND_ACK_RECEIVED_OK if data send ok and Acknowledge received
	    RF_ERROR_CHIP_NOT_RESPONDING if data send time overlimit
  */
unsigned char RF_SendPayloadACK (char * data, unsigned char DataLen) //return 0 if no ACK received, return 0xFF if ERROR
{
  unsigned long DS_Delay=Payload_send_delay;
  unsigned char tmp=RF_DATA_SEND_NO_ACK_RECEIVED;
  PIN_OFF(RF_CSN_PIN); 
  SPI_SendByte(RF_SendPayload_CMD);
  while(DataLen) 
  {
    SPI_SendByte(*data++);
    DataLen--; 
  }
  PIN_ON(RF_CSN_PIN);
  PIN_ON(RF_CE_PIN);
  while((DS_Delay!=0)&(PIN_SYG(RF_IRQ_PIN)!=0)) DS_Delay--;
  PIN_OFF(RF_CE_PIN);

  PIN_OFF(RF_CSN_PIN);
  if (SPI_ReadByte(0xFF)&RF_TX_DS_IRQ_CLEAR) tmp=RF_DATA_SEND_ACK_RECEIVED_OK; //IRQ reflected, ACK received
  PIN_ON(RF_CSN_PIN);
  
  if(DS_Delay==0) tmp=RF_ERROR_CHIP_NOT_RESPONDING;  
  RF_IRQ_CLEAR(RF_MAX_RT_IRQ_CLEAR|RF_TX_DS_IRQ_CLEAR);
  RF_Flush(RF_Flush_TX_CMD);

  return tmp; 
}

/**
  * @brief  Receive data from NRF24L01
  * @param  unsigned char * data: pointer to first data memory location
  * @param  unsigned char DataLen: lenth of data array [1..32] for NRF24L01
  * @retval unsigned char PipeNum - PIPE number of data received
	RF_NO_DATA_RECEIVED if no data in FIFO and IRQ pin not reflected
  */
unsigned char RF_Receive_Data(char * Data, unsigned char Data_Len)					  			//returns 0xFF if no data received. Data_Len = num of bytes to receive
{																																							//returns PIPE num if data received succesfully		
	unsigned char tmp;		
	if (PIN_SYG(RF_IRQ_PIN)!=0)	return RF_NO_DATA_RECEIVED;										//NO DATA received
	else
	{	
		PIN_OFF(RF_CSN_PIN);
		tmp = (SPI_ReadByte(0x61)&0x0E)>>1;					//read status reg and send R_RX_PAYLOAD Command															  	
		while(Data_Len)																			//read DATA from FIFO
		{
			*Data++=SPI_ReadByte(0xFF);
			Data_Len--;
		}
		PIN_ON(RF_CSN_PIN);		
		
		if ((RF_Read_Cmd(FIFO_STATUS_REG)&RF_RX_FIFO_EMPTY_Bit)!=0)	//check available data in RX FIFO
			RF_IRQ_CLEAR(RF_RX_DR_IRQ_CLEAR);	//if NO DATA in RX FIFO, clear IRQ
		return tmp;
	}			
}





/* INTERNAL FUNCTIONS DEFINITIONS */
/* NOTHING INTERESTING HERE */

unsigned char RF_Send_Cmd(unsigned char adrs, unsigned char cmd)							//write data to register procedure
{	unsigned char temp=0;
	if ((adrs<0x0A)|(cmd!=0)){
	PIN_OFF(RF_CSN_PIN);
	temp=SPI_ReadByte((0x1F&adrs)|(1<<5));
	SPI_SendByte(cmd);
	PIN_ON(RF_CSN_PIN);			 }
	return temp;
}
unsigned char RF_Read_Cmd(unsigned char adrs)										//read data from register
{	unsigned char temp;	
	PIN_OFF(RF_CSN_PIN);
	SPI_SendByte(0x1F&adrs);
	temp=SPI_ReadByte(0xFF);
	PIN_ON(RF_CSN_PIN);
	return temp;
}

void RF_Send_Adrs(unsigned char adrs, unsigned char cmd[Max_Adress_Len])
{	unsigned char temp;
	if (cmd[0]!=0){
	PIN_OFF(RF_CSN_PIN);
	SPI_SendByte(adrs|(1<<5));
	for (temp=0;temp!=Max_Adress_Len;temp++) SPI_SendByte(cmd[temp]);	
	PIN_ON(RF_CSN_PIN);  } 
}

unsigned char RF_Carrier_Detect(void)										//returns 1 if Carrier Detected on current channel
	{return (RF_Read_Cmd(CD_REG)&0x01);}

unsigned char RF_Count_Lost_Packets(void)									//returns num of Lost Packatets 
	{return (((RF_Read_Cmd(OBSERV_TX_REG))&0xF0)>>4);}

unsigned char RF_Count_Resend_Packets(void)								//returns nut of Resend Packets
	{return ((RF_Read_Cmd(OBSERV_TX_REG))&0x0F);}

void RF_IRQ_CLEAR (unsigned char CMD)											//Clears IRQ
	{RF_Send_Cmd(STATUS_REG, CMD);} 

void RF_Flush (unsigned char CMD)
{
  PIN_OFF(RF_CSN_PIN);
  SPI_SendByte(CMD);
  PIN_ON(RF_CSN_PIN);
}

unsigned char RF_Sleep (void)
{
 return RF_Send_Cmd(CONFIG_REG, RF_Read_Cmd(CONFIG_REG)&(~RF_Power_On));  
}

unsigned char RF_WakeUp (void)
{
 return RF_Send_Cmd(CONFIG_REG, RF_Read_Cmd(CONFIG_REG)|RF_Power_On); 
}

unsigned char RF_FastTX_Address_Confirure (char * Address)
{
  u8 cnt, rtn;
  PIN_OFF(RF_CSN_PIN);
  rtn = SPI_ReadByte(TX_ADDR_REG|(1<<5));
  for (cnt=0; cnt!=sizeof(Address); cnt++)
    SPI_SendByte(Address[cnt]);	
  PIN_ON(RF_CSN_PIN);
  return rtn;
}


