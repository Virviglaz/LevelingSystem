#include "DS1822.h"
unsigned char Search_Rom_SN[One_Wire_Device_Number_MAX][DS1822_SERIAL_NUM_SIZE];

unsigned char DS1822_Search_Rom_One_Device (unsigned char (*Serial_Num)[DS1822_SERIAL_NUM_SIZE])
{																
	unsigned char cnt_bits;
	unsigned char cnt_bytes;
	unsigned char tmp;
	unsigned char tmp2=0;
	unsigned char dev_cnt=0;

	tmp = One_Wire_Reset();
	if (tmp!=One_Wire_Success) return tmp;
	One_Wire_Write_Byte(One_Wire_Search_ROM);
	
	for (cnt_bytes=0;cnt_bytes!=8;cnt_bytes++)
	{						
		for (cnt_bits=0;cnt_bits!=8;cnt_bits++)
		{
			tmp=One_Wire_Read_Bit();
	 		if (One_Wire_Read_Bit()==tmp) dev_cnt++;
			One_Wire_Write_Bit(tmp);
			if (tmp!=0) tmp2|=(1<<cnt_bits);		
		}
	( *Serial_Num)[cnt_bytes]=tmp2;
	tmp2=0;	
	}
	if (Crc8Dallas(DS1822_SERIAL_NUM_SIZE,(*Serial_Num))==0) return One_Wire_Success;
	else return One_Wire_CRC_Error; 
}

unsigned char DS1822_Start_Conversion_by_ROM (unsigned char (*Serial_Num)[DS1822_SERIAL_NUM_SIZE])
{
	unsigned char cnt;
	cnt=One_Wire_Reset();
	if (cnt!=One_Wire_Success) return cnt;
	One_Wire_Write_Byte(One_Wire_Match_ROM);
	for (cnt=0;cnt!=8;cnt++) One_Wire_Write_Byte((*Serial_Num)[cnt]);
	One_Wire_Write_Byte(DS1822_CONVERT_T_CMD);
	return One_Wire_Success;
}

unsigned char DS1822_Get_Conversion_Result_by_ROM_CRC (unsigned char (*Serial_Num)[DS1822_SERIAL_NUM_SIZE], unsigned int * temp_code, char * opcode)
{
	unsigned char cnt;
	unsigned char inbuff[DS1822_STRATCHPAD_SIZE];
	cnt=One_Wire_Reset();
	if (cnt!=One_Wire_Success) return cnt;
	One_Wire_Write_Byte(One_Wire_Match_ROM);
	for (cnt=0;cnt!=8;cnt++) One_Wire_Write_Byte((*Serial_Num)[cnt]);
	One_Wire_Write_Byte(DS1822_READ_STRATCHPAD_CMD);
	for (cnt=0;cnt!=DS1822_STRATCHPAD_SIZE;cnt++) inbuff[cnt]=One_Wire_Read_Byte();
	if (Crc8Dallas(DS1822_STRATCHPAD_SIZE,inbuff)==0) 
        {
          *temp_code = inbuff[0]|(inbuff[1]<<8);
          *opcode = inbuff[2];
        }
	else	return One_Wire_CRC_Error;
	return One_Wire_Success;
}

unsigned char DS1822_Search_Rom (unsigned char * devices_found)
{
	unsigned long path,next,pos;                          /* decision markers */                              
 	unsigned char bit,chk;                                /* bit values */
	unsigned char cnt_bit, cnt_byte, cnt_num,tmp;
 	path=0;                                     /* initial path to follow */
 	cnt_num=0;
	do
	{                                         /* each ROM search pass */
		//(initialize the bus with a reset pulse)
 		tmp=One_Wire_Reset();
 			if (tmp!=One_Wire_Success) return tmp;
		//(issue the 'ROM search' command)
		One_Wire_Write_Byte(One_Wire_Search_ROM);
     	next=0;                                 /* next path to follow */
     	pos=1;                                  /* path bit pointer */                              
  		for (cnt_byte=0;cnt_byte!=8;cnt_byte++)
		{
			Search_Rom_SN[cnt_num][cnt_byte]=0;
			for (cnt_bit=0;cnt_bit!=8;cnt_bit++)
  			{                                     /* each bit of the ROM value */
 				//(read two bits, 'bit' and 'chk', from the 1-wire bus)
				bit=One_Wire_Read_Bit();
				chk=One_Wire_Read_Bit();
        		if(!bit && !chk)
					{                   /* collision, both are zero */
             			if(pos&path) bit=1;             /* if we've been here before */
             			else next=(path&(pos-1))|pos;   /* else, new branch for next */
             			pos<<=1;
          			}
 				//(write 'bit' to the 1-wire bus)
 				One_Wire_Write_Bit(bit);
	 
 				//(save this bit as part of the current ROM value)
				if (bit!=0) Search_Rom_SN[cnt_num][cnt_byte]|=(1<<cnt_bit);
     		}
		}
 		//(output the just-completed ROM value)
     	path=next;
		cnt_num++;
 	}while(path);
	* devices_found = cnt_num;
	return One_Wire_Success;
}

unsigned char DS1822_Search_Rom2 (unsigned char * devices_found, unsigned char (* SN_ROM)[One_Wire_Device_Number_MAX][DS1822_SERIAL_NUM_SIZE])
{
	unsigned long path,next,pos;                                                       
 	unsigned char bit,chk;                                
	unsigned char cnt_bit, cnt_byte, cnt_num,tmp;
 	path=0;                                     
 	cnt_num=0;
	do
	{                                         
 		tmp=One_Wire_Reset();
 			if (tmp!=One_Wire_Success) return tmp;
			One_Wire_Write_Byte(One_Wire_Search_ROM);
     	next=0;                                 
     	pos=1;                                                             
  		for (cnt_byte=0;cnt_byte!=8;cnt_byte++)
		{
			(*SN_ROM)[cnt_num][cnt_byte]=0;
			for (cnt_bit=0;cnt_bit!=8;cnt_bit++)
  		{                                     
				bit=One_Wire_Read_Bit();
				chk=One_Wire_Read_Bit();
        if(!bit && !chk)
				{                   
          if(pos&path) bit=1;            
            else next=(path&(pos-1))|pos;   
          pos<<=1;
        }
 			One_Wire_Write_Bit(bit);
			if (bit!=0) (*SN_ROM)[cnt_num][cnt_byte]|=(1<<cnt_bit);
     	}
		}
    path=next;
		cnt_num++;
 	}while(path);
	* devices_found = cnt_num;
	return One_Wire_Success;
}

unsigned char DS1822_Start_Conversion_Skip_Rom (void)			 
{	
	unsigned char tmp; 											
	tmp=One_Wire_Reset();
	if (tmp!=One_Wire_Success) return tmp;												
	One_Wire_Write_Byte(One_Wire_Skip_ROM);
	One_Wire_Write_Byte(DS1822_CONVERT_T_CMD);
	if (One_Wire_Read_Byte()==0)	return One_Wire_Success;
		else	return One_Wire_Device_Busy;
}

unsigned char DS1822_Read_Temp_NoCRC_Skip_Rom (unsigned int * temperatura)				 
{
	unsigned char tmp; 											
	tmp=One_Wire_Reset();
	if (tmp!=One_Wire_Success) return tmp;
	One_Wire_Write_Byte(One_Wire_Skip_ROM);
	One_Wire_Write_Byte(DS1822_READ_STRATCHPAD_CMD);
	* temperatura = One_Wire_Read_Byte()|(One_Wire_Read_Byte()<<8);
	return One_Wire_Success;
}
