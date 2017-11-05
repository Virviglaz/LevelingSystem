#include "usbhw.h"
#include "usbreg.h"
#include "type.h"
#include "usbcfg.h"

#ifndef PRINT
	#define PRINT(fmt, ...) \
							do { printf(fmt, ##__VA_ARGS__); printf("\n"); } while (0)
#endif

#ifndef	DEBUG					
	#define DEBUG(fmt, ...) \
							do if (MainConfig.Debug) \
							{ \
								printf("%s:%d:%s(): ", __FILE__, __LINE__, __func__); \
								printf(fmt, ##__VA_ARGS__); \
								printf("\n"); \
							}while (0)					
#endif

typedef struct 
{
	BYTE TX_Buffer[USB_PACKET_LEN];
	BYTE RX_Buffer[USB_PACKET_LEN];
	BYTE ZERO_Buffer[USB_PACKET_LEN];
	BYTE NOP_Buffer[USB_PACKET_LEN];
}USBStructTypeDef;

void GetReportToPC (void);
void ReportFromPCHandle (void);
void ZeroPacketInitTask (void * pvArg);
void USB_RX_DataHandler (void * pvArg);
void USB_TX_DataHandler (void * pvArg);
void USB_SetTime (void);

typedef union
{
	long lVar;
	short sVar[2];
	char bVar[4];
} V32_TypeDef;
