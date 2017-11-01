#include "usbhw.h"
#include "usbreg.h"
#include "type.h"
#include "usbcfg.h"

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
