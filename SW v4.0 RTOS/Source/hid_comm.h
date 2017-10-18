#include "usbhw.h"
#include "usbreg.h"
#include "type.h"
#include "usbcfg.h"

typedef struct 
{
	char RX_ReadyFlag;
	char TX_DataReadyFlag;
	char TX_Buffer[USB_PACKET_LEN];
	char * RX_Buffer;
	char ZERO_Buffer[USB_PACKET_LEN];
	char LastPackageZeroFlag;
}USBStructTypeDef;

void GetReportToPC (void);
void ReportFromPCHandle (void);
void ZeroPacketInitTask (void * pvArg);
void USB_RX_DataHandler (void);
void USB_TX_DataHandler (void);
void USB_SetTime (void);

typedef union
{
	long lVar;
	short sVar[2];
	char bVar[4];
} V32_TypeDef;
