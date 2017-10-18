#include "error_collector.h"
#include "data_logger.h"
#include "rtc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "strings.h"

/* Variables */
ErrorTypeDef Error;

void ErrorHandle (ErrorGroupTypeDef Group, const char * Error)
{
	char * buf;
	if (!Group) return;
	
	buf = pvPortMalloc(StringLen((char*)Error) + 20);
	DateAndTimePrint(buf);
	FillString((char*)Error, buf + StringLen(buf));
	LogToSD("ERRLOG.TXT",buf, StringLen(buf), 0);
	vPortFree(buf);
}


