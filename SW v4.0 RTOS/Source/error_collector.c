#include "error_collector.h"
#include "data_logger.h"
#include "rtc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "strings.h"
#include <stdio.h>

/* Variables */
ErrorTypeDef Error;

void ErrorHandle (ErrorGroupTypeDef Group, const char * Error)
{
	char * buf, * date;
	if (!Group) return;
	date = pvPortMalloc(21);
	DateAndTimePrint(date);
	
	buf = pvPortMalloc(StringLen((char*)Error) + 25);
	sprintf(buf, "%s %s", date, Error);
	//DateAndTimePrint(buf);
	//FillString((char*)Error, buf + StringLen(buf));
	LogToSD("ERRLOG.TXT", buf, StringLen(buf), 0);
	vPortFree(date);
	vPortFree(buf);
}


