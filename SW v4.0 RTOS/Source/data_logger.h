char LoggerInit (void);
char LogToSD (char * filename, char * buf, unsigned int size, char CreateNew);
char DeleteFile (char * filename);
char PrintToFile (char * filename, char * buf);
void DateAndTimePrint (char * string);
void DatePrint (char * string);
char PrintFloatDataToFile (char * filename, const char * Text, float Data, const char * units);
char PrintIntDataToFile (char * filename, const char * Text, int Data, const char * units);
char PrintFloatResultToFile (char * filename, const char * Text, char num, float Data, const char * units);
char PrintIntResultToFile (char * filename, const char * Text, char num, int Data, const char * units);
char ReadFromSD (const char * filename, char * buf, unsigned int * ByteReaded);
void LogToSD_Task (void * pvArg);
