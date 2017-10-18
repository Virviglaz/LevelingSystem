
#define HID_OK														0x30																								//DEC				//HEX				//Version 2.0
#define TurnOnLevelingSystem							0x31	//returns nothing															//49				//0x31
#define TurnOffLevelingSystem							0x32	//returns nothing															//50				//0x32
#define CallibrateZeroPosition						0x33	//returns nothing															//51				//0x33
#define SearchSensorsRoutine							0x34	//returns sensors found amount								//52				//0x34			//UNSUPPORTED
#define StartConversionAll								0x35	//returns error num														//53				//0x35			//UNSUPPORTED
#define GetTempResultSenseNum							0x36  //returns Sensor temperature value						//54				//0x36
#define StartConversionSingleSensor				0x37	//returns error num														//55				//0x37			//UNSUPPORTED
#define GetBatteryVoltage									0x38  //returns wADC value													//56				//0x38			//UNSUPPORTED
#define GetDCINVoltage										0x39  //returns wADC value													//57				//0x39			//UNSUPPORTED
#define GetErrorsAmount										0x3A  //return bAmount of errors from Error Buffer	//58				//0x3A			//UNSUPPORTED
#define GetErrorBuffer										0x3B  //receive all error buffer										//59				//0x3B			//UNSUPPORTED
#define ClearErrorBuffer									0x3C  //clears all errors from memory								//60				//0x3C
#define CheckLeds													0x3D	//checking connection to leds and store error to Error Buffer 
#define SaveSensorsDataToEEPROM						0x3E	//save s/n all found sensors to EEPROM				//62				//0x3E			//UNSUPPORTED
#define GetSensorSerialNumber							0x3F  //get sensor's serial number + CRC = 8 bytes	//63				//0x3F			//UNSUPPORTED
#define TurnOnAutoMeaguring								0x40	//temp, voltages auto meaguring								//64				//0x40			//UNSUPPORTED
#define TurnOffAutoMeaguring							0x41	//temp, voltages auto meaguring								//65				//0x41			//UNSUPPORTED
#define GetTime														0x42	//hours, minutes, seconds retrieve						//66				//0x42			//UNSUPPORTED
#define GetDate														0x43	//year, month, day retrieve										//67				//0x43			//UNSUPPORTED
#define SetTime														0x44	//hours, minutes, seconds retrieve						//68				//0x44
#define SetDate														0x45	//year, month, day retrieve										//69				//0x45			//UNSUPPORTED
				

/* Version 2.0 */
#define InstallNewSensor									0x46																								//70
#define SaveConfigToEEPROM								0x47
#define RecoverConfig											0x48
#define EnableFloatingMode								0x49
#define DisableFloatingMode								0x50
#define RelayChangeState									0x51
#define ADDLEDSChangeState								0x52
#define PROTECT_FW_COMM										0x53
#define DELETE_SENSOR_COMM								0x54
#define LED_TEST_ROUTINE									0x55
#define SET_LEDS_BRIGHTNESS								0x56
#define GET_INSTALLED_SENSORS							0x57
#define BMP180_ResultReady								0x58
#define INSTALL_NEY_KEY         					0x72
#define BOOTLOADER_STM8         					0x73
#define IR_COMMAND              					0x80

/* Version 4.0 RTOS */
#define ChangeSettingsCommand							0x81
#define GetSettingsCommand								0x82
#define SaveFileDefineNameCommand					0x83
#define SaveFileDefineName								0x84
#define PERFORM_SW_RESET									0x85
#define SaveFileFinishedCommand						0x86

/* Subgroup for Version 4.0 RTOS */
#define sAutoOffTimerValueS								0x00
#define sPositionNum											0x01
#define sSI7005_Int  											0x02
#define sBMP180_Int												0x03
#define sLogToSDint												0x04
#define sNormZeroAreaSize									0x05
#define sNormFilterValue									0x06
#define sCoarseZeroAreaSize								0x07
#define sCoarseFilterValue								0x08
