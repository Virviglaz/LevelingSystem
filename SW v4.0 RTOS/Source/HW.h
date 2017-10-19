//#define VER321
#define DIAG_F									GPIOA, GPIO_Pin_1
#define DIAG_R									GPIOB, GPIO_Pin_12

#define BrightnessControl
#define LEDS_POWER_ON						GPIOA, GPIO_Pin_2

#define FLU_LED									GPIOC, GPIO_Pin_4
#define FLM_LED									GPIOA, GPIO_Pin_7
#define FLD_LED									GPIOA, GPIO_Pin_6

#define FRU_LED									GPIOA, GPIO_Pin_5
#define FRM_LED									GPIOA, GPIO_Pin_4
#define FRD_LED									GPIOA, GPIO_Pin_3

#define RLU_LED									GPIOB, GPIO_Pin_11
#define RLM_LED									GPIOB, GPIO_Pin_10
#define RLD_LED									GPIOB, GPIO_Pin_2

#define RRU_LED									GPIOB, GPIO_Pin_1
#define RRM_LED									GPIOB, GPIO_Pin_0
#define RRD_LED									GPIOC, GPIO_Pin_5

#define RCS											GPIOC, GPIO_Pin_0
#define SD_Present							GPIOC, GPIO_Pin_1
#define SDCS										GPIOC, GPIO_Pin_2
#define RMSCK										GPIOC, GPIO_Pin_10
#define RMOSI										GPIOC, GPIO_Pin_11
#define JP1											GPIOC, GPIO_Pin_7
#define JP2											GPIOC, GPIO_Pin_6
#define LED11										GPIOC, GPIO_Pin_12
#define MCO											GPIOA, GPIO_Pin_8
#define DRY											GPIOA, GPIO_Pin_15

#define I2C_SCL									GPIOB, GPIO_Pin_6
#define I2C_SDA									GPIOB, GPIO_Pin_7

#define LED_OK_AN_TRIGGER				200
#define USB_VOLTAGE							ADC_Channel_13
#define LED_FB_ADC							ADC_Channel_1

#define USB_ON									GPIOC, GPIO_Pin_9

#ifdef VER321
	#define BTN										GPIOC, GPIO_Pin_13
	//for TOP PCB since 3.2, PC13 traced
#else
	#define BTN										GPIOC, GPIO_Pin_8
	//Pin.2 left unconnected, PC8 used in that case
#endif

#define BTN_PRESSED	           (PIN_SYG(SW1) == 0)
#define BTN_DEPRESSED					 (PIN_SYG(SW1) != 0)
#define LevelingOnButtonCNT    3 //0.1sec * Value
#define FloatingModeButtonCNT	 50 

/* I2C 24C64 EEPROM Configure */
#define I2c_EEPROM_Address					0xA0			//default address 0xA0 for current board revision
#define PageSizeBytes								32				//32 bytes default page size
#define I2c_EEPROM_WriteTime_ms			50				//10ms minimum write time

#define GyroSampleRateHzNorm				10
#define GyroSampleRateHzFast				100
#define RoomSizeH										6058
#define RoomSizeW										2407
#define MaxBlinkFreq								50
#define XNormAcceleration						3
#define YNormAcceleration						3
#define XFastAcceleration						8
#define YFastAcceleration						8
#define XNormZeroAccuracy						50
#define YNormZeroAccuracy						50
#define XLowZeroAccuracy						250
#define YLowZeroAccuracy						250
#define BlinkAcc										10
#define Integrator									10

#define TIM2_Freq										100 //Hz
#define X_OffsetAddress							BKP_DR2
#define Y_OffsetAddress							BKP_DR3
#define LV_ConfigurationAddress			BKP_DR4

#define TimeLogSecAddress						BKP_DR7
#define RelayStateAddress						BKP_DR8
#define AddLedsNumAddress						BKP_DR9
#define AddLedsColor								BKP_DR10

#define BUTTON1_PRESSED							PIN_SYG(SW1)==0
#define LED_Delay										20
#define MaxADDLEDsAmount						100
#define LevelingAutoOffMin					120

#define White 											0x00FFFFFF
#define Red   											0x00FF0000
#define Green 											0x0000FF00
#define Blue												0x000000FF

#define SI7005_ReadInterval					10
#define SI7005_I2C									I2C1
#define SI7005_I2C_Address					0x80
#define Humidity_EXT1_Number				10	
#define Humidity_EXT2_Number				11
#define SI7005_SensorNumberINT 			12

#define BMP180_ReadIntervalSec			10

#define AddLedsSPI									SPI2
#define MaxSensors									15

// 0...9 Temperature Sensors DS18B20
// 10, 11 Humidity Sensors Internal & External

/* STM8 Commands */
#define CONV_RESULT     0xEE
#define RF_DATA_RES     0xFA
#define SENSOR_SAVED    0x0F
#define REG_NEW_SENSOR	0xAA
#define PROTECT_FW      0xAE
#define DELETE_SENSOR   0xDE

/* BOOTLOADER CONFIGURATION */
#define FLASH_PAGE_SIZE		0x03FF
#define BOOT_FLASH_OFFSET	0x08000000
#define BOOT_PAGES_USED		10
#define MAIN_FLASH_SIZE		0x16800
#define MAIN_FLASH_OFFSET 0x08009800

