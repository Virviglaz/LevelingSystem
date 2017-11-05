using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LevelingSystem5._0
{
    public static class Const
    {
        public const string MainTitle = "MMH Utility";
        public const byte FilePageSize = 50;

        /* Commands List */
        public const byte TurnOnLevelingCommand = 0x31;
        public const byte TurnOffLevelingCommand = 0x32;
        public const byte CallibrateZeroCommand = 0x33;
        public const byte SearchSensorsCommand = 0x34;
        public const byte GetTempResultCommand = 0x36;
        public const byte StartConversionSingleSensor = 0x37;
        public const byte GetBatteryVoltageCommand = 0x38;
        public const byte GetDCINVoltageCommand = 0x39;
        public const byte CheckLedsCommand = 0x3D;
        public const byte TurnOnAutoMeaguring = 0x40;
        public const byte TurnOffAutoMeaguring = 0x41;
        public const byte GetErrorBuffer = 0x3B;
        public const byte ClearErrorBuffer = 0x3C;
        public const byte SaveSensorsDataToEEPROM = 0x3E;
        public const byte InstallNewSensor = 0x46;
        public const byte GetTime = 0x42;
        public const byte GetDate = 0x43;
        public const byte SetTime = 0x44;
        public const byte SetDate = 0x45;
        public const byte EnableFloatingMode = 0x49;
        public const byte DisableFloatingMode = 0x50;
        public const byte RelayChangeState = 0x51;
        public const byte ADDLEDSChangeState = 0x52;
        public const byte PROTECT_FW = 0x53;
        public const byte DELETE_SENSOR = 0x54;
        public const byte LED_TEST_ROUTINE = 0x55;
        public const byte SET_LEDS_BRIGHTNESS = 0x56;
        public const byte GET_INSTALLED_SENSORS = 0x57;
        public const byte BMP180_ResultReady = 0x58;
        public const byte SendIRCommand = 0x80;
        public const byte ChangeSettingsCommand = 0x81;
        public const byte GetSettingsCommand = 0x82;
        public const byte SaveFileDefineNameCommand = 0x83;
        public const byte SaveFileDefineName = 0x84;
        public const byte Perform_SW_Reset = 0x85;
        public const byte FileFinishCommand = 0x86;
        public const byte SendTextOverUSB_Command = 0x87;

        public const byte sAutoOffTimerValueS = 0x00;
        public const byte sPositionNum = 0x01;
        public const byte sSI7005_Int = 0x02;
        public const byte sBMP180_Int = 0x03;
        public const byte sLogToSDint = 0x04;
        public const byte sNormZeroAreaSize = 0x05;
        public const byte sNormFilterValue = 0x06;
        public const byte sCoarseZeroAreaSize = 0x07;
        public const byte sCoarseFilterValue = 0x08;

        public const UInt16 IR_HighDelayValue = 1180;
        public const UInt16 IR_LowDelayValue = 400;

        public static readonly string[] OneWireErrorList = { "Success",
                                                              "No echo: Devices on 1-Wire bus not found",
                                                             "Bus low: Bus voltage too low",
                                                             "Device busy: Conversion aborted",
                                                             "CRC: CRC mismatch." };

    }
}
