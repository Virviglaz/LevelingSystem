using HidSharp;
using Microsoft.Win32;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Media;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Threading;

namespace LevelingSystem5._0
{
    /// <summary>
    /// Логика взаимодействия для MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        static HidDeviceLoader loader = new HidDeviceLoader();
        static HidDevice LevelingSystemDevice = null; // = loader.GetDevices(2003, 7003).FirstOrDefault();
        static HidStream USB_Stream;

        static Mutex HID_Mutex = new Mutex(); 
        static byte[] TxData = new byte[64];
        static byte[] ZrBuff = new byte[65];
        static Thread HID_CheckConnThread = new Thread(HID_CheckConnectionThread);
        static Thread HID_RX_DataHandlerThread = new Thread(HID_RX_DataHandler);
        static Thread FW_ProgressBarThread = new Thread(FW_ProgressBarHadler);
        static Queue<byte[]> RxQueue = new Queue<byte[]>();
        static Queue<byte[]> TXQueue = new Queue<byte[]>();
        static MainWindow MW;
        static public List<Label> SensLabel = new List<Label>();
        static public bool isSensLogEnable = true;
        static public Thickness InitAimPos;
        static public bool RoughtModeEnabled = false;
        static public bool AutoScrollEnabled = true;
        static public List<string> SensorsFoundList = new List<string>();
        static public List<string> SensorsInstalledList = new List<string>();
        static public CheckBox[] RelayCheckBox = new CheckBox[8];
        static public bool UpdateZeroData = true;

        public MainWindow()
        {
            InitializeComponent();
            MW = MainWindow1;
            SensLabel.Add(MW.SensorValue0);
            SensLabel.Add(MW.SensorValue1);
            SensLabel.Add(MW.SensorValue2);
            SensLabel.Add(MW.SensorValue3);
            SensLabel.Add(MW.SensorValue4);
            SensLabel.Add(MW.SensorValue5);
            SensLabel.Add(MW.SensorValue6);
            SensLabel.Add(MW.SensorValue7);
            SensLabel.Add(MW.SensorValue8);
            SensLabel.Add(MW.SensorValue9);
            RelayCheckBox[0] = Relay1CheckBox;
            RelayCheckBox[1] = Relay2CheckBox;
            RelayCheckBox[2] = Relay3CheckBox;
            RelayCheckBox[3] = Relay4CheckBox;
            RelayCheckBox[4] = Relay5CheckBox;
            RelayCheckBox[5] = Relay6CheckBox;
            RelayCheckBox[6] = Relay7CheckBox;
            RelayCheckBox[7] = Relay8CheckBox;

            RxQueue.Clear();
            HID_CheckConnThread.Start();
            HID_RX_DataHandlerThread.Start();
            FW_ProgressBarThread.Start();
            InitAimPos = AimCircle.Margin;
        }

        #region USB Transfer Handler
        private static bool HID_Write (byte[] data)
        {
            byte[] txdata = new byte[65];
            txdata[0] = 0;
            data.CopyTo(txdata, 1);
            TXQueue.Enqueue(txdata);
            return true;
        }

        private static void HID_CheckConnectionThread()
        {
            byte[] RxBuff = new byte[65];
            byte[] TxBuff = new byte[65];

            Disconnected:
            while (LevelingSystemDevice == null)
            {
                Thread.Sleep(100);
                LevelingSystemDevice = loader.GetDevices(2003, 7003).FirstOrDefault(d => d.MaxInputReportLength == 65);
            }
            DataHanlderClass.ChangeRadioButtonState(MW.ConnectedInd, true, "Connected");
            UpdateZeroData = true;
            if (!LevelingSystemDevice.TryOpen(out USB_Stream))
            {
                MessageBox.Show("Failed to connect to device", "Error!", MessageBoxButton.OK, MessageBoxImage.Error);
                Environment.Exit(2);
            }
            using (USB_Stream)
            {
                while(true)
                {
                    try
                    {
                        RxBuff = USB_Stream.Read();
                        if (TXQueue.Count > 0)
                        {
                            TxBuff = TXQueue.Dequeue();
                            USB_Stream.Write(TxBuff);
                        }
                    }
                    catch
                    {
                        USB_Stream.Close();
                        LevelingSystemDevice = null;
                        DataHanlderClass.ChangeRadioButtonState(MW.ConnectedInd, false, "Disconnected");
                        goto Disconnected;
                    }
                    if (RxBuff != null)
                        RxQueue.Enqueue(RxBuff);

                    Thread.Sleep(1);
                }
            }
        }
      
        private static void HID_RX_DataHandler()
        {
            byte[] Buff = new byte[64];
            while (true)
            {
                Thread.Sleep(2);
                if (RxQueue.Count > 0)
                {
                    Buff = RxQueue.Dequeue();
                    DataHanlderClass.RX_DataHandler(Buff, MW);
                }
            }
        }
        #endregion
        private void MainWindow_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (LevelingSystemDevice != null)
                USB_Stream.Close();
            HID_CheckConnThread.Abort();
            HID_RX_DataHandlerThread.Abort();
            FW_ProgressBarThread.Resume();
            FW_ProgressBarThread.Abort();
        }

        #region ElementsHandlers
        private void SensLogEnable_Checked(object sender, RoutedEventArgs e)
        {
            isSensLogEnable = true;
        }

        private void SensLogEnable_Unchecked(object sender, RoutedEventArgs e)
        {
            isSensLogEnable = false;
        }

        private void isLevelingOnBox_Checked(object sender, RoutedEventArgs e)
        {
            TxData[0] = Const.TurnOnLevelingCommand;
            HID_Write(TxData);
        }

        private void isLevelingOnBox_Unchecked(object sender, RoutedEventArgs e)
        {
            TxData[0] = Const.TurnOffLevelingCommand;
            HID_Write(TxData);
        }

        private void isRoughModeBox_Checked(object sender, RoutedEventArgs e)
        {
            TxData[0] = Const.EnableFloatingMode;
            HID_Write(TxData);
            RoughtModeEnabled = true;
        }

        private void isRoughModeBox_Unchecked(object sender, RoutedEventArgs e)
        {
            TxData[0] = Const.DisableFloatingMode;
            HID_Write(TxData);
            RoughtModeEnabled = false;
        }

        private void isTestLedOnBox_Checked(object sender, RoutedEventArgs e)
        {
            TxData[0] = Const.LED_TEST_ROUTINE;
            TxData[1] = 1;
            HID_Write(TxData);
        }

        private void isTestLedOnBox_Unchecked(object sender, RoutedEventArgs e)
        {
            TxData[0] = Const.LED_TEST_ROUTINE;
            TxData[1] = 0;
            HID_Write(TxData);
        }
        #endregion

        private void button_Click(object sender, RoutedEventArgs e)
        {
            if (MessageBox.Show("Saved callibration data will be lost!", "Update callibration data?", System.Windows.MessageBoxButton.YesNo) == System.Windows.MessageBoxResult.Yes)
            {
                TxData[0] = Const.CallibrateZeroCommand;
                HID_Write(TxData);
            }
        }

        private void Button_Click_1(object sender, RoutedEventArgs e)
        {
            DataHanlderClass.PrintTextToOutput(MW.TextOutput, "", true);
        }

        #region Autoscroll
        private void AutoscrollCheckBox_Unchecked(object sender, RoutedEventArgs e)
        {
            AutoScrollEnabled = false;
        }

        private void AutoscrollCheckBox_Checked(object sender, RoutedEventArgs e)
        {
            AutoScrollEnabled = true;
        }
        #endregion
        private void SearchSensorsButton_Click(object sender, RoutedEventArgs e)
        {
            TxData[0] = Const.SearchSensorsCommand;
            HID_Write(TxData);
            SensorsFoundList.Clear();
            DeleteSensorButton.IsEnabled = false;
        }

        private void InstalledSensorsButton_Click(object sender, RoutedEventArgs e)
        {
            TxData[0] = Const.GET_INSTALLED_SENSORS;
            HID_Write(TxData);
            SensorsInstalledList.Clear();
            DeleteSensorButton.IsEnabled = true;
        }

        private void DeleteSensorButton_Click(object sender, RoutedEventArgs e)
        {
            TxData[0] = Const.DELETE_SENSOR;
            if (SensorsInstalledList.Count == 0) return;
            byte CurrentSensorNum =  (byte)(SensorsInstalledList.Count - SensListBox.SelectedIndex - 1);
            string CurrentSensorSN = SensorsInstalledList[CurrentSensorNum];
            if (CurrentSensorSN.StartsWith("10"))
                CurrentSensorNum = byte.Parse(CurrentSensorSN.Substring(0, 2));
            else
                CurrentSensorNum = byte.Parse(CurrentSensorSN.Substring(0, 1));
            TxData[1] = CurrentSensorNum;
            TxData[1]--;
            DataHanlderClass.PrintTextToOutput(TextOutput, string.Format("Sensor {0:0} ", CurrentSensorNum) + " deleted!");
            SensorsInstalledList.Remove(CurrentSensorSN);
            DataHanlderClass.CopyListToComboBox(SensListBox, MainWindow.SensorsInstalledList, true);
            SensLabel[CurrentSensorNum - 1].Content = "Deleted";
            HID_Write(TxData);
        }

        private void DeviceTimeLabel_MouseDoubleClick(object sender, MouseButtonEventArgs e)
        {
            TxData[0] = Const.SetTime;
            TxData[1] = (byte)DateTime.Now.Hour;
            TxData[2] = (byte)DateTime.Now.Minute;
            TxData[3] = (byte)DateTime.Now.Second;
            TxData[4] = (byte)(DateTime.Today.Year - 2000);
            TxData[5] = (byte)DateTime.Today.Month;
            TxData[6] = (byte)DateTime.Today.Day;
            TxData[7] = (byte)DateTime.Now.DayOfWeek;
            HID_Write(TxData);
            DataHanlderClass.PrintTextToOutput(TextOutput, "Time on device updated!");
        }

        #region StoreSensorHandlers
        private void StoreSensorToEEPRO(byte SensNum)
        {
            if (SensListBox.SelectedIndex == -1 || SensorsFoundList.Count <= SensListBox.SelectedIndex) return;
            string ChoosedSensor = SensorsFoundList[SensListBox.SelectedIndex].Substring(3); //unsafe code!
            DataHanlderClass.PrintTextToOutput(TextOutput, string.Format("Sensor {0:0} ", SensNum + 1) + ChoosedSensor + " installed!");
            TxData = DataHanlderClass.ConvertStrToHEX(ChoosedSensor, TxData,(byte)ChoosedSensor.Length, 2);
            TxData[0] = Const.InstallNewSensor;
            TxData[1] = SensNum;
            HID_Write(TxData);
        }
        private void SensorValue0_MouseDoubleClick(object sender, MouseButtonEventArgs e)
        {
            StoreSensorToEEPRO(0);
        }

        private void SensorValue1_MouseDoubleClick(object sender, MouseButtonEventArgs e)
        {
            StoreSensorToEEPRO(1);
        }

        private void SensorValue2_MouseDoubleClick(object sender, MouseButtonEventArgs e)
        {
            StoreSensorToEEPRO(2);
        }

        private void SensorValue3_MouseDoubleClick(object sender, MouseButtonEventArgs e)
        {
            StoreSensorToEEPRO(3);
        }

        private void SensorValue4_MouseDoubleClick(object sender, MouseButtonEventArgs e)
        {
            StoreSensorToEEPRO(4);
        }

        private void SensorValue5_MouseDoubleClick(object sender, MouseButtonEventArgs e)
        {
            StoreSensorToEEPRO(5);
        }

        private void SensorValue6_MouseDoubleClick(object sender, MouseButtonEventArgs e)
        {
            StoreSensorToEEPRO(6);
        }

        private void SensorValue7_MouseDoubleClick(object sender, MouseButtonEventArgs e)
        {
            StoreSensorToEEPRO(7);
        }

        private void SensorValue8_MouseDoubleClick(object sender, MouseButtonEventArgs e)
        {
            StoreSensorToEEPRO(8);
        }

        private void SensorValue9_MouseDoubleClick(object sender, MouseButtonEventArgs e)
        {
            StoreSensorToEEPRO(9);
        }
        #endregion

        private void slider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            TxData[0] = Const.SET_LEDS_BRIGHTNESS;
            TxData[1] = (byte)LedBrighnessSlider.Value;
            HID_Write(TxData);
        }

        private void AutoOffTimerTextBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key != Key.Return) return;
            TxData[0] = Const.ChangeSettingsCommand;
            TxData[1] = Const.sAutoOffTimerValueS;
            UInt32 TimerValue;
            if (UInt32.TryParse(AutoOffTimerTextBox.Text, out TimerValue) == false)
            {
                DataHanlderClass.PrintTextToOutput(TextOutput, "Change timer value failed! Data incorrect.");
                return;
            }
            if (TimerValue > Int32.MaxValue) return;
            TxData[7] = (byte)((TimerValue & 0xFF000000) >> 24);
            TxData[6] = (byte)((TimerValue & 0x00FF0000) >> 16);
            TxData[5] = (byte)((TimerValue & 0x0000FF00) >> 8);
            TxData[4] = (byte)(TimerValue  & 0x000000FF);
            HID_Write(TxData);
            DataHanlderClass.PrintTextToOutput(TextOutput, "Leveling auto off timer changed.");
        }

        private void PositionTextBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key != Key.Return) return;
            TxData[0] = Const.ChangeSettingsCommand;
            TxData[1] = Const.sPositionNum;
            if (byte.TryParse(PositionTextBox.Text, out TxData[4]) == false)
            {
                DataHanlderClass.PrintTextToOutput(TextOutput, "Change timer value failed! Data incorrect.");
                return;
            }
            if (TxData[4] > 16) return;
            HID_Write(TxData);
            DataHanlderClass.PrintTextToOutput(TextOutput, "Leveling position changed.");
        }

        private void SI7005_ReadIntervalTextBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key != Key.Return) return;
            TxData[0] = Const.ChangeSettingsCommand;
            TxData[1] = Const.sSI7005_Int;
            UInt16 SI7005_Int;
            if (UInt16.TryParse(SI7005_ReadIntervalTextBox.Text, out SI7005_Int) == false)
            {
                DataHanlderClass.PrintTextToOutput(TextOutput, "Change interval value failed! Data incorrect.");
                return;
            }
            TxData[5] = (byte)((SI7005_Int & 0x0000FF00) >> 8);
            TxData[4] = (byte)(SI7005_Int & 0x000000FF);
            HID_Write(TxData);
            DataHanlderClass.PrintTextToOutput(TextOutput, "SI7005 read interval changed.");
        }

        private void BMP180_ReadIntervalTextBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key != Key.Return) return;
            TxData[0] = Const.ChangeSettingsCommand;
            TxData[1] = Const.sBMP180_Int;
            UInt16 BMP180_Int;
            if (UInt16.TryParse(BMP180_ReadIntervalTextBox.Text, out BMP180_Int) == false)
            {
                DataHanlderClass.PrintTextToOutput(TextOutput, "Change interval value failed! Data incorrect.");
                return;
            }
            TxData[5] = (byte)((BMP180_Int & 0x0000FF00) >> 8);
            TxData[4] = (byte)(BMP180_Int & 0x000000FF);
            HID_Write(TxData);
            DataHanlderClass.PrintTextToOutput(TextOutput, "BMP180 read interval changed.");
        }

        private void SDCardLogIntervalTextBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key != Key.Return) return;
            TxData[0] = Const.ChangeSettingsCommand;
            TxData[1] = Const.sLogToSDint;
            UInt16 LogToSDint;
            if (UInt16.TryParse(SDCardLogIntervalTextBox.Text, out LogToSDint) == false)
            {
                DataHanlderClass.PrintTextToOutput(TextOutput, "Change interval value failed! Data incorrect.");
                return;
            }
            TxData[5] = (byte)((LogToSDint & 0x0000FF00) >> 8);
            TxData[4] = (byte)(LogToSDint & 0x000000FF);
            HID_Write(TxData);
            DataHanlderClass.PrintTextToOutput(TextOutput, "SD logging interval changed.");
        }

        private void RelayValueChanged(object sender, RoutedEventArgs e)
        {
            TxData[0] = Const.RelayChangeState;
            TxData[1] = 0;
            for (byte cnt = 0; cnt != 8; cnt++)
                if (RelayCheckBox[cnt].IsChecked == true)
                    TxData[1] |= (byte)(1 << cnt);
            HID_Write(TxData);
        }

        private void GetSettingsButton_Click(object sender, RoutedEventArgs e)
        {
            TxData[0] = Const.GetSettingsCommand;
            HID_Write(TxData);
        }

        private void NormZeroSizeTextBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key != Key.Return) return;
            TxData[0] = Const.ChangeSettingsCommand;
            TxData[1] = Const.sNormZeroAreaSize;
            double NormZeroAreaSize;
            if (double.TryParse(NormZeroSizeTextBox.Text, out NormZeroAreaSize) == false)
            {
                DataHanlderClass.PrintTextToOutput(TextOutput, "Change area size failed! Data incorrect.");
                return;
            }

            UInt32 u32_NormZeroAreaSize = (UInt32)(NormZeroAreaSize * 1000);
            TxData[7] = (byte)((u32_NormZeroAreaSize & 0xFF000000) >> 24);
            TxData[6] = (byte)((u32_NormZeroAreaSize & 0x00FF0000) >> 16);
            TxData[5] = (byte)((u32_NormZeroAreaSize & 0x0000FF00) >> 8);
            TxData[4] = (byte)(u32_NormZeroAreaSize & 0x000000FF);
            HID_Write(TxData);
            DataHanlderClass.PrintTextToOutput(TextOutput, "Normal area zone size changed.");
        }

        private void CoarseModeZeroSizeTextBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key != Key.Return) return;
            TxData[0] = Const.ChangeSettingsCommand;
            TxData[1] = Const.sCoarseZeroAreaSize;
            double CoarseZeroAreaSize;
            if (double.TryParse(CoarseModeZeroSizeTextBox.Text, out CoarseZeroAreaSize) == false)
            {
                DataHanlderClass.PrintTextToOutput(TextOutput, "Change area size failed! Data incorrect.");
                return;
            }

            UInt32 u32_CoarseZeroAreaSize = (UInt32)(CoarseZeroAreaSize * 1000);
            TxData[7] = (byte)((u32_CoarseZeroAreaSize & 0xFF000000) >> 24);
            TxData[6] = (byte)((u32_CoarseZeroAreaSize & 0x00FF0000) >> 16);
            TxData[5] = (byte)((u32_CoarseZeroAreaSize & 0x0000FF00) >> 8);
            TxData[4] = (byte)(u32_CoarseZeroAreaSize & 0x000000FF);
            HID_Write(TxData);
            DataHanlderClass.PrintTextToOutput(TextOutput, "Coarse area zone size changed.");
        }

        private void NormFilterValueTextBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key != Key.Return) return;
            TxData[0] = Const.ChangeSettingsCommand;
            TxData[1] = Const.sNormFilterValue;
            double NormFilterValue;
            if (double.TryParse(NormFilterValueTextBox.Text, out NormFilterValue) == false)
            {
                DataHanlderClass.PrintTextToOutput(TextOutput, "Filter value didn't changed! Data incorrect.");
                return;
            }

            UInt32 u32_NormFilterValue = (UInt32)(NormFilterValue * 1000);
            TxData[7] = (byte)((u32_NormFilterValue & 0xFF000000) >> 24);
            TxData[6] = (byte)((u32_NormFilterValue & 0x00FF0000) >> 16);
            TxData[5] = (byte)((u32_NormFilterValue & 0x0000FF00) >> 8);
            TxData[4] = (byte)(u32_NormFilterValue & 0x000000FF);
            HID_Write(TxData);
            DataHanlderClass.PrintTextToOutput(TextOutput, "Normal mode filter value changed.");
        }

        private void CoarseFilterValueTextBox_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key != Key.Return) return;
            TxData[0] = Const.ChangeSettingsCommand;
            TxData[1] = Const.sCoarseFilterValue;
            double CoarseFilterValue;
            if (double.TryParse(CoarseFilterValueTextBox.Text, out CoarseFilterValue) == false)
            {
                DataHanlderClass.PrintTextToOutput(TextOutput, "Filter value didn't changed! Data incorrect.");
                return;
            }

            UInt32 u32_CoarseFilterValue = (UInt32)(CoarseFilterValue * 1000);
            TxData[7] = (byte)((u32_CoarseFilterValue & 0xFF000000) >> 24);
            TxData[6] = (byte)((u32_CoarseFilterValue & 0x00FF0000) >> 16);
            TxData[5] = (byte)((u32_CoarseFilterValue & 0x0000FF00) >> 8);
            TxData[4] = (byte)(u32_CoarseFilterValue & 0x000000FF);
            HID_Write(TxData);
            DataHanlderClass.PrintTextToOutput(TextOutput, "Coarse mode filter value changed.");
        }

        private void TestIR_BTN1_Click(object sender, RoutedEventArgs e)
        {
            TxData[0] = Const.SendIRCommand;
            TxData[1] = (0xFF00 & Const.IR_HighDelayValue) >> 8;
            TxData[2] = (0x00FF & Const.IR_HighDelayValue);
            TxData[3] = (0xFF00 & Const.IR_LowDelayValue) >> 8;
            TxData[4] = (0x00FF & Const.IR_LowDelayValue);
            TxData[5] = 11;
            TxData[6] = 0x52;
            TxData[7] = 0xAE;
            TxData[8] = 0xC3;
            TxData[9] = 0x26;
            TxData[10] = 0xD9;
            TxData[11] = 0xBB;
            TxData[12] = 0x44;
            TxData[13] = 0x87;
            TxData[14] = 0x78;
            TxData[15] = 0x26;
            TxData[16] = 0xD9;
            HID_Write(TxData);
        }

        private void UploadFW_Button_Click(object sender, RoutedEventArgs e)
        {
            byte[] TxFile = new byte[64];
            OpenFileDialog FW_FileDialog = new OpenFileDialog();
            //FW_FileDialog.Filter = "All files (*.hex)|*.HEX";
            FW_FileDialog.ShowDialog();
            string FW_FilePath = FW_FileDialog.FileName;
            if (FW_FilePath == "") return;
            byte[] File = System.IO.File.ReadAllBytes(FW_FilePath);

            /* Send Filename */
            TxFile[0] = Const.SaveFileDefineName;
            TxFile[1] = (byte)FW_FileDialog.SafeFileName.Count();
            for (int i = 0; i < FW_FileDialog.SafeFileName.Count(); i++)
                TxFile[i + 2] = (byte)FW_FileDialog.SafeFileName[i];
            HID_Write(TxFile);
            DataHanlderClass.PrintTextToOutput(TextOutput, string.Format("Uploading file {0} size {1:N0} bytes", FW_FileDialog.SafeFileName, File.Count()));

            TxFile[0] = Const.SaveFileDefineNameCommand;
            byte PageSize = Const.FilePageSize;
            UInt32 CurrentPos = 0;
           
            while (CurrentPos < File.Count())
            {
                if (CurrentPos > File.Count() - PageSize)
                    PageSize = (byte)(File.Count() - CurrentPos);

                /* Copy file content */
                for (int i = 0; i < PageSize; i++)
                    TxFile[i + 2] = (byte)File[CurrentPos + i];
                TxFile[1] = PageSize;
                CurrentPos += PageSize;

                HID_Write(TxFile);             
            }

            TxFile[0] = Const.FileFinishCommand;
            HID_Write(TxFile);

            MW.BootLoaderProgressBar.Value = MW.BootLoaderProgressBar.Maximum = TXQueue.Count();
            FW_ProgressBarThread.Resume();
        }

        private static void FW_ProgressBarHadler ()
        {
            while (true)
            {
                Thread.CurrentThread.Suspend();
                while (TXQueue.Count > 0)
                {
                    MW.BootLoaderProgressBar.Dispatcher.Invoke(DispatcherPriority.Background, new Action(() =>
                    {
                        MW.BootLoaderProgressBar.Value = MW.BootLoaderProgressBar.Maximum - TXQueue.Count;
                    }));
                    Dispatcher.CurrentDispatcher.InvokeShutdown();
                    Thread.Sleep(100);
                }
                MessageBox.Show("Done!", "File upload", MessageBoxButton.OK, MessageBoxImage.Information);               
            }
        }

        private void ResetBtn_Click(object sender, RoutedEventArgs e)
        {
            TxData[0] = Const.Perform_SW_Reset;
            HID_Write(TxData);
        }
    }
}
