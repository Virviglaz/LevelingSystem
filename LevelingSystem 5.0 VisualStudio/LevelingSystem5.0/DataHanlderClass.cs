using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace LevelingSystem5._0
{
    static public class DataHanlderClass
    {
        static UInt32 I2C_ErrorCounter = 0;
        static UInt16 I2C_ErrorShowCnt = 0;
        static public void RX_DataHandler(byte[] data, MainWindow MW)
        {
            if (data == null) return;

            TemperatureSensorHandling(data, MW);
            PressureSensorHandling(data, MW);
            ZeroBufferHandling(data, MW);
            SearchSensorsHandling(data, MW);
            InstalledSensorsHandling(data, MW);
            GetSettingCommandHandler(data, MW);
        }

        static private void TemperatureSensorHandling (byte[] data, MainWindow MW)
        {
            /* Temperature sensors handle */
            if (data[1] != Const.GetTempResultCommand) return;
            
            int SNum = (data[2] & 0xF0) >> 4;
            int SErr = (data[2] & 0x0F);
            if (SNum < 10)
            {
                if (SErr == 0)
                {
                    /* Procceed temperature sensors data */
                    float T = (float)((data[3] & 0x0F) * 0xFF + data[4]) / 16;
                    string Output = string.Format("Sensor {0}: {1:0.00}°C", SNum + 1, T);
                    if (MainWindow.isSensLogEnable)
                        PrintTextToOutput(MW.TextOutput, Output, false);
                    ChangeLabelText(MainWindow.SensLabel[SNum], Output);
                }
                else
                    /* Error print out */
                    PrintTextToOutput(MW.TextOutput, string.Format("Sensor {0}: Error {1:0} ", SNum + 1, SErr) + Const.OneWireErrorList[SErr]);
                return;
            }

            if (SNum == 10 || SNum == 11)
            {
                if (data[4] < 100)
                {
                    /* Procceed internal humidy sensors data */
                    string Output = string.Format("EXT{0} humidity: {1}%", SNum - 9,data[4]);
                    if (MainWindow.isSensLogEnable)
                        PrintTextToOutput(MW.TextOutput, Output);
                    ChangeLabelText(MW.INT_Humidy, Output);
                    Output = "";
                }
                else
                    /* Error print out */
                    PrintTextToOutput(MW.TextOutput, "External humidity sensor error\r\nData out of range!");
                return;
            }

            if (SNum == 12)
            {
                if (data[4] < 100)
                {
                    /* Procceed internal humidy sensors data */
                    string Output = string.Format("INT humidity: {0:0}%", data[4]);
                    if (MainWindow.isSensLogEnable)
                        PrintTextToOutput(MW.TextOutput, Output);
                    ChangeLabelText(MW.INT_Humidy, Output);
                    Output = "";
                }
                else
                    /* Error print out */
                    PrintTextToOutput(MW.TextOutput, "Internal humidity sensor error\r\nData out of range!");
                return;
            }
            
        }

        static private void PressureSensorHandling (byte[] data, MainWindow MW)
        {
            if (data[1] != Const.BMP180_ResultReady) return;
            int Pressure = data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5];
            int mmHg = data[10] << 8 | data[11];
            ChangeLabelText(MW.PressureLabel, string.Format("{0:0.00} kPa, {1} mmHg", (float)Pressure / 1000, mmHg));
        }

        static private void ZeroBufferHandling (byte[] data, MainWindow MW)
        {
            /* Zero buffer id packet */
            if (data[1] != 48) return;
            if (data[2] != 49) return;
            if (data[3] != 50) return;

            short X = (short)(data[31] << 8 | data[32]);
            short Y = (short)(data[33] << 8 | data[34]);

            ChangeLabelText(MW.X_Pos, string.Format("X = {0:0}", X));
            ChangeLabelText(MW.Y_Pos, string.Format("Y = {0:0}", Y));

            Brush CB = Brushes.Green;
            short TrigValue;

            if (MainWindow.RoughtModeEnabled)
            {
                TrigValue = 50;
                ChangeElipse(MW.BigCircleAim, Brushes.LightGreen, 0, 0, 100, 100, true);
            }
            else
            {
                TrigValue = 30;
                ChangeElipse(MW.BigCircleAim, Brushes.LightGreen, 0, 0, 70, 70, true);
            }
            if (Math.Abs(X) > TrigValue || Math.Abs(Y) > TrigValue) CB = Brushes.Red;

            ChangeElipse(MW.AimCircle, CB, X, Y, 30, 30, false);

            string DeviceTime = string.Format("{0:00}:{1:00}:{2:00} {3:00}/{4:00}/{5:0000}", data[25], data[26], data[27], data[23], data[22], data[21] + 2000);
            ChangeLabelText(MW.DeviceTimeLabel, DeviceTime);
            DeviceTime = "";

            TimeSpan Timeout = TimeSpan.FromSeconds(data[37] << 8 | data[38]);
            ChangeLabelText(MW.TimerLabel, "Auto off: " + Convert.ToString(Timeout));

            ChangeLabelText(MW.USB_VoltageLabel, String.Format("USB Voltage: {0:0.00}", (double)(data[18] << 8 | data[19]) * 2 * 3.3 / 4096));

            I2C_ErrorCounter = ConvertToU32(data, 41);
            I2C_ErrorShowCnt++;
            if (I2C_ErrorShowCnt > 1000)
            {
                PrintTextToOutput(MW.TextOutput, string.Format("I2C interface error counter: {0}", I2C_ErrorCounter));
                I2C_ErrorShowCnt = 0;
            }

            // Performs only one during startup
            if (MainWindow.UpdateZeroData == false) return;

            /* Update relays state info */
            for (byte cnt = 0; cnt != 8; cnt++)
            {
                if ((data[14] & (1 << cnt)) != 0)
                    ChangeCheckBoxState(MainWindow.RelayCheckBox[cnt], true);
                else
                    ChangeCheckBoxState(MainWindow.RelayCheckBox[cnt], false);
            }

            /* Update leveling status */
            if (data[35] != 0)
                ChangeCheckBoxState(MW.isLevelingOnBox, true);
            else
                ChangeCheckBoxState(MW.isLevelingOnBox, false);

            if (data[36] != 0)
                ChangeCheckBoxState(MW.isRoughModeBox, true);
            else
                ChangeCheckBoxState(MW.isRoughModeBox, false);

            if (data[39] != 0)
                ChangeCheckBoxState(MW.isTestLedOnBox, true);
            else
                ChangeCheckBoxState(MW.isTestLedOnBox, false);

            ChangeSliderValue(MW.LedBrighnessSlider, data[40]);

            MainWindow.UpdateZeroData = false;
        }

        static private void SearchSensorsHandling (byte[] data, MainWindow MW)
        {
            if (data[1] != Const.SearchSensorsCommand) return;
            string SerialNum = string.Format("{0}: ", data[2]) + ConvertHEXtoStr(data, 3, 9);
            if (!MainWindow.SensorsFoundList.Contains(SerialNum))
                MainWindow.SensorsFoundList.Add(SerialNum);
            SerialNum = "";
            if (data[2] == 1) //last one
            {
                PrintSensorList(MW, "Found sensors:");
                CopyListToComboBox(MW.SensListBox, MainWindow.SensorsFoundList, true);
            }
        }

        static private void GetSettingCommandHandler(byte[] data, MainWindow MW)
        {
            if (data[1] != Const.GetSettingsCommand) return;

            UInt32 AutoOffTimerValue = ConvertToU32(data, 2);
            DataHanlderClass.ChangeTextBoxText(MW.AutoOffTimerTextBox, string.Format("{0}", AutoOffTimerValue));

            byte PositionNumber = data[6];
            DataHanlderClass.ChangeTextBoxText(MW.PositionTextBox, string.Format("{0}", PositionNumber));

            double NormZeroSize = (double)(ConvertToU32(data, 13)) / 1000;
            DataHanlderClass.ChangeTextBoxText(MW.NormZeroSizeTextBox, string.Format("{0}", NormZeroSize));

            double CoarseModeZeroSize = (double)(ConvertToU32(data, 21)) / 1000;
            DataHanlderClass.ChangeTextBoxText(MW.CoarseModeZeroSizeTextBox, string.Format("{0}", CoarseModeZeroSize));

            double NormFilterValue = (double)(ConvertToU32(data, 17)) / 1000;
            DataHanlderClass.ChangeTextBoxText(MW.NormFilterValueTextBox, string.Format("{0}", NormFilterValue));

            double CoarseFilterValue = (double)(ConvertToU32(data, 25)) / 1000;
            DataHanlderClass.ChangeTextBoxText(MW.CoarseFilterValueTextBox, string.Format("{0}", CoarseFilterValue));

            UInt16 SI7005_ReadInterval = ConvertToU16(data, 7);
            DataHanlderClass.ChangeTextBoxText(MW.SI7005_ReadIntervalTextBox, string.Format("{0}", SI7005_ReadInterval));

            UInt16 BMP180_ReadInterval = ConvertToU16(data, 9);
            DataHanlderClass.ChangeTextBoxText(MW.BMP180_ReadIntervalTextBox, string.Format("{0}", BMP180_ReadInterval));

            UInt16 SDCardLogInterval = ConvertToU16(data, 11);
            DataHanlderClass.ChangeTextBoxText(MW.SDCardLogIntervalTextBox, string.Format("{0}", SDCardLogInterval));
        }

        static private void InstalledSensorsHandling(byte[] data, MainWindow MW)
        {
            if (data[1] != Const.GET_INSTALLED_SENSORS) return;
            string SerialNum = string.Format("{0}: ", data[3] + 1) + "28" + ConvertHEXtoStr(data, 4, 8);
            if (!MainWindow.SensorsInstalledList.Contains(SerialNum))
                MainWindow.SensorsInstalledList.Add(SerialNum);
            SerialNum = "";
            if (data[2] == 1) //last one
            {
                PrintSensorList(MW, "Installed sensors:");
                CopyListToComboBox(MW.SensListBox, MainWindow.SensorsInstalledList, true);
            }
        }

        static private void PrintSensorList (MainWindow MW, string Text)
        {
            byte cnt = 1;
            PrintTextToOutput(MW.TextOutput, Text);
            foreach (string Sensor in MainWindow.SensorsFoundList)
                PrintTextToOutput(MW.TextOutput, string.Format("{0:0}: ", cnt++) + Sensor);
        }

        #region Data Convert helpers
        static public string ConvertHEXtoStr (byte[] data, int StartFrom, int Size)
        {
            const string HEX_Var = "0123456789ABCDEF";
            string Result = "";
            for (int cnt = StartFrom; cnt != StartFrom + Size; cnt++)
                Result += Convert.ToString(HEX_Var[(data[cnt] >> 4)]) + Convert.ToString(HEX_Var[(data[cnt] & 0x0F)]);
            return Result;
        }

        static public byte[] ConvertStrToHEX(string Text, byte[] Data, byte Size, byte StartFrom = 0)
        {
            const string HEX_Var = "0123456789ABCDEF";
            byte cnt2 = 0;
            for (byte cnt = StartFrom; cnt != StartFrom + Size / 2; cnt++)
            {
                Data[cnt] = (byte)(HEX_Var.IndexOf(Text[cnt2++]) << 4);
                Data[cnt] |= (byte)HEX_Var.IndexOf(Text[cnt2++]);
            }         
            return Data;
        }

        static public UInt32 ConvertToU32 (byte[] buf, UInt16 StartPos)
        {
            return (UInt32)(buf[StartPos] << 24 | buf[StartPos+1] << 16 | buf[StartPos+2] << 8 | buf[StartPos+3]);
        }

        static public UInt16 ConvertToU16 (byte[] buf, UInt16 StartPos)
        {
            return (UInt16)(buf[StartPos] << 8 | buf[StartPos + 1]);
        }
        #endregion

        #region DeligateSync
        static public void ChangeRadioButtonState(RadioButton RB, bool State, string Text)
        {
            if (!RB.Dispatcher.CheckAccess())
            {
                RB.Dispatcher.Invoke(DispatcherPriority.Background, new Action(() =>
                {
                    RB.IsChecked = State;
                    RB.Content = Text;
                }));
                Dispatcher.CurrentDispatcher.InvokeShutdown();
            }
            else
            {
                RB.IsChecked = State;
                RB.Content = Text;
            }
            
        }
        static private void ChangeLabelText(Label LB, string text)
        {
            if (!LB.Dispatcher.CheckAccess())
            {
                LB.Dispatcher.Invoke(DispatcherPriority.Background, new Action(() =>
                {
                    LB.Content = text;
                }));
                Dispatcher.CurrentDispatcher.InvokeShutdown();
            }
            else
                LB.Content = text;
        }

        static public void PrintTextToOutput(ScrollViewer TB, string text, bool Erase = false)
        {
            if (!TB.Dispatcher.CheckAccess())
            {
                TB.Dispatcher.Invoke(DispatcherPriority.Background, new Action(() =>
                {
                    if (Erase)
                        TB.Content = "";
                    else
                    {
                        TB.Content += text + Environment.NewLine;
                        if (MainWindow.AutoScrollEnabled)
                            TB.ScrollToEnd();
                    }
                }));
                Dispatcher.CurrentDispatcher.InvokeShutdown();
            }
            else
            {
                if (Erase)
                    TB.Content = "";
                else
                {
                    TB.Content += text + Environment.NewLine;
                    if (MainWindow.AutoScrollEnabled)
                        TB.ScrollToEnd();
                }
            }
        }

        static public void ChangeCheckBoxState(CheckBox CHB, bool State)
        {
            if (!CHB.Dispatcher.CheckAccess())
            {
                CHB.Dispatcher.Invoke(DispatcherPriority.Background, new Action(() =>
                {
                    CHB.IsChecked = State;
                }));
                Dispatcher.CurrentDispatcher.InvokeShutdown();
            }
            else
                CHB.IsChecked = State;
        }

        static public void ChangeElipse (Ellipse E, Brush B, Double X, Double Y, double W, double H, bool DontMove)
        {
            if (!E.Dispatcher.CheckAccess())
            {
                E.Dispatcher.Invoke(DispatcherPriority.Background, new Action(() =>
                {
                    E.Fill = B;
                    E.Width = W;
                    E.Height = H;
                    if (DontMove) return;
                    if (X < -200) X = -200;
                    if (X > 200) X = 200;
                    if (Y < -200) Y = -200;
                    if (Y > 200) Y = 200;
                    E.Margin = new Thickness(MainWindow.InitAimPos.Left + X, MainWindow.InitAimPos.Top + Y, 0, 0);
                    E.UpdateLayout();
                }));
                Dispatcher.CurrentDispatcher.InvokeShutdown();
            }
            else
            {
                E.Fill = B;
                E.Width = W;
                E.Height = H;
                if (DontMove) return;
                if (X < -200) X = -200;
                if (X > 200) X = 200;
                if (Y < -200) Y = -200;
                if (Y > 200) Y = 200;
                E.Margin = new Thickness(MainWindow.InitAimPos.Left + X, MainWindow.InitAimPos.Top + Y, 0, 0);
                E.UpdateLayout();
            }
        }

        static public void CopyListToComboBox (ComboBox  CMB, List<string> DataList, bool ErasePrevios = false)
        {
            if (!CMB.Dispatcher.CheckAccess())
            {
                CMB.Dispatcher.Invoke(DispatcherPriority.Background, new Action(() =>
                {
                    if (ErasePrevios)
                        CMB.Items.Clear();
                    for (int cnt = DataList.Count; cnt != 0; cnt--)
                    {
                        ComboBoxItem CMB_Item = new ComboBoxItem();
                        CMB_Item.Content = DataList[cnt - 1];
                        CMB.Items.Add(CMB_Item);
                    }
                    CMB.SelectedIndex = 0;
                }));
                Dispatcher.CurrentDispatcher.InvokeShutdown();
            }
            else
            {
                if (ErasePrevios)
                    CMB.Items.Clear();
                for (int cnt = DataList.Count; cnt != 0; cnt--)
                {
                    ComboBoxItem CMB_Item = new ComboBoxItem();
                    CMB_Item.Content = DataList[cnt - 1];
                    CMB.Items.Add(CMB_Item);
                }
            }
        }

        static public void ChangeSliderValue(Slider SL, int Value)
        {
            if (!SL.Dispatcher.CheckAccess())
            {
                SL.Dispatcher.Invoke(DispatcherPriority.Background, new Action(() =>
                {
                    SL.Value = Value;
                }));
                Dispatcher.CurrentDispatcher.InvokeShutdown();
            }
            else
                SL.Value = Value;
        }

        static public void ChangeTextBoxText(TextBox TB, string Value)
        {
            if (!TB.Dispatcher.CheckAccess())
            {
                TB.Dispatcher.Invoke(DispatcherPriority.Background, new Action(() =>
                {
                    TB.Text = Value;
                }));
                Dispatcher.CurrentDispatcher.InvokeShutdown();
            }
            else
                TB.Text = Value;
        }

        static public void ChangeProgressBarValue (ProgressBar PG, double Value)
        {
            if (!PG.Dispatcher.CheckAccess())
            {
                PG.Dispatcher.Invoke(DispatcherPriority.Background, new Action(() =>
                {
                    PG.Value = Value;
                }));
                Dispatcher.CurrentDispatcher.InvokeShutdown();
            }
            else
                PG.Value = Value;
        }
        #endregion
    }
}
