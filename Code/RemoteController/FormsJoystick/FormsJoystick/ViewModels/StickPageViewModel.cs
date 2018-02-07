using System;
using System.Windows.Input;
using Xamarin.Forms;

namespace FormsJoystick.ViewModels
{
    public class StickPageViewModel : BaseViewModel
    {
        const string DEV_NAME = "EMS01_1152";
        const int TIMER_INTERVAL = 40;
        const int VOL_REQ_COUNT = 5000 / 40;

        private int vol_req_counter = 1;

        public StickPageViewModel()
        {
            RegisterProcesser(0x05, ret => Voltage = ret / 10.0f);

            ConnectCommand = new Command(execute: async () =>
            {
                IsBusy = true;
                Message = "";
                (ConnectCommand as Command).ChangeCanExecute();

                try
                {
                    if (ConnectText == "Connect")
                    {
                        if (BTCom == null)
                        {
                            Message = "IBluetoothCom can't be initialized";
                            return;
                        }
                        if (!BTCom.FindDevice(DEV_NAME))
                        {
                            Message = "Can't find device";
                            return;
                        }
                        if (!await BTCom.ConnectAsync())
                        {
                            Message = "Can't connect device";
                            return;
                        }
                        Device.StartTimer(TimeSpan.FromMilliseconds(40), () =>
                        {
                            byte command = 0x00;
                            if (_joystickXposition > 30)
                            {
                                // turn right
                                command |= 1 << 0;
                            }
                            else if (_joystickXposition < -30)
                            {
                                // turn right
                                command |= 1 << 1;
                            }
                            if (_joystickYposition > 30)
                            {
                                // foreward
                                command |= 1 << 2;
                            }
                            else if (_joystickYposition < -30)
                            {
                                // foreward
                                command |= 1 << 3;
                            }

                            if (_command != "00000000" || command != 0x00)
                            {
                                Command = Convert.ToString(command, 2).PadLeft(8, '0');
                                if (!BTCom.Connected)
                                {
                                    ConnectText = "Connect";
                                    return false;
                                }
                                else
                                {
                                    PushNewCommand(new CmdPacket { Command = 0x01, Value = command });
                                }
                            }

                            if (BTCom.Connected && (vol_req_counter++) % VOL_REQ_COUNT == 0)
                            {
                                PushNewCommand(new CmdPacket { Command = 0x05, Value = 0xff });
                                vol_req_counter = 1;
                            }

                            return true;
                        });
                        ConnectText = "Disconnect";
                    }
                    else
                    {
                        BTCom.Close();
                        ConnectText = "Connect";
                    }
                }
                finally
                {
                    IsBusy = false;
                    (ConnectCommand as Command).ChangeCanExecute();
                }
            }, canExecute: () => !IsBusy);
        }

        private bool _isBusy;
        public bool IsBusy
        {
            get { return _isBusy; }
            set { _isBusy = value; NotifyPropertyChanged(nameof(IsBusy)); }
        }

        private string _command;
        public string Command
        {
            get { return _command; }
            set { _command = value; NotifyPropertyChanged(nameof(Command)); }
        }

        private string _message;
        public string Message
        {
            get { return _message; }
            set { _message = value; NotifyPropertyChanged(nameof(Message)); }
        }

        //Voltage
        private float _voltage;
        public float Voltage
        {
            get { return _voltage; }
            set { _voltage = value; NotifyPropertyChanged(nameof(Voltage)); }
        }

        private string _commandText = "Connect";
        public string ConnectText
        {
            get { return _commandText; }
            set { _commandText = value; NotifyPropertyChanged(nameof(ConnectText)); }
        }

        private int _joystickXposition;
        public int JoystickXposition
        {
            get { return _joystickXposition; }
            set { _joystickXposition = value; NotifyPropertyChanged(nameof(JoystickXposition)); }
        }

        private int _joystickYposition;
        public int JoystickYposition
        {
            get { return _joystickYposition; }
            set { _joystickYposition = value; NotifyPropertyChanged(nameof(JoystickYposition)); }
        }

        public ICommand ConnectCommand { protected set; get; }
    }
}
