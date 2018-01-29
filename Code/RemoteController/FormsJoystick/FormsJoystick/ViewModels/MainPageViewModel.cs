using System;
using System.ComponentModel;
using System.Windows.Input;
using FormsJoystick.Communication;
using Xamarin.Forms;

namespace FormsJoystick.ViewModels
{
    public class MainPageViewModel : ViewModelBase, INotifyPropertyChanged
    {
        IBluetoothCom BTCom = DependencyService.Get<IBluetoothCom>();

        public override void OnAppearing()
        {
            base.OnAppearing();
        }

        public override void OnDisappearing()
        {
            base.OnDisappearing();
            if (BTCom != null)
            {
                BTCom.Close();
                ConnectText = "Connect";
            }
        }

        public MainPageViewModel()
        {
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
                        if (!BTCom.FindDevice("EMS01_1152"))
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
                            command |= 0b00000010;
                            }
                            else if (_joystickXposition < -30)
                            {
                            // turn right
                            command |= 0b00000001;
                            }
                            if (_joystickYposition > 30)
                            {
                            // foreward
                            command |= 0b00000100;
                            }
                            else if (_joystickYposition < -30)
                            {
                            // foreward
                            command |= 0b00001000;
                            }
                            if (BTCom.SendData(command))
                            {
                                Command = Convert.ToString(command, 2).PadLeft(8, '0');
                                return true;
                            }
                            else
                            {
                                Message = "Send command failed";
                                BTCom.Close();
                                ConnectText = "Connect";
                                return false;
                            }
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

        public event PropertyChangedEventHandler PropertyChanged;

        void NotifyPropertyChanged(string name)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
        }
    }
}
