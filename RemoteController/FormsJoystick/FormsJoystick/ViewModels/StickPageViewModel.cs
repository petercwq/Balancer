using System;
using System.Windows.Input;
using Xamarin.Forms;

namespace FormsJoystick.ViewModels
{
    public class StickPageViewModel : BaseViewModel
    {
        const int TIMER_INTERVAL = 40;
        const int VOL_REQ_COUNT = 5000 / 40;

        private int vol_req_counter = 1;

        public StickPageViewModel()
        {
            AttachCommandAction(Commands.GetBattery, 
                                ret => 
                                Voltage = ret.ReadFloatArg());

            ConnectCommand = new Command(() =>
            {
                IsBusy = true;
                Message = "";
                (ConnectCommand as Command).ChangeCanExecute();

                try
                {
                    if (ConnectText == "Connect" && Connect())
                    {
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
                                QueueCommand(new CommandMessenger.SendCommand((int)Commands.Move, (short)command));
                            }

                            if ((vol_req_counter++) % VOL_REQ_COUNT == 0)
                            {
                                QueueCommand(new CommandMessenger.SendCommand((int)Commands.GetBattery));
                                vol_req_counter = 1;
                            }

                            return true;
                        });
                        ConnectText = "Disconnect";
                    }
                    else
                    {
                        Disconnect();
                        ConnectText = "Connect";
                    }
                }
                finally
                {
                    IsBusy = false;
                    (ConnectCommand as Command).ChangeCanExecute();
                }
            }, () => !IsBusy);
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
