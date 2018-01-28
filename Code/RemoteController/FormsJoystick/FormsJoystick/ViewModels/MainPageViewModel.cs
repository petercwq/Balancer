using System;
using System.ComponentModel;
using System.Threading.Tasks;
using FormsJoystick.Communication;
using Xamarin.Forms;

namespace FormsJoystick.ViewModels
{
    public class MainPageViewModel : ViewModelBase, INotifyPropertyChanged
    {
        // turn left     B00000001
        // turn right    B00000010
        // forward       B00000100
        // backward      B00001000

        IBluetoothCom BTCom  = DependencyService.Get<IBluetoothCom>();

        public override void OnAppearing()
        {
            base.OnAppearing();
        }

        public override void OnDisappearing()
        {
            base.OnDisappearing();
            if(BTCom!=null)
            {
                BTCom.Close();
            }
        }

        public MainPageViewModel()
        {
            Task.Run(() => {
               if (BTCom == null)
               {
                   Message = "IBluetoothCom can't be initialized";
                   return false;
               }
               if (!BTCom.FindDevice("EMS01_1152"))
               {
                   Message = "Can't find device";
                   return false;
               }
               if (!BTCom.Connect())
               {
                   Message = "Can't connect device";
                   return false;
               }
               return true;
           }).ContinueWith(t => {
               if (t.Result)
               {
                   Device.StartTimer(TimeSpan.FromMilliseconds(50), () => {
                       byte command = 0x00;
                       if (_joystickXposition > 20)
                       {
                           // turn right
                           command |= 0b00000010;
                       }
                       else if (_joystickXposition < -20)
                       {
                           // turn right
                           command |= 0b00000001;
                       }
                       if (_joystickYposition > 20)
                       {
                           // foreward
                           command |= 0b00000100;
                       }
                       else if (_joystickYposition < -20)
                       {
                           // foreward
                           command |= 0b00001000;
                       }
                       BTCom.SendData(command);
                       Command = Convert.ToString(command, 2).PadLeft(8, '0');
                       return true;
                   });
               }
           });
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
            set { _message = value;  NotifyPropertyChanged(nameof(Message));}
        }

        private int _joystickXposition;
        public int JoystickXposition
        {
            get { return _joystickXposition; }
            set { _joystickXposition = value; NotifyPropertyChanged(nameof(JoystickXposition));}
        }

        private int _joystickYposition;
        public int JoystickYposition
        {
            get { return _joystickYposition; }
            set { _joystickYposition = value; NotifyPropertyChanged(nameof(JoystickYposition));}
        }

        //private int _joystickDistance;
        //public int JoystickDistance
        //{
        //    get { return _joystickDistance; }
        //    set { _joystickDistance = value; NotifyPropertyChanged(nameof(JoystickDistance)); }
        //}

        //private int _joystickAngle;        

        //public int JoystickAngle
        //{
        //    get { return _joystickAngle; }
        //    set { _joystickAngle = value; NotifyPropertyChanged(nameof(JoystickAngle)); }
        //}

        public event PropertyChangedEventHandler PropertyChanged;

        void NotifyPropertyChanged(string name)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
        }
    }
}
