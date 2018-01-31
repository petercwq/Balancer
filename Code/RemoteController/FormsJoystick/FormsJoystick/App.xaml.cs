using FormsJoystick.Communication;
using Xamarin.Forms;

namespace FormsJoystick
{
    public partial class App : Application
    {
        public App()
        {
            InitializeComponent();
            MainPage = new Views.MainPage();
        }

        protected override void OnStart()
        {
            // Handle when your app starts
        }

        protected override void OnSleep()
        {
            var btcom = DependencyService.Get<IBluetoothCom>();
            // Handle when your app sleeps
            if (btcom != null && btcom.Connected)
                btcom.Close();
        }

        protected override void OnResume()
        {
            // Handle when your app resumes
        }
    }
}
