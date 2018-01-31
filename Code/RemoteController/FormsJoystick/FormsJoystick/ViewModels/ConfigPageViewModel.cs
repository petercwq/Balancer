using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Text;

namespace FormsJoystick.ViewModels
{
    public class BalancerState
    {
        public float Pid_P;
        public float Pid_I;
        public float Pid_D;

        public float Voltage;
        public int Turn_Speed;
        public int Move_Speed;
    }


    public class ConfigPageViewModel : BaseViewModel
    {

    }
}
