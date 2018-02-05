using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Text;
using System.Windows.Input;

namespace FormsJoystick.ViewModels
{
    public class BalancerState
    {
        public float Pid_P;
        public float Pid_I;
        public float Pid_D;
        public int Turn_Speed;
        public int Move_Speed;
    }


    public class ConfigPageViewModel : BaseViewModel
    {
        public ICommand CalibGyroCommand { protected set; get; }
        public ICommand CalibAccCommand { protected set; get; }
        public ICommand ReadCommand { protected set; get; }
        public ICommand WriteCommand { protected set; get; }

        public ConfigPageViewModel()
        {
            
        }
    }
}
