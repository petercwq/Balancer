using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Text;
using System.Windows.Input;
using Xamarin.Forms;

namespace FormsJoystick.ViewModels
{
    public class ConfigPageViewModel : BaseViewModel
    {
        public ICommand CalibCommand { protected set; get; }

        public ICommand ReadCommand { protected set; get; }

        public ICommand WriteCommand { protected set; get; }

        public ICommand ResetCommand { protected set; get; }

        private float p, i, d, turnspeed, movespeed;

        public float P { get { return p; } set { p = value; NotifyPropertyChanged(nameof(P)); } }

        public float I { get { return i; } set { i = value; NotifyPropertyChanged(nameof(I)); } }

        public float D { get { return d; } set { d = value; NotifyPropertyChanged(nameof(D)); } }

        public float Turn { get { return turnspeed; } set { turnspeed = value; NotifyPropertyChanged(nameof(Turn)); } }

        public float Move { get { return movespeed; } set { movespeed = value; NotifyPropertyChanged(nameof(Move)); } }

        public ConfigPageViewModel()
        {
            AttachCommandAction(Commands.GetParams, ret => { P = ret.ReadFloatArg(); I = ret.ReadFloatArg();D = ret.ReadFloatArg();Turn = ret.ReadFloatArg();Move = ret.ReadFloatArg(); });


            ResetCommand = new Command(() =>
            {
                P = 15;
                I = 1f;
                D = 5;
                Turn = 30;
                Move = 30;
            });

            CalibCommand = new Command(() =>
            {
                SendCommand(new CommandMessenger.SendCommand((int)Commands.Calibrate, (short)0x03));
            });

            ReadCommand = new Command(() =>
            {
                SendCommand(new CommandMessenger.SendCommand((int)Commands.GetParams));
            });

            WriteCommand = new Command(() =>
            {
                var command = new CommandMessenger.SendCommand((int)Commands.SetParams);
                command.AddArgument(P);
                command.AddArgument(I);
                command.AddArgument(D);
                command.AddArgument(Turn);
                command.AddArgument(Move);

                SendCommand(command);
            });

            ResetCommand.Execute(null);
            QueueCommand(new CommandMessenger.SendCommand((int)Commands.GetParams));
        }
    }
}
