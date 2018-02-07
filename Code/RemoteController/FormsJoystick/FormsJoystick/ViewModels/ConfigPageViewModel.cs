﻿using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Text;
using System.Windows.Input;
using Xamarin.Forms;

namespace FormsJoystick.ViewModels
{
    public class ConfigPageViewModel : BaseViewModel
    {
        public ICommand CalibGyroCommand { protected set; get; }

        public ICommand CalibAccCommand { protected set; get; }

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
            RegisterProcesser(0x02,ret => P = ret / 10.0f );
            RegisterProcesser(0x03, ret => I = ret / 10.0f);
            RegisterProcesser(0x04, ret => D = ret / 10.0f);
            RegisterProcesser(0x07, ret => Turn = ret);
            RegisterProcesser(0x08, ret => Move = ret);

            ResetCommand = new Command(() =>
            {
                P = 15;
                I = 1.5f;
                D = 30;
                Turn = 30;
                Move = 100;
            });

            CalibGyroCommand = new Command(() =>
            {
                PushNewCommand(new CmdPacket() { Command = 0x06, Value = 0x01 });
            });

            CalibAccCommand = new Command(() =>
            {
                PushNewCommand(new CmdPacket() { Command = 0x06, Value = 0x03 });
            });

            ReadCommand = new Command(() =>
            {
                PushNewCommand(new CmdPacket() { Command = 0x02, Value = 0xff});
                PushNewCommand(new CmdPacket() { Command = 0x03, Value = 0xff});
                PushNewCommand(new CmdPacket() { Command = 0x04, Value = 0xff});
                PushNewCommand(new CmdPacket() { Command = 0x07, Value = 0xff});
                PushNewCommand(new CmdPacket() { Command = 0x08, Value = 0xff});
            });

            WriteCommand = new Command(() =>
            {
                PushNewCommand(new CmdPacket() { Command = 0x02, Value = (byte)(P * 10)});
                PushNewCommand(new CmdPacket() { Command = 0x03, Value = (byte)(I * 10)});
                PushNewCommand(new CmdPacket() { Command = 0x04, Value = (byte)(D * 10)});
                PushNewCommand(new CmdPacket() { Command = 0x07, Value = (byte)(Turn)});
                PushNewCommand(new CmdPacket() { Command = 0x08, Value = (byte)(Move)});
            });

            ResetCommand.Execute(null);
        }
    }
}
