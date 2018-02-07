using FormsJoystick.Communication;
using System;
using System.Collections.Concurrent;
using System.ComponentModel;
using Xamarin.Forms;
using System.Threading.Tasks;
using System.Collections.Generic;

namespace FormsJoystick.ViewModels
{
    public class CmdPacket
    {
        public const byte START = (byte)'$';
        public const int LEN = 3;

        public CmdPacket()
        {
            Timestamp = DateTime.Now.Ticks;
        }

        public byte Command;
        public byte Value;
        public long Timestamp;

        //public bool HasReturn;
        //public Action<byte> Callback;
    }

    public abstract class BaseViewModel : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        protected void NotifyPropertyChanged(string name)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
        }

        protected static IBluetoothCom BTCom = DependencyService.Get<IBluetoothCom>();
        static ConcurrentQueue<CmdPacket> ToSend = new ConcurrentQueue<CmdPacket>();
        static Queue<byte> ToProcess = new Queue<byte>();
        static Dictionary<byte, Action<byte>> Processers = new Dictionary<byte, Action<byte>>();

        private byte[] buffer = new byte[CmdPacket.LEN];

        protected void PushNewCommand(CmdPacket packet)
        {
            ToSend.Enqueue(packet);
        }

        protected void RegisterProcesser(byte command, Action<byte> processer)
        {
            Processers[command] = processer;
        }

        public BaseViewModel()
        {
            Device.StartTimer(TimeSpan.FromMilliseconds(20), () =>
            {
                if (BTCom.Connected)
                {
                    while (!ToSend.IsEmpty)
                    {
                        CmdPacket pack = null;
                        if (ToSend.TryDequeue(out pack))
                        {
                            //if (Math.Abs(DateTime.Now.Ticks - pack.Timestamp) < 5 * 10000000)
                            //{
                                if (!BTCom.SendData(new byte[] { CmdPacket.START, pack.Command, pack.Value }))
                                {
                                    BTCom.Close();
                                }                               
                            //}
                        }
                        else
                        {
                            break;
                        }
                    }
                }
                return true;
            });

            Device.StartTimer(TimeSpan.FromMilliseconds(10), () =>
            {
                if (BTCom.Connected)
                {
                    var readed = 0;
                    do
                    {
                        readed = BTCom.ReadData(buffer, 0, buffer.Length);
                        for (int i = 0; i < readed; i++)
                            ToProcess.Enqueue(buffer[i]);

                    } while (readed == buffer.Length);

                    while(ToProcess.Count >= CmdPacket.LEN)
                    {
                        if (ToProcess.Dequeue() != '$')
                        {
                            continue;
                        }
                        else
                        {
                            var cmd = ToProcess.Dequeue();
                            var val = ToProcess.Dequeue();
                            if (Processers.ContainsKey(cmd))
                                Processers[cmd]?.Invoke(val);
                        }
                    }
                }
                return true;
            });
        }
    }
}
