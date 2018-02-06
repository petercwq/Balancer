using FormsJoystick.Communication;
using System;
using System.Collections.Concurrent;
using System.ComponentModel;
using Xamarin.Forms;
using System.Threading.Tasks;

namespace FormsJoystick.ViewModels
{
    public class CmdPacket
    {
        public CmdPacket()
        {
            Timestamp = DateTime.Now.Ticks;
        }

        public byte Command;
        public byte Value;
        public long Timestamp;

        public bool HasReturn;
        public Action<byte> Callback;
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

        protected void PushNewCommand(CmdPacket packet)
        {
            ToSend.Enqueue(packet);
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
                            if (Math.Abs(DateTime.Now.Ticks - pack.Timestamp) < 5 * 10000000)
                            {
                                if (!BTCom.SendData(new byte[] { pack.Command, pack.Value }))
                                {
                                    BTCom.Close();
                                }
                                else if (pack.HasReturn)
                                {
                                    var buffer = new byte[2];
                                    var index = 0;
                                    var start = DateTime.Now;
                                    while (BTCom.Connected && DateTime.Now - start < TimeSpan.FromMilliseconds(10))
                                    {
                                        var readed = BTCom.ReadData(buffer, index, buffer.Length - index);
                                        index += readed;

                                        if (index == buffer.Length)
                                        {
                                            if (buffer[0] == pack.Command)
                                            {
                                                pack.Callback?.Invoke(buffer[1]);
                                                break;
                                            }
                                            else
                                            {
                                                index = 0;
                                            }
                                        }
                                        Task.Delay(2).Wait();
                                    }
                                }
                            }
                        }
                        else
                        {
                            break;
                        }
                    }
                }
                return true;
            });
        }
    }
}
