using System;
using System.Threading.Tasks;

namespace FormsJoystick.Communication
{
    public interface IBluetoothCom
    {
        bool FindDevice(string deviceName);
        Task<bool> ConnectAsync();
        void Close();
        bool SendData(byte command);
        bool SendData(byte[] command, int offset = 0, int count = 0);
        bool Connected { get; }
        int ReadData(byte[] buffer, int offset, int count);
    }
}
